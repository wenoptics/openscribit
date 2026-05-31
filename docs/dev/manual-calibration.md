# Manual Feedback Calibration for Drawing Accuracy

Status: **design / research** — not yet implemented.

This document discusses a proposed manual-measurement calibration loop to
improve the dimensional accuracy of drawings produced by
[scribit_svg_to_gcode.py](../../tools/scribit_svg_to_gcode.py). It also
surveys alternative calibration strategies and explains why a manual loop is
the best near-term option.

For the existing IMU-based position bootstrap (`M777` tilt sampling, remote
solver), see [autocal.md](autocal.md). That mechanism establishes *where the
robot is* before a print; it does **not** correct the geometry the converter
uses to *plan moves*. The two systems are complementary.

---

## 1. Problem

When a drawing is produced via the SVG→G-code path, the actual dimensions on
the wall diverge from the intended dimensions by roughly **10–30 mm**, and the
error scales with drawing size. A 1 m drawing drifts visibly; a 200 mm
drawing barely does. The error is reproducible (not random walk) and is
larger near the edges of the working area than near the center.

The cause is almost certainly **systematic geometry-model error**, not motor
mis-stepping. The converter's forward kinematics
([scribit_svg_to_gcode.py:108-112](../../tools/scribit_svg_to_gcode.py#L108-L112))
is the idealised polargraph:

```python
def xy_to_lr(x_mm, y_mm, D_mm):
    L = math.hypot(x_mm, y_mm)
    R = math.hypot(D_mm - x_mm, y_mm)
    return L, R
```

This treats the robot as a point mass whose pen tip coincides with the cable
junction, with cables anchoring exactly at two nails separated by exactly
`D_mm`. Real Scribit hardware violates all of those assumptions to small but
compounding degrees.

---

## 2. Where the Ideal Model Breaks Down

Below is the catalogue of physical effects the current model omits, ordered
roughly by how much they contribute to the observed 10–30 mm drift.

### 2.1 Pen offset from the cable junction (biggest single contributor)

The pen tip is mounted on the carousel below the cable attachment point. Call
this vertical offset `h_pen` (typically several centimetres). The kinematics
controls the cable junction; the pen draws somewhere offset from that point.

When `L = R` the robot hangs straight and the pen sits directly below the
junction — so the offset is a constant `(0, +h_pen)` and is absorbed by the
starting-position constants. **When `L ≠ R` the body tilts by some angle θ,
and the pen swings horizontally by `h_pen · sin(θ)`.**

The tilt angle θ at any wall position is exactly the quantity
`predicted_pitch()` in [geometry.py:94](../../services/calibration-service/geometry.py#L94):

```
θ = atan2(D − 2x, 2y)
```

At the wall corners reachable by a fit-frac=0.7 drawing, θ can easily reach
20°–30°. With `h_pen = 50 mm`, that produces 17–25 mm of horizontal pen
drift — exactly the order of magnitude the user reports.

This effect:
- Is purely horizontal (the pen swings sideways, not up/down) to first order.
- Has *opposite sign* on left vs. right halves of the wall (compresses the
  drawing horizontally on both sides → narrower than intended).
- Vanishes near the vertical centreline.
- Cannot be removed by adjusting `D_mm` or scale — it requires a tilt-aware
  forward model.

### 2.2 Effective nail separation `D_mm`

The default `D_MM_DEFAULT = 1860`
([scribit_svg_to_gcode.py:41](../../tools/scribit_svg_to_gcode.py#L41)) is
the *intended* nail spacing for a particular tape configuration. The actual
spacing on a given wall can differ by a few millimetres depending on how the
nails were hammered, and the *effective* anchor point is not the nail tip but
where the cable bends around the eyelet — which adds a small extra offset.

A `D_mm` error of `ΔD` produces a position error that grows roughly linearly
with horizontal distance from centre; a 5 mm `D` error over a 1 m drawing
contributes a few mm of width error.

### 2.3 Starting-position offset

`STARTING_X, STARTING_Y`
([scribit_svg_to_gcode.py:50-51](../../tools/scribit_svg_to_gcode.py#L50-L51))
is the assumed pose at file start. If the robot was actually placed even a
few mm off, every absolute coordinate in the drawing is shifted by that
amount. This is a pure translation, doesn't affect shape, but does affect
where the drawing lands on the wall.

### 2.4 Spool / capstan effective radius

Cable is wound on a stepper-driven drum. The steps/mm calibration
(`M92 X29.6 Y-29.6`) assumes a fixed effective radius, but as the cable
winds on or off the drum the wound-cable diameter changes slightly, so each
motor step pays out (or reels in) a slightly different length depending on
how much cable is currently wound.

This is the classic "drum-winding" non-linearity. It is most visible when one
cable is very long (drum nearly empty) and the other very short (drum
heavily wound) — i.e. near the corners. For a single-layer wind on a thin
cable the effect is negligible (≪ 1 mm/m); for thicker cable or multi-layer
winding it can reach several mm/m at extremes. Whether this matters for
Scribit needs measurement.

### 2.5 Steps-per-mm error

`M92 X29.6 Y-29.6` is calibrated at the factory but can be slightly wrong
per-unit. A 1 % error over a 1 m move is 10 mm. Captured by fitting a single
multiplicative scale to each axis.

### 2.6 Cable stretch and sag

Cables under tension stretch elastically (Young's modulus); they also sag
under their own weight (catenary). For Scribit's short, light Dyneema-style
cords both effects are sub-mm at typical scales and can be ignored at this
stage. Worth revisiting only if all coarser errors are eliminated and
residuals are still > 1 mm.

### 2.7 Wall flatness / verticality

If the wall tilts forward/back by even 1°, the projection of the cable
geometry onto the wall plane stretches/compresses. Out of scope here — treat
the wall as a perfect vertical plane.

---

## 3. Proposed Extended Forward Model

Replace `xy_to_lr` with a parameterised model. Notation:

- `(x, y)` = intended pen position on the wall (the user's coordinate system).
- `(x_j, y_j)` = cable junction point in the wall frame.
- `θ` = body tilt angle (positive = robot tilts left).
- `D` = effective nail separation.
- `(dx_offset, dy_offset)` = translation between the user's `(x, y)` origin
  and the converter's frame (captures starting-position error).
- `h_pen` = vertical offset of pen tip below cable junction (in body frame).
  The horizontal offset is *known to be zero* — the pen carousel mounts the
  pen on the robot's horizontal centreline by design, so we drop the previous
  `e_pen` parameter from the fit.
- `k_L, k_R` = per-axis steps/mm scale corrections (multiplicative,
  nominal = 1.0).
- `α_L, α_R` = optional spool non-linearity coefficients (drum effect).

The forward model — from intended pen position `(x, y)` to commanded cable
deltas — becomes:

```
# 1. Apply starting-position correction
x_w = x + dx_offset
y_w = y + dy_offset

# 2. Solve for cable-junction position whose tilted body lands the pen at (x_w, y_w).
#    With e_pen = 0 (pen on body centreline) this simplifies to:
#      x_w = x_j − h_pen·sin(θ)
#      y_w = y_j + h_pen·cos(θ)
#      θ   = atan2(D − 2·x_j, 2·y_j)
#    Implicit in x_j, y_j; a 2D fixed-point or Newton iteration converges in
#    a few steps (initialise with x_j = x_w, y_j = y_w − h_pen).

# 3. Ideal cable lengths at the junction
L = hypot(x_j, y_j)
R = hypot(D − x_j, y_j)

# 4. Apply per-axis scale corrections (and optional spool non-linearity)
L_cmd = k_L · L  + α_L · L^2
R_cmd = k_R · R  + α_R · R^2
```

The parameters split cleanly into two groups:

**Robot-intrinsic** — physical properties of *this* Scribit unit. Do not
change when the robot is moved to a different wall. Fit **once** (per
hardware), then treat as constants.

| Param | Meaning | Typical range | Identifiable? |
|-------|---------|---------------|---------------|
| `h_pen` | pen vertical offset below cable junction | 20–80 mm | yes (tilt-dependent) |
| `k_L`, `k_R` | per-axis steps/mm scale correction | within ±2 % | yes |
| `α_L`, `α_R` | spool non-linearity | small | weak unless pattern reaches extremes |

**Wall-extrinsic** — properties of the current installation. Must be
re-fitted whenever the robot is moved or the nails are re-hammered.

| Param | Meaning | Typical range | Identifiable? |
|-------|---------|---------------|---------------|
| `D` | effective nail separation | ±10 mm of nominal | yes |
| `dx_offset`, `dy_offset` | starting-pose correction | ±20 mm | yes (translation) |

`e_pen` is omitted entirely — it is known by construction to be zero (pen
mounted on the body centreline).

Three "strong" robot-intrinsic params + three "strong" wall-extrinsic params
(plus 2 optional weak spool ones) is plenty for a manual loop with ~15
measured distances. A typical user flow re-fits only the wall-extrinsic
trio between installations.

---

## 4. Manual Feedback Calibration Loop

The proposed workflow:

```
 ┌────────────────────┐    ┌─────────────────────┐    ┌────────────────────┐
 │ 1. Draw pattern    │───▶│ 2. User measures    │───▶│ 3. Fit parameters  │
 │    (known intent)  │    │    actual distances │    │    (least-squares) │
 └────────────────────┘    └─────────────────────┘    └─────────┬──────────┘
                                                                │
                                                                ▼
                              ┌──────────────────────────────────────────┐
                              │ 4. Save calibration.json; converter      │
                              │    uses extended model from now on       │
                              └──────────────────────────────────────────┘
```

### 4.1 Calibration pattern

Goal: give every parameter strong leverage in the measurement set.

Recommended pattern: a **5×5 grid of small `+` crosses** spanning roughly
`±0.45·D` horizontally and `0.3·D` to `0.75·D` vertically (i.e. covering
most of the practical working area without grazing the edges). Each cross is
~20 mm tall × 20 mm wide; the centre of the cross is the reference point.

```
   +     +     +     +     +        (top row,    y ≈ 0.30·D)
   +     +     +     +     +
   +     +     +     +     +        (middle,     y ≈ 0.52·D)
   +     +     +     +     +
   +     +     +     +     +        (bottom row, y ≈ 0.75·D)
```

Why a grid (and not, say, a single big rectangle):

- Edges/corners alone don't separate `h_pen` from `D`. The grid gives
  intermediate points whose tilt angle is different, breaking the degeneracy.
- A grid lets the user measure any subset of distances — they don't have to
  measure all 300 pairs. Even 10–15 measurements pick out the dominant
  parameters.
- Crosses are easy to measure tape-to-cross-centre with ~1 mm precision.

The pattern G-code is generated once and shipped as e.g.
`tools/calibration_patterns/grid5x5.gcode`. The converter writes a companion
JSON file recording the *intended* wall position of every cross centre.

### 4.2 Measurement protocol

User measures *actual* distances on the wall and types them into a small
helper (CLI or notebook). The most useful measurements, in priority order:

1. **Outer rectangle width and height** — top-left to top-right, top-left to
   bottom-left. Anchors overall scale and aspect.
2. **Diagonals** — top-left to bottom-right and top-right to bottom-left.
   Sensitive to tilt-induced compression.
3. **Adjacent column spacings on each row** — picks up `h_pen` because
   tilt-driven horizontal compression varies with `y`.
4. **Spot-checks of any cross-to-cross diagonal** — sanity.

10–15 measurements is plenty. Each measurement is an entry like:

```yaml
- from: r0c0     # row 0, column 0 (top-left)
  to:   r0c4     # top-right
  mm:   1245
```

### 4.3 Parameter fitting

A least-squares fit minimises:

```
sum over all measurements:
    (measured_distance − predicted_distance(params))²
```

where `predicted_distance` uses the **extended** forward model from §3 to
compute the actual on-wall position of each cross centre given the parameter
vector, then takes the Euclidean distance between the two referenced
crosses.

`scipy.optimize.least_squares` (Levenberg-Marquardt) is already a dependency
of the calibration service and is well-suited here. The residual function is
~30 lines.

Initial guesses: nominal `D`, zero offsets, `h_pen = 50 mm`, scales = 1.0.
With reasonable measurements the fit converges in milliseconds.

Output is **two** JSON files that mirror the parameter split from §3:

`robot.json` — fitted once per Scribit unit; lives in a stable per-robot
location (e.g. `~/.scribit/<robot-id>/robot.json`):

```json
{
  "version": 1,
  "robot_id": "30aea4da06f4",
  "h_pen_mm": 48.2,
  "k_L": 1.0042,
  "k_R": 0.9991,
  "alpha_L": 0.0,
  "alpha_R": 0.0,
  "fit_rms_mm": 1.4,
  "n_measurements": 14,
  "fitted_at": "2026-05-30"
}
```

`wall.json` — fitted whenever the robot is installed on a new wall; lives
beside the per-wall config (e.g. `~/.scribit/<robot-id>/walls/<wall-id>.json`):

```json
{
  "version": 1,
  "robot_id": "30aea4da06f4",
  "wall_id": 3,
  "D_mm": 1862.3,
  "dx_offset_mm": -3.1,
  "dy_offset_mm": 4.7,
  "fit_rms_mm": 1.2,
  "n_measurements": 6,
  "fitted_at": "2026-05-30"
}
```

The converter loads both files (via `--robot-cal` and `--wall-cal` flags) and
uses the extended forward model. The two-file split makes the user flow
explicit:

- **First-time setup of a robot:** run the full grid fit (~15 measurements)
  with `D` fixed at its nominal value to solve for the robot-intrinsic
  parameters; write `robot.json`.
- **New wall (same robot):** run a **smaller** fit (~5–6 measurements,
  enough for 3 wall-extrinsic params) with `h_pen, k_L, k_R` *frozen* from
  `robot.json`; write `wall.json`. This is the common case and should be
  fast.
- **Robot has been repaired / pen carousel re-seated:** re-do the full
  robot fit.

Both files include `robot_id` so the converter can detect when a
`wall.json` was fitted against a different robot's `robot.json` and warn.

### 4.4 Validation

After the first fit, re-draw the same calibration pattern using the new
parameters and re-measure the worst few distances. The residuals should drop
to within tape-measurement noise (~1–2 mm). Iterate once if needed.

---

## 5. Alternative Calibration Approaches

### 5.1 IMU-based autocal (the current system)

**How it works:** robot performs a 150 mm square at Point Zero, reads IMU
pitch at each corner, remote service solves for the robot's absolute wall
position. See [autocal.md](autocal.md).

**What it solves:** the *position bootstrap* problem — "where am I right
now?" — and writes a `G92` to seed the cable-length counters.

**What it does *not* solve:** geometry-model errors. Every M777 sample uses
the same idealised polargraph model
([geometry.py:114](../../services/calibration-service/geometry.py#L114))
that produces the drift in the first place. The IMU has roughly 0.5°–1°
noise floor; over four samples averaged via least-squares the position
estimate is good to ~10–20 mm on a 2.5 m wall, which is *worse* than what
the manual loop can achieve and contains no information about per-pen
mechanical offsets or wall-specific `D` error.

**Conclusion:** keep IMU autocal as the boot-time position seed (it requires
no human intervention and gets within "good enough to start drawing"), but
do not expect it to fix drawing dimensions. The two systems target different
problems.

### 5.2 Camera-based fiducial calibration

Mount a webcam looking at the wall (or use a fixed wall-side camera). Robot
draws fiducials (ArUco markers, dot grid); computer vision recovers their
positions in metric units via a known reference scale; the rest of the
pipeline is identical to §4 but with photo-derived distances replacing
tape-measured ones.

**Pros:** sub-mm accuracy; very low user effort once set up; can measure all
cross-pair distances at once.

**Cons:** requires extra hardware, lens calibration, lighting; the wall
isn't always accessible to a tripod; harder to set up than a tape measure.

**Verdict:** excellent for a lab / R&D setting, overkill for end users. Not
recommended as the primary path.

### 5.3 Phone-camera photogrammetry

Variant of 5.2 where the user takes a photo of the drawn pattern with their
phone, including a reference scale (e.g. an A4 sheet taped to the wall). An
app (or notebook) extracts cross centres and computes distances. There are
mature image-processing libraries (OpenCV) that make this a weekend project.

**Pros:** no extra hardware beyond a phone; much faster than tape measuring;
records the result for later re-analysis.

**Cons:** lens distortion correction, perspective rectification, and
fiducial detection all need to be solid; an end-user app would be a real
project to build.

**Verdict:** a strong **future** upgrade to the manual loop (§4). The
parameter-fitting backend in §4 is identical — only the measurement source
changes. Build the tape-measure version first, swap in photo measurement
later.

### 5.4 Direct physical measurement of `D` and start position

Just measure the nail-to-nail distance and the robot's starting position
with a tape, plug those numbers in.

**Pros:** trivial. Catches the biggest two static errors (`D`, start
offset).

**Cons:** doesn't capture `h_pen` (the dominant residual after `D` and
offset are correct), can't measure scale-per-axis, can't see drum
non-linearity. Will reduce error from 10–30 mm to maybe 5–15 mm but not to
the few-mm range a fitted model achieves.

**Verdict:** do this as a one-line preflight check before running §4 (gives
the fit better initial conditions), but it is not a substitute.

### 5.5 Plumb-bob + tape "gold-standard" survey

Hang a plumb bob from each nail, measure horizontal distance between plumb
lines (= true `D`), then measure the pen position from each nail vertically
and horizontally for a known set of commanded moves.

**Pros:** highest possible accuracy short of a CMM.

**Cons:** slow, fiddly, requires a plumb bob and clear floor space.

**Verdict:** worth doing **once** during R&D to establish a ground-truth
reference for comparing the other methods against. Not for routine
calibration.

### 5.6 Comparison

| Method | Accuracy | User effort | Extra hardware | Captures `h_pen`/tilt | Captures D | Captures axis scale |
|---|---|---|---|---|---|---|
| IMU autocal (current) | ~10–20 mm | none | none | no | partial | no |
| Direct tape (D + start only) | ~5–15 mm | low | tape | no | yes | no |
| **Manual grid fit (§4)** | **~1–3 mm** | **medium** | **tape** | **yes** | **yes** | **yes** |
| Phone-photo grid fit | ~1–2 mm | low (after app) | phone | yes | yes | yes |
| Camera fiducial | sub-mm | very low | camera rig | yes | yes | yes |
| Plumb-bob survey | sub-mm | high | plumb + tape | yes | yes | yes |

The manual grid fit is the sweet spot for a near-term implementation.

---

## 6. Recommended Implementation Path

1. **Pattern generator.** Add `tools/scribit_calibration_pattern.py` that
   emits both `grid5x5.gcode` and `grid5x5.json` (intended cross positions in
   wall coordinates). Reuse the gcode helpers from
   [scribit_svg_to_gcode.py](../../tools/scribit_svg_to_gcode.py).

2. **Extended forward model.** Add a `Kinematics` class (or just module-level
   functions) that implements §3. Default parameters reproduce the current
   `xy_to_lr` exactly, so behaviour is unchanged until a calibration file is
   loaded.

3. **Measurement CLI** with two modes:
   - `fit-robot` — full grid fit, solves the robot-intrinsic trio
     (`h_pen, k_L, k_R`) with `D` fixed at nominal. Writes `robot.json`.
   - `fit-wall` — short fit (~5–6 measurements) with robot-intrinsic params
     frozen from `robot.json`. Writes `wall.json`.

4. **Converter integration.** Add `--robot-cal <robot.json>` and
   `--wall-cal <wall.json>` to
   [scribit_svg_to_gcode.py](../../tools/scribit_svg_to_gcode.py). When both
   are present, replace `xy_to_lr` with the calibrated extended model; warn
   if either is missing or the `robot_id` fields disagree.

5. **Validation.** Re-draw the grid, re-measure ~5 distances, confirm
   residuals dropped. Document the typical improvement in this file.

6. **Future:** swap the manual measurement step for phone-photo extraction
   (§5.3) once the math pipeline is proven.

---

## 7. Open Questions

- **How much does the drum non-linearity actually contribute?** It is
  parameterised in §3 but may be negligible. A targeted test — draw a long
  horizontal line at the top of the wall (where one cable is short and the
  other long) and measure its length — would answer this.
- **Is `e_pen` actually zero for every pen slot?** The carousel mounts the
  pen on the body centreline by construction, so `e_pen = 0` is taken as
  fact and omitted from the fit. Worth a sanity check if a drawing shows a
  per-pen horizontal shift between layers — if so, `e_pen` would need to
  become a *per-pen* robot-intrinsic parameter, not a global one.
- **Can the IMU help bootstrap the manual fit?** The four M777 samples
  encode tilt at known relative offsets — that's information about `h_pen`
  in principle. Could be used as additional residuals in the §4.3 fit, but
  IMU noise (~0.5°) is comparable to the tilt change from a 50 mm move so
  the leverage is weak.

---

## See Also

- [autocal.md](autocal.md) — existing IMU-based position bootstrap.
- [gcode-reference.md](gcode-reference.md) — coordinate system, `xy_to_lr`, G-code dialect.
- [scribit_svg_to_gcode.py](../../tools/scribit_svg_to_gcode.py) — current converter (ideal-polargraph forward model).
- [services/calibration-service/geometry.py](../../services/calibration-service/geometry.py) — tilt model used by the IMU solver; relevant for §2.1 and §3.
