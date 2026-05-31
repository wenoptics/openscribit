# scribit-plot

SVG → Scribit G-code converter, with a manual calibration system to correct dimensional drawing errors.

## Installation

```sh
cd tools/scribit-plot
pip install -e .
```

This installs two commands: `sbplot` (convert SVG → G-code) and `sbcal` (calibration workflow).

---

## `sbplot` — Convert SVG to G-code

```
sbplot <svg> [options]
```

Reads an SVG file and writes two G-code files:

- `bbox_dots.gcode` — dots at the four mapped bounding-box corners (use to confirm placement before drawing)
- `drawing.gcode` — the actual drawing

### Key options

| Option | Default | Description |
|---|---|---|
| `--D_mm` | 1860 | Nail separation in mm |
| `--fit_frac` | 0.70 | Scale drawing to fit within this fraction of D |
| `--step_mm` | 1.0 | Pen-down segment length (mm) |
| `--travel_step_mm` | 5.0 | Pen-up travel segment length (mm) |
| `--f_travel` | 600 | Feed rate for pen-up moves |
| `--f_draw` | 300 | Feed rate for pen-down drawing |
| `--f_z` | 600 | Feed rate for carousel (Z) moves |
| `--bbox_pen` | 1 | Pen slot for bounding-box dots |
| `--default_pen` | 1 | Fallback pen slot when SVG has no colour mapping |
| `--no_home_carousel` | — | Skip G77 carousel homing at file start |
| `--no-return-after-finish` | — | Do not return to start position when done |
| `--gcode-comments` | — | Include `;` comment lines in output |
| `--out_bbox` | bbox_dots.gcode | Output filename for bbox dots |
| `--out_draw` | drawing.gcode | Output filename for drawing |
| `--robot-cal` | — | Path to `robot.json` calibration profile |
| `--wall-cal` | — | Path to `wall.json` calibration profile |

`--robot-cal` and `--wall-cal` must be given together. When present, the converter uses the calibrated forward model instead of the ideal polargraph, which reduces dimensional error from ~10–30 mm down to ~1–3 mm.

### Example

```sh
# Basic conversion
sbplot artwork.svg --D_mm 1860

# With calibration profiles active
sbplot artwork.svg --robot-cal robot.json --wall-cal wall.json
```

---

## `sbcal` — Manual calibration workflow

Guides you through a tape-measure calibration loop that fits the physical parameters of your robot and wall installation. Run this once per robot (full fit) and again whenever the robot moves to a new wall (fast wall-only fit).

### Sub-commands

#### `sbcal generate-pattern`

Generates the calibration files to draw on the wall.

```
sbcal generate-pattern [--D_mm MM] [--out FILE] [--pen N]
```

Outputs:
- `grid5x5.gcode` — G-code for a 5×5 grid of small + crosses
- `grid5x5.json` — intended wall position of every cross centre

The grid spans ±0.45·D horizontally and 0.30–0.75·D vertically, covering the practical working area. Each cross is 20 mm × 20 mm; its centre is the reference point for measurements.

| Option | Default | Description |
|---|---|---|
| `--D_mm` | 1860 | Nominal nail separation (mm) |
| `--out` | grid5x5.gcode | Output G-code filename |
| `--json-out` | *(same name, .json)* | Output JSON filename |
| `--pen` | 1 | Pen slot (1–4) to use |

---

#### `sbcal fit-robot`

**Full fit — use this the first time you calibrate a robot.**

Interactively walks you through:
1. Confirming the grid has been drawn on the wall
2. Collecting ~12 tape-measure distances between cross centres
3. Fitting all six parameters: `h_pen`, `k_L`, `k_R` (robot-intrinsic) and `D`, `dx_offset`, `dy_offset` (wall-extrinsic)
4. Writing `robot.json` and `wall.json`

```
sbcal fit-robot --intent grid5x5.json [--robot-out robot.json] [--wall-out wall.json]
```

| Option | Default | Description |
|---|---|---|
| `--intent` | *(required)* | Path to `grid5x5.json` |
| `--robot-out` | robot.json | Output path for robot profile |
| `--wall-out` | wall.json | Output path for wall profile |

---

#### `sbcal fit-wall`

**Fast wall-only fit — use this when the robot moves to a new wall.**

Robot-intrinsic parameters (`h_pen`, `k_L`, `k_R`) are frozen from an existing `robot.json`. Only ~6 measurements needed to fit `D`, `dx_offset`, `dy_offset`.

```
sbcal fit-wall --intent grid5x5.json --robot robot.json [--wall-out wall.json]
```

| Option | Default | Description |
|---|---|---|
| `--intent` | *(required)* | Path to `grid5x5.json` |
| `--robot` | *(required)* | Path to existing `robot.json` |
| `--wall-out` | wall.json | Output path for wall profile |

---

#### `sbcal show`

Print the contents of existing calibration profiles.

```
sbcal show --robot robot.json --wall wall.json
```

---

## Calibration walkthrough

### First time (new robot)

```sh
# 1. Generate the calibration pattern
sbcal generate-pattern --D_mm 1860

# 2. Send grid5x5.gcode to the robot and let it draw the full grid.
#    Use 'sbcmd draw' or your preferred method.

# 3. With a tape measure, run the guided fit.
#    The tool asks for ~12 distances between cross centres.
sbcal fit-robot --intent grid5x5.json --robot-out robot.json --wall-out wall.json

# 4. Use calibration when converting SVG
sbplot artwork.svg --robot-cal robot.json --wall-cal wall.json
```

### New wall (same robot)

```sh
# 1. Generate a fresh pattern (same gcode works if D_mm hasn't changed)
sbcal generate-pattern --D_mm 1860

# 2. Draw the grid on the new wall

# 3. Run the fast wall-only fit (~6 measurements)
sbcal fit-wall --intent grid5x5.json --robot robot.json --wall-out wall_living_room.json

# 4. Plot with the new wall profile
sbplot artwork.svg --robot-cal robot.json --wall-cal wall_living_room.json
```

### Validating a fit

Re-draw the grid using the calibrated parameters and re-measure 4–5 distances. Residuals should be within 1–2 mm (tape-measure noise). If RMS > 3 mm after fitting, check that you measured centre-to-centre of the + crosses and that `grid5x5.json` matches the `.gcode` you drew.

---

## Profile file format

**`robot.json`** — saved once per robot unit:

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

**`wall.json`** — saved per wall installation:

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

`robot_id` must match between the two files. The converter warns if they disagree.

---

## How calibration works

The standard converter uses an ideal polargraph model:

```
L = hypot(x, y)
R = hypot(D − x, y)
```

This ignores several real physical effects that compound into 10–30 mm of dimensional error. The extended model corrects for:

- **Pen offset (`h_pen`)** — the pen tip sits below the cable junction; when the robot tilts the pen swings horizontally. This is the dominant error source (can reach 17–25 mm at the corners).
- **Effective nail separation (`D`)** — the true anchor spacing may differ from the nominal value by a few mm.
- **Starting-position offset (`dx_offset`, `dy_offset`)** — translates the whole drawing if the robot was placed slightly off.
- **Per-axis scale (`k_L`, `k_R`)** — corrects steps/mm error on each motor axis.

See [docs/dev/manual-calibration.md](../../docs/dev/manual-calibration.md) for the full mathematical derivation.
