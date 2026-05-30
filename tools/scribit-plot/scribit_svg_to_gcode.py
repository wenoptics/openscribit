#!/usr/bin/env python3
"""
svg2scribit_gcode.py

Convert stroked SVG paths into Scribit-compatible G-code:
- drawing.gcode: draws the SVG paths (pen down on strokes)
- bbox_dots.gcode: marks the mapped drawing bounding box corners as dots

Notes / assumptions:
- SVG coordinate system is y-down; Scribit "wall XY" here is also treated as y-down.
- Wall origin is top-left; X to the right; Y downward.
- Scribit kinematics are expressed as delta lengths of left/right cords (L/R).
- G-code output uses:
  - G21  : mm units
  - G91  : incremental moves
  - M17  : enable motors
  - G101 : pen down (we use it 3x for reliable latch)
  - G4 S : dwell in seconds (for dots)

Requires:
  pip install svgpathtools
"""
# /// script
# dependencies = [
#     "svgpathtools",
# ]
# ///

from __future__ import annotations

import argparse
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, Iterable, Iterator, List, Optional, Tuple

from svgpathtools import Path, parse_path
from svgpathtools.svg_to_paths import (
    ellipse2pathd,
    line2pathd,
    polygon2pathd,
    polyline2pathd,
    rect2pathd,
)


# Scribit default distance between nails
D_MM_DEFAULT = 1860

# Pen slot Z degrees 
PEN_SLOTS_Z: Dict[int, int] = {1: 89, 2: 161, 3: 233, 4: 305}

# Proven reliable Z reference after homing (G77)
Z_AFTER_G77 = -56.0

# You may use your own starting position, e.g. D/2, D/2.
_ = math.sqrt(1240**2 - 1000**2)
STARTING_X, STARTING_Y = (1000, _)

@dataclass
class CarouselState:
    """Tracks the commanded carousel Z (degrees) so we can force CCW-only pen changes."""
    z: Optional[float] = None


def ccw_only_target(current_z: Optional[float], slot_z: float) -> float:
    """Return an absolute Z target that moves CCW-only by adding +360 as needed."""
    if current_z is None:
        # Best effort (should be avoided by homing once at the top of the file)
        return float(slot_z)
    target = float(slot_z)
    while target < current_z:
        target += 360.0
    return target


# ----------------------------
# Small utilities
# ----------------------------

def clamp_int(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))

def clamp_float(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def parse_color(s: Optional[str]) -> Optional[str]:
    """Normalize common SVG color formats. Returns None for 'none'/'transparent'/missing."""
    if not s:
        return None
    s = s.strip().lower()
    if s in ("none", "transparent"):
        return None
    if s.startswith("#"):
        if len(s) == 4:  # #rgb -> #rrggbb
            r, g, b = s[1], s[2], s[3]
            return f"#{r}{r}{g}{g}{b}{b}"
        if len(s) == 7:
            return s
        return s  # unknown length, keep as-is
    m = re.match(r"rgb\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\)", s)
    if m:
        r, g, b = (int(m.group(1)), int(m.group(2)), int(m.group(3)))
        r = clamp_int(r, 0, 255)
        g = clamp_int(g, 0, 255)
        b = clamp_int(b, 0, 255)
        return f"#{r:02x}{g:02x}{b:02x}"
    return s


# ----------------------------
# Scribit geometry & G-code
# ----------------------------

def xy_to_lr(x_mm: float, y_mm: float, D_mm: float) -> Tuple[float, float]:
    """Convert wall XY (mm) to left/right cord lengths (mm)."""
    L = math.hypot(x_mm, y_mm)
    R = math.hypot(D_mm - x_mm, y_mm)
    return L, R

def gcode_header() -> List[str]:
    return [
        "; --- header: mm units, incremental mode, motors on ---",
        "G21",   # mm units
        "G91",   # incremental positioning
        "M17",   # enable stepper motors
    ]

def gcode_home_carousel(st: CarouselState) -> List[str]:
    """Home carousel (G77) and set a known Z reference (G92 Z-56)."""
    st.z = Z_AFTER_G77
    return [
        "; --- home carousel: find Z home, set reference position ---",
        "G21",                        # mm units
        "G90",                        # absolute positioning (needed for G77)
        "M17",                        # enable stepper motors
        "G77",                        # home carousel (runs until index sensor)
        f"G92 Z{Z_AFTER_G77:g}",      # declare current Z as reference
        "G91",                        # back to incremental positioning
    ]

def gcode_pen_select_ccw(pen: int, f_z: int, st: CarouselState) -> List[str]:
    """Select a pen slot, forcing CCW-only carousel motion via +360 wrap."""
    if pen not in PEN_SLOTS_Z:
        raise ValueError(f"pen must be 1..4, got {pen}")
    slot = float(PEN_SLOTS_Z[pen])
    target = ccw_only_target(st.z, slot)
    st.z = target
    # Temporarily absolute for Z, then back to incremental (same style as original).
    return [
        f"; --- pen select: rotate carousel CCW to slot {pen} (Z={target:.3f} deg) ---",
        "G90",                            # absolute positioning for Z move
        f"G1 Z{target:.3f} F{f_z}",      # rotate carousel to target angle
        "G91",                            # back to incremental positioning
    ]

def gcode_pen_down() -> List[str]:
    # Reliable latch: 30 degrees isn't enough; do G101 three times (~90 degrees).
    return [
        "; --- pen down: engage pen (3x G101 for reliable latch) ---",
        "G101",   # engage pen latch (~30 deg)
        "G101",   # engage pen latch (~30 deg)
        "G101",   # engage pen latch (~30 deg)
    ]

def gcode_pen_up(pen: int, f_z: int, st: CarouselState) -> List[str]:
    """Pen-up is implemented by returning to the slot Z (CCW-only)."""
    lines = gcode_pen_select_ccw(pen, f_z, st)
    # Replace the pen-select comment with a pen-up comment
    lines[0] = f"; --- pen up: retract pen by returning carousel to slot {pen} ---"
    return lines

def gcode_dwell(seconds: float) -> List[str]:
    s = max(0.0, seconds)
    return [
        f"; --- dwell: pause {s:.3f} s (let pen mark the surface) ---",
        f"G4 S{s:.3f}",
    ]

def wall_xy_to_lr_delta_g1(
    cur_xy: Tuple[float, float],
    next_xy: Tuple[float, float],
    D_mm: float,
    feed: int,
) -> Tuple[str, Tuple[float, float]]:
    """
    Emit a single incremental G1 move in (dL, -dR) space, based on wall XY change.
    Returns (gcode_line, updated_xy).
    """
    x0, y0 = cur_xy
    x1, y1 = next_xy
    L0, R0 = xy_to_lr(x0, y0, D_mm)
    L1, R1 = xy_to_lr(x1, y1, D_mm)
    dL = L1 - L0
    dR = R1 - R0
    return (f"G1 X{dL:.3f} Y{-dR:.3f} F{feed}", (x1, y1))

def move_xy_segmented(
    cur_xy: Tuple[float, float],
    target_xy: Tuple[float, float],
    D_mm: float,
    feed: int,
    max_step_mm: float,
) -> Tuple[List[str], Tuple[float, float]]:
    """
    Move from cur_xy to target_xy by splitting the straight-line wall XY path into
    <= max_step_mm segments (helps avoid huge single deltas).
    """
    x0, y0 = cur_xy
    x1, y1 = target_xy
    dx = x1 - x0
    dy = y1 - y0
    dist = math.hypot(dx, dy)
    if dist <= 1e-9:
        return ([], cur_xy)
    if max_step_mm <= 0:
        # fallback: single move
        line, new_xy = wall_xy_to_lr_delta_g1(cur_xy, target_xy, D_mm, feed)
        return ([line], new_xy)

    n = max(1, int(math.ceil(dist / max_step_mm)))
    lines: List[str] = []
    xy = cur_xy
    for i in range(1, n + 1):
        t = i / n
        mid = (x0 + dx * t, y0 + dy * t)
        line, xy = wall_xy_to_lr_delta_g1(xy, mid, D_mm, feed)
        lines.append(line)
    return (lines, xy)


# ----------------------------
# SVG sampling & mapping
# ----------------------------

def sample_path_uniform_t(path: Path, n: int) -> List[complex]:
    """Uniform-in-t sampling: returns n+1 points including endpoint."""
    n = max(1, n)
    return [path.point(i / n) for i in range(n + 1)]


def split_into_continuous_subpaths(path: Path, *, eps: float = 1e-7) -> List[Path]:
    """Split an svgpathtools.Path into continuous subpaths.

    Why this exists:
      A single SVG `<path d="...">` can contain multiple `M/m` “move-to” commands.
      Many SVG tools emit one `<path>` element containing several disjoint subpaths.

      If we treat that as one continuous stroke, the converter will occasionally
      produce a very large pen-down jump between disjoint subpaths.

    Notes:
      - If svgpathtools provides `continuous_subpaths()`, we use it.
      - Otherwise we fall back to splitting whenever a segment's start does not
        match the previous segment's end (within `eps`).
    """

    # Preferred: use library helper if present.
    f = getattr(path, "continuous_subpaths", None)
    if callable(f):
        try:
            subs = f()
            # Some versions return a generator.
            subs = list(subs)
            return [sp for sp in subs if len(sp) > 0]
        except Exception:
            # Fall back to our own splitting.
            pass

    if len(path) == 0:
        return []

    out: List[Path] = []
    cur: List = []
    prev_end: Optional[complex] = None

    def close_enough(a: complex, b: complex) -> bool:
        return abs(a - b) <= eps

    for seg in path:
        # svgpathtools segments have .start and .end
        s = getattr(seg, "start", None)
        e = getattr(seg, "end", None)
        if s is None or e is None:
            # Unexpected segment type: keep it in the current subpath.
            cur.append(seg)
            prev_end = e if e is not None else prev_end
            continue

        if prev_end is not None and not close_enough(s, prev_end):
            # Discontinuity => start a new subpath.
            if cur:
                out.append(Path(*cur))
            cur = [seg]
        else:
            cur.append(seg)

        prev_end = e

    if cur:
        out.append(Path(*cur))

    return out

@dataclass(frozen=True)
class SvgToWallMapper:
    u_center: float
    v_center: float
    scale: float
    wall_cx: float
    wall_cy: float

    def map_uv(self, u: float, v: float) -> Tuple[float, float]:
        # Preserve y-down
        x = (u - self.u_center) * self.scale + self.wall_cx
        y = (v - self.v_center) * self.scale + self.wall_cy
        return (x, y)


# ----------------------------
# Main conversion pipeline
# ----------------------------

@dataclass(frozen=True)
class Args:
    svg: str
    D_mm: float
    fit_frac: float
    step_mm: float
    travel_step_mm: float
    f_travel: int
    f_draw: int
    f_z: int
    dot_dwell_s: float
    bbox_pen: int
    default_pen: int
    home_carousel: bool
    return_after_finish: bool
    gcode_comments: bool
    out_bbox: str
    out_draw: str

def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description=(
            "Convert an SVG into Scribit G-code.\n"
            "Outputs:\n"
            "  - bbox_dots.gcode : dots at the mapped bounding box corners\n"
            "  - drawing.gcode   : the actual drawing (stroked paths)\n"
        ),
    )
    p.add_argument("svg", help="Input SVG file path")

    p.add_argument("--D_mm", type=float, default=D_MM_DEFAULT,
                   help="Distance between nails in mm (Scribit D)")

    p.add_argument("--fit_frac", type=float, default=0.70,
                   help="Scale drawing to fit within fit_frac * D in both width/height")

    p.add_argument("--step_mm", type=float, default=1.0,
                   help="Pen-down step size along curves in wall mm (smaller = smoother, larger = faster)")

    p.add_argument("--travel_step_mm", type=float, default=5.0,
                   help="Pen-up max step size in wall mm when repositioning between paths")

    p.add_argument("--f_travel", type=int, default=600, help="Feed rate for pen-up travel moves")
    p.add_argument("--f_draw", type=int, default=300, help="Feed rate for pen-down drawing moves")
    p.add_argument("--f_z", type=int, default=600, help="Feed rate for Z moves (pen select / pen up)")

    p.add_argument("--dot_dwell_s", type=float, default=0.20,
                   help="Dwell time (seconds) when making bbox corner dots")

    p.add_argument("--bbox_pen", type=int, default=1,
                   help="Pen slot (1..4) used for bbox dots")
    p.add_argument("--default_pen", type=int, default=1,
                   help="Fallback pen slot (1..4) if pen mapping overflows")

    p.add_argument("--no_home_carousel", action="store_true",
                   help="Do NOT emit G77 + G92 Z-56 at file start (not recommended)")

    p.add_argument("--return-after-finish", dest="return_after_finish",
                   action="store_true", default=True,
                   help="Return robot to starting position (D/2, D/2) after finishing with pen up")
    p.add_argument("--no-return-after-finish", dest="return_after_finish",
                   action="store_false",
                   help="Do not return to starting position after finishing")

    p.add_argument("--gcode-comments", dest="gcode_comments",
                   action="store_true", default=False,
                   help="Emit '; ---' comment lines in G-code output (off by default)")

    p.add_argument("--out_bbox", default="bbox_dots.gcode",
                   help="Output filename for bbox dots G-code")
    p.add_argument("--out_draw", default="drawing.gcode",
                   help="Output filename for drawing G-code")

    return p

RENDERABLE_TAGS = frozenset(
    {"path", "polyline", "polygon", "line", "rect", "circle", "ellipse"}
)

# SVG presentation attributes that inherit from parent elements (subset relevant
# to pen selection). `style` is handled separately because CSS rules override
# presentation attributes.
INHERITABLE_PRESENTATION_ATTRS = ("stroke", "fill", "stroke-width", "opacity")


def _local_tag(tag: str) -> str:
    """Strip the XML namespace prefix from a tag name (e.g. '{ns}path' -> 'path')."""
    return tag.rsplit("}", 1)[-1]


def _parse_style(style: Optional[str]) -> Dict[str, str]:
    """Parse a CSS-style 'k:v;k:v' string into a dict."""
    out: Dict[str, str] = {}
    if not style:
        return out
    for piece in style.split(";"):
        if ":" in piece:
            k, v = piece.split(":", 1)
            out[k.strip()] = v.strip()
    return out


def resolve_inherited_attrs(
    el: ET.Element, inherited: Dict[str, str]
) -> Dict[str, str]:
    """Merge attributes inherited from ancestors with this element's own.

    `style` overrides presentation attributes on the same element, per CSS rules.
    """
    resolved = dict(inherited)
    for k in INHERITABLE_PRESENTATION_ATTRS:
        v = el.get(k)
        if v is not None:
            resolved[k] = v
    for k, v in _parse_style(el.get("style")).items():
        resolved[k] = v
    return resolved


def _element_to_path_d(el: ET.Element) -> Optional[str]:
    """Convert an SVG renderable element to a Path d-string, or None if unsupported/empty."""
    tag = _local_tag(el.tag)
    if tag == "path":
        return el.get("d") or None
    if tag == "polyline":
        return polyline2pathd(el.attrib) or None
    if tag == "polygon":
        return polygon2pathd(el.attrib) or None
    if tag == "line":
        return line2pathd(el)
    if tag == "rect":
        return rect2pathd(el.attrib)
    if tag in ("circle", "ellipse"):
        return ellipse2pathd(el.attrib)
    return None


@dataclass
class RenderableElement:
    """An SVG renderable element with its inheritance-resolved presentation attrs."""
    attrs: Dict[str, str]
    d: str
    svg_id: str


def iter_renderable_elements(root: ET.Element) -> Iterable[RenderableElement]:
    """Yield renderable SVG elements with inherited stroke/fill resolved.

    Walks the tree top-down, accumulating inheritable presentation attrs from
    each ancestor (so a `<polyline>` inside `<g stroke="red">` reports stroke="red").
    """
    counter = [0]

    def walk(el: ET.Element, inherited: Dict[str, str]) -> Iterable[RenderableElement]:
        resolved = resolve_inherited_attrs(el, inherited)
        if _local_tag(el.tag) in RENDERABLE_TAGS:
            counter[0] += 1
            d = _element_to_path_d(el)
            if d:
                svg_id = el.get("id") or f"path_{counter[0]}"
                yield RenderableElement(attrs=resolved, d=d, svg_id=svg_id)
        for child in el:
            yield from walk(child, resolved)

    yield from walk(root, {})


def effective_stroke(attrs: Dict[str, str]) -> Optional[str]:
    """Decide which stroke color to draw an element with.

    Returns:
      - normalized color (e.g. "#ff2600") if the element should be drawn
      - None if the element should be skipped (filled shape without a stroke)

    Rules (preserved from the original implementation):
      - Use stroke color if present and not "none".
      - If stroke missing/none AND fill missing/none => assume black stroke.
      - If fill is set (visible) but stroke missing/none => skip.
    """
    stroke = parse_color(attrs.get("stroke"))
    if stroke is not None:
        return stroke
    fill = parse_color(attrs.get("fill"))
    if fill is None:
        return "#000000"
    return None


class PenAssigner:
    """Maps stroke colors to pen slots (1..max_pens) in first-seen order.

    Colors past `max_pens` fall back to `default_pen`.
    """

    def __init__(self, default_pen: int, max_pens: int = 4):
        self.default_pen = default_pen
        self.max_pens = max_pens
        self.color_to_pen: Dict[str, int] = {}
        self._next_pen = 1

    def assign(self, color: str) -> int:
        if color not in self.color_to_pen and self._next_pen <= self.max_pens:
            self.color_to_pen[color] = self._next_pen
            self._next_pen += 1
        return self.color_to_pen.get(color, self.default_pen)


def load_drawable_paths(svg_path: str, default_pen: int) -> Tuple[List[Tuple[Path, int, str]], Dict[str, int]]:
    """
    Returns:
      drawable: list of (Path, pen_id, svg_id)
      color_to_pen: mapping used
    Rules:
      - Use stroke color if present and not 'none'
      - If stroke missing AND fill missing/none => assume black stroke
      - If fill is set (visible) but stroke missing => skip (likely a filled shape)
      - Map first seen colors to pens 1..4; overflow uses default_pen
      - Inheritance: stroke/fill are inherited from ancestor <g> elements, so a
        <polyline> inside <g stroke="#ff2600"> draws with #ff2600.
    """
    tree = ET.parse(svg_path)
    root = tree.getroot()

    assigner = PenAssigner(default_pen=default_pen)
    drawable: List[Tuple[Path, int, str]] = []

    for el in iter_renderable_elements(root):
        stroke = effective_stroke(el.attrs)
        if stroke is None:
            continue
        pen = assigner.assign(stroke)
        drawable.append((parse_path(el.d), pen, el.svg_id))

    return drawable, assigner.color_to_pen

def compute_svg_bbox(drawable: Iterable[Tuple[Path, int, str]]) -> Tuple[float, float, float, float]:
    xmin = ymin = float("inf")
    xmax = ymax = float("-inf")
    any_path = False
    for p, _, _ in drawable:
        any_path = True
        bx0, bx1, by0, by1 = p.bbox()
        xmin = min(xmin, bx0)
        xmax = max(xmax, bx1)
        ymin = min(ymin, by0)
        ymax = max(ymax, by1)
    if not any_path:
        raise SystemExit("No drawable paths.")
    return xmin, xmax, ymin, ymax

def strip_gcode_comments(lines: List[str]) -> List[str]:
    return [l for l in lines if not l.startswith(";")]

def write_lines(path: str, lines: List[str]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

def main() -> None:
    ap = build_argparser()
    ns = ap.parse_args()
    ns.home_carousel = (not ns.no_home_carousel)

    d = vars(ns)
    d.pop("no_home_carousel", None)
    args = Args(**d)

    # Basic validation
    args_D = float(args.D_mm)
    if args_D <= 0:
        raise SystemExit("--D_mm must be > 0")
    if args.bbox_pen not in PEN_SLOTS_Z:
        raise SystemExit("--bbox_pen must be 1..4")
    if args.default_pen not in PEN_SLOTS_Z:
        raise SystemExit("--default_pen must be 1..4")
    if not (0.0 < args.fit_frac <= 1.5):
        # allow >1 a bit for experimentation, but block nonsense
        raise SystemExit("--fit_frac must be in (0, 1.5]")

    # Wall "center" target
    wall_cx = args_D / 2.0
    wall_cy = args_D / 2.0

    drawable, color_to_pen = load_drawable_paths(args.svg, args.default_pen)
    if not drawable:
        raise SystemExit("No drawable stroked paths found (stroke != none).")

    xmin, xmax, ymin, ymax = compute_svg_bbox(drawable)
    svg_w = xmax - xmin
    svg_h = ymax - ymin
    if svg_w <= 0 or svg_h <= 0:
        raise SystemExit("Degenerate SVG bbox (zero width/height).")

    u_center = (xmin + xmax) / 2.0
    v_center = (ymin + ymax) / 2.0
    target = args.fit_frac * args_D
    scale = min(target / svg_w, target / svg_h)

    mapper = SvgToWallMapper(
        u_center=u_center,
        v_center=v_center,
        scale=scale,
        wall_cx=wall_cx,
        wall_cy=wall_cy,
    )

    # ---------- (1) bbox dots ----------
    bbox_corners_wall = [
        mapper.map_uv(xmin, ymin),
        mapper.map_uv(xmax, ymin),
        mapper.map_uv(xmax, ymax),
        mapper.map_uv(xmin, ymax),
    ]
    wall_bbox_left = bbox_corners_wall[0][0]
    wall_bbox_top = bbox_corners_wall[0][1]
    wall_bbox_right = bbox_corners_wall[1][0]
    wall_bbox_bottom = bbox_corners_wall[2][1]

    g_bbox: List[str] = []
    g_bbox += gcode_header()
    st_bbox = CarouselState()
    if args.home_carousel:
        g_bbox += gcode_home_carousel(st_bbox)
        g_bbox += gcode_home_carousel(st_bbox)

    cur_xy = (STARTING_X, STARTING_Y)
    pen = args.bbox_pen

    g_bbox += gcode_pen_select_ccw(pen, args.f_z, st_bbox)
    corner_labels = ["top-left", "top-right", "bottom-right", "bottom-left"]
    for label, xy in zip(corner_labels, bbox_corners_wall):
        g_bbox.append(f"; --- travel to bbox corner: {label} ({xy[0]:.2f}, {xy[1]:.2f}) mm ---")
        lines, cur_xy = move_xy_segmented(
            cur_xy, xy, args_D, args.f_travel, max_step_mm=args.travel_step_mm
        )
        g_bbox += lines
        g_bbox += gcode_pen_down()
        g_bbox += gcode_dwell(args.dot_dwell_s)
        g_bbox += gcode_pen_up(pen, args.f_z, st_bbox)

    if args.return_after_finish:
        g_bbox.append("; --- return to start position after bbox dots ---")
        lines, cur_xy = move_xy_segmented(
            cur_xy, (STARTING_X, STARTING_Y), args_D, args.f_travel, max_step_mm=args.travel_step_mm
        )
        g_bbox += lines

    write_lines(args.out_bbox, g_bbox if args.gcode_comments else strip_gcode_comments(g_bbox))

    # ---------- (2) drawing ----------
    g_draw: List[str] = []
    g_draw += gcode_header()
    st_draw = CarouselState()
    if args.home_carousel:
        g_draw += gcode_home_carousel(st_draw)
        g_draw += gcode_home_carousel(st_draw)
    cur_xy = (STARTING_X, STARTING_Y)

    pen_down_block = 0
    for path, pen, svg_id in drawable:

        # A single SVG <path> may contain multiple disjoint subpaths (multiple M/m).
        # If we draw them as one continuous stroke, we'd create giant pen-down jumps.
        subpaths = split_into_continuous_subpaths(path)
        if not subpaths:
            continue

        g_draw += gcode_pen_select_ccw(pen, args.f_z, st_draw)

        for sp in subpaths:
            # Estimate number of samples so wall spacing is ~ step_mm
            try:
                length_svg = sp.length(error=1e-3)
            except TypeError:
                length_svg = sp.length()

            length_wall = length_svg * scale
            n = max(1, int(math.ceil(length_wall / max(1e-9, args.step_mm))))

            pts = sample_path_uniform_t(sp, n)
            poly_wall = [mapper.map_uv(pt.real, pt.imag) for pt in pts]
            if len(poly_wall) < 2:
                continue

            # Reposition (pen-up), segmented
            g_draw.append(
                f"; --- travel (pen-up) to subpath start: "
                f"({poly_wall[0][0]:.2f}, {poly_wall[0][1]:.2f}) mm ---"
            )
            lines, cur_xy = move_xy_segmented(
                cur_xy, poly_wall[0], args_D, args.f_travel, max_step_mm=args.travel_step_mm
            )
            g_draw += lines

            pen_down_block += 1
            g_draw.append(
                f"; --- draw stroke #{pen_down_block:03d}: svg_id={svg_id} pen={pen} "
                f"pts={len(poly_wall)} start=({poly_wall[0][0]:.2f},{poly_wall[0][1]:.2f}) "
                f"end=({poly_wall[-1][0]:.2f},{poly_wall[-1][1]:.2f}) ---"
            )

            g_draw += gcode_pen_down()

            # Emit drawing as per-point deltas in LR space
            for xy in poly_wall[1:]:
                line, cur_xy = wall_xy_to_lr_delta_g1(cur_xy, xy, args_D, args.f_draw)
                g_draw.append(line)

            g_draw += gcode_pen_up(pen, args.f_z, st_draw)

    if args.return_after_finish:
        g_draw.append("; --- return to start position after drawing ---")
        lines, cur_xy = move_xy_segmented(
            cur_xy, (STARTING_X, STARTING_Y), args_D, args.f_travel, max_step_mm=args.travel_step_mm
        )
        g_draw += lines

    write_lines(args.out_draw, g_draw if args.gcode_comments else strip_gcode_comments(g_draw))

    print(f"Wrote: {args.out_bbox}")
    print(f"Wrote: {args.out_draw}")
    print(f"D_mm={args_D:.1f} scale={scale:.6f} fit_frac={args.fit_frac} step_mm={args.step_mm} travel_step_mm={args.travel_step_mm}")
    print(f"Color->pen map: {color_to_pen}")
    bbox_w = wall_bbox_right - wall_bbox_left
    bbox_h = wall_bbox_bottom - wall_bbox_top

    print(
        "bbox margins: "
        f"left={(wall_bbox_left / args_D) * 100:.2f}% ({wall_bbox_left:.1f}mm) "
        f"right={((args_D - wall_bbox_right) / args_D) * 100:.2f}% ({(args_D - wall_bbox_right):.1f}mm) "
        f"top={(wall_bbox_top / args_D) * 100:.2f}% ({wall_bbox_top:.1f}mm) "
        f"bottom={((args_D - wall_bbox_bottom) / args_D) * 100:.2f}% ({(args_D - wall_bbox_bottom):.1f}mm)"
    )
    print(
        "bbox size: "
        f"width={bbox_w:.1f}mm ({(bbox_w / args_D) * 100:.2f}%) "
        f"height={bbox_h:.1f}mm ({(bbox_h / args_D) * 100:.2f}%)"
    )

    print(f"home_carousel={args.home_carousel} (disable with --no_home_carousel)")
    print(f"return_after_finish={args.return_after_finish} (disable with --no-return-after-finish)")


if __name__ == "__main__":
    main()

