"""
Generate the 5×5 calibration grid:
  - grid5x5.gcode  — draws 25 small + crosses on the wall
  - grid5x5.json   — intended wall XY of each cross centre
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

from .config import D_MM_DEFAULT, STARTING_X, STARTING_Y
from .gcode import (
    CarouselState,
    gcode_dwell,
    gcode_header,
    gcode_home_carousel,
    gcode_pen_down,
    gcode_pen_select_ccw,
    gcode_pen_up,
    strip_comments,
)
from .geometry import move_xy_segmented, wall_xy_to_lr_delta_g1

# Cross arm half-length (mm) — each cross is 20 mm × 20 mm
CROSS_HALF = 10.0

# Grid layout: row/column fractions of D along horizontal and vertical axes.
# Spans ±0.45·D horizontally (centre at 0.5·D) and 0.30–0.75·D vertically.
_ROW_FRACS = (0.30, 0.39, 0.525, 0.61, 0.75)
_COL_FRACS = (0.05, 0.275, 0.50, 0.725, 0.95)


def grid_cross_centres(D_mm: float) -> Dict[str, Tuple[float, float]]:
    """
    Return {label: (x_mm, y_mm)} for every cross centre in the 5×5 grid.
    Label format: "r{row}c{col}" with row 0 at top.
    """
    centres: Dict[str, Tuple[float, float]] = {}
    for ri, ry in enumerate(_ROW_FRACS):
        for ci, cx in enumerate(_COL_FRACS):
            label = f"r{ri}c{ci}"
            centres[label] = (cx * D_mm, ry * D_mm)
    return centres


def _cross_gcode(
    centre: Tuple[float, float],
    cur_xy: Tuple[float, float],
    D_mm: float,
    f_travel: int,
    f_draw: int,
    f_z: int,
    pen: int,
    travel_step_mm: float,
    draw_step_mm: float,
    dwell_s: float,
    st: CarouselState,
    emit_comments: bool,
) -> Tuple[List[str], Tuple[float, float]]:
    """Emit G-code to draw one + cross and return updated position."""
    cx, cy = centre
    lines: List[str] = []

    # Four arm endpoints: left, right, top, bottom
    arms = [
        (cx - CROSS_HALF, cy),
        (cx + CROSS_HALF, cy),
        (cx, cy - CROSS_HALF),
        (cx, cy + CROSS_HALF),
    ]

    # Pen-up travel to cross centre
    if emit_comments:
        lines.append(f"; --- travel to cross centre ({cx:.1f}, {cy:.1f}) ---")
    mvs, cur_xy = move_xy_segmented(cur_xy, (cx, cy), D_mm, f_travel, travel_step_mm)
    lines += mvs

    # Draw horizontal stroke: left → right (pen down between)
    if emit_comments:
        lines.append("; --- draw horizontal arm ---")
    mvs, cur_xy = move_xy_segmented(cur_xy, arms[0], D_mm, f_travel, travel_step_mm)
    lines += mvs
    lines += gcode_pen_down()
    mvs, cur_xy = move_xy_segmented(cur_xy, arms[1], D_mm, f_draw, draw_step_mm)
    lines += mvs
    lines += gcode_pen_up(pen, f_z, st)

    # Draw vertical stroke: top → bottom (pen down between)
    if emit_comments:
        lines.append("; --- draw vertical arm ---")
    mvs, cur_xy = move_xy_segmented(cur_xy, arms[2], D_mm, f_travel, travel_step_mm)
    lines += mvs
    lines += gcode_pen_down()
    mvs, cur_xy = move_xy_segmented(cur_xy, arms[3], D_mm, f_draw, draw_step_mm)
    lines += mvs
    lines += gcode_pen_up(pen, f_z, st)

    return lines, cur_xy


def generate_pattern(
    D_mm: float = D_MM_DEFAULT,
    pen: int = 1,
    f_travel: int = 600,
    f_draw: int = 300,
    f_z: int = 600,
    travel_step_mm: float = 5.0,
    draw_step_mm: float = 1.0,
    dwell_s: float = 0.0,
    home_carousel: bool = True,
    return_after_finish: bool = True,
    gcode_comments: bool = False,
) -> Tuple[List[str], Dict[str, Tuple[float, float]]]:
    """
    Build the calibration G-code and the intent dict.

    Returns:
        (gcode_lines, centres_dict)
        centres_dict maps label → (x_mm, y_mm) intended wall position.
    """
    centres = grid_cross_centres(D_mm)

    st = CarouselState()
    lines: List[str] = []
    lines += gcode_header()
    if home_carousel:
        lines += gcode_home_carousel(st)
        lines += gcode_home_carousel(st)

    lines += gcode_pen_select_ccw(pen, f_z, st)

    cur_xy: Tuple[float, float] = (STARTING_X, STARTING_Y)

    # Iterate row-major so the robot sweeps top-to-bottom, left-to-right
    for ri in range(5):
        for ci in range(5):
            label = f"r{ri}c{ci}"
            cross_lines, cur_xy = _cross_gcode(
                centre=centres[label],
                cur_xy=cur_xy,
                D_mm=D_mm,
                f_travel=f_travel,
                f_draw=f_draw,
                f_z=f_z,
                pen=pen,
                travel_step_mm=travel_step_mm,
                draw_step_mm=draw_step_mm,
                dwell_s=dwell_s,
                st=st,
                emit_comments=gcode_comments,
            )
            lines += cross_lines

    if return_after_finish:
        if gcode_comments:
            lines.append("; --- return to start position ---")
        mvs, cur_xy = move_xy_segmented(
            cur_xy, (STARTING_X, STARTING_Y), D_mm, f_travel, travel_step_mm
        )
        lines += mvs

    if not gcode_comments:
        lines = strip_comments(lines)

    return lines, centres


def write_pattern_files(
    out_gcode: str | Path = "grid5x5.gcode",
    out_json: str | Path = "grid5x5.json",
    D_mm: float = D_MM_DEFAULT,
    pen: int = 1,
    f_travel: int = 600,
    f_draw: int = 300,
    f_z: int = 600,
    travel_step_mm: float = 5.0,
    draw_step_mm: float = 1.0,
    home_carousel: bool = True,
    return_after_finish: bool = True,
    gcode_comments: bool = False,
) -> None:
    lines, centres = generate_pattern(
        D_mm=D_mm,
        pen=pen,
        f_travel=f_travel,
        f_draw=f_draw,
        f_z=f_z,
        travel_step_mm=travel_step_mm,
        draw_step_mm=draw_step_mm,
        home_carousel=home_carousel,
        return_after_finish=return_after_finish,
        gcode_comments=gcode_comments,
    )

    Path(out_gcode).write_text("\n".join(lines) + "\n", encoding="utf-8")

    intent = {
        "D_mm": D_mm,
        "crosses": {
            label: {"x_mm": round(x, 3), "y_mm": round(y, 3)}
            for label, (x, y) in centres.items()
        },
    }
    Path(out_json).write_text(json.dumps(intent, indent=2), encoding="utf-8")
