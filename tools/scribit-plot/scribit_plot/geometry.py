"""Scribit wall geometry: XY ↔ cord-length conversion and segmented moves."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple


def xy_to_lr(x_mm: float, y_mm: float, D_mm: float) -> Tuple[float, float]:
    """Convert wall XY (mm) to left/right cord lengths (mm)."""
    L = math.hypot(x_mm, y_mm)
    R = math.hypot(D_mm - x_mm, y_mm)
    return L, R


def wall_xy_to_lr_delta_g1(
    cur_xy: Tuple[float, float],
    next_xy: Tuple[float, float],
    D_mm: float,
    feed: int,
) -> Tuple[str, Tuple[float, float]]:
    """
    Emit a single incremental G1 move in (dL, -dR) space from cur_xy to next_xy.
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
    <= max_step_mm segments (avoids large single deltas in cord-length space).
    """
    x0, y0 = cur_xy
    x1, y1 = target_xy
    dx = x1 - x0
    dy = y1 - y0
    dist = math.hypot(dx, dy)
    if dist <= 1e-9:
        return ([], cur_xy)
    if max_step_mm <= 0:
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


@dataclass(frozen=True)
class SvgToWallMapper:
    """Maps SVG UV coordinates to wall XY (mm), preserving y-down orientation."""
    u_center: float
    v_center: float
    scale: float
    wall_cx: float
    wall_cy: float

    def map_uv(self, u: float, v: float) -> Tuple[float, float]:
        x = (u - self.u_center) * self.scale + self.wall_cx
        y = (v - self.v_center) * self.scale + self.wall_cy
        return (x, y)
