"""Scribit wall geometry: XY ↔ cord-length conversion and segmented moves."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


def xy_to_lr(x_mm: float, y_mm: float, D_mm: float) -> Tuple[float, float]:
    """Convert wall XY (mm) to left/right cord lengths (mm)."""
    L = math.hypot(x_mm, y_mm)
    R = math.hypot(D_mm - x_mm, y_mm)
    return L, R


# ---------------------------------------------------------------------------
# Extended (calibration-aware) forward model
# ---------------------------------------------------------------------------

@dataclass
class RobotProfile:
    """Robot-intrinsic calibration parameters — fit once per hardware unit."""
    robot_id: str = ""
    h_pen_mm: float = 0.0        # pen tip vertical offset below cable junction (mm)
    k_L: float = 1.0             # left-axis steps/mm scale correction
    k_R: float = 1.0             # right-axis steps/mm scale correction
    alpha_L: float = 0.0         # left spool non-linearity coefficient (mm⁻¹)
    alpha_R: float = 0.0         # right spool non-linearity coefficient (mm⁻¹)
    fit_rms_mm: float = 0.0
    n_measurements: int = 0
    fitted_at: str = ""


@dataclass
class WallProfile:
    """Wall-extrinsic calibration parameters — refit for each installation."""
    robot_id: str = ""
    wall_id: int = 0
    D_mm: float = 1860.0         # effective nail separation (mm)
    dx_offset_mm: float = 0.0   # starting-position X correction (mm)
    dy_offset_mm: float = 0.0   # starting-position Y correction (mm)
    fit_rms_mm: float = 0.0
    n_measurements: int = 0
    fitted_at: str = ""


def _solve_junction(
    x_pen: float,
    y_pen: float,
    D_mm: float,
    h_pen: float,
) -> Tuple[float, float]:
    """
    Solve for cable-junction (x_j, y_j) such that when the body hangs at its
    natural tilt the pen tip lands at (x_pen, y_pen).

    With e_pen = 0 (pen on body centreline):
        x_pen = x_j - h_pen * sin(θ)
        y_pen = y_j + h_pen * cos(θ)
        θ     = atan2(D - 2*x_j, 2*y_j)

    Solved by Newton iteration (converges in ~5 steps from the naive init).
    Returns (x_j, y_j).
    """
    if h_pen == 0.0:
        return x_pen, y_pen

    x_j = x_pen
    y_j = max(y_pen - h_pen, 1.0)  # initial guess

    for _ in range(20):
        theta = math.atan2(D_mm - 2.0 * x_j, 2.0 * y_j)
        sin_t = math.sin(theta)
        cos_t = math.cos(theta)

        # Residuals
        fx = x_j - h_pen * sin_t - x_pen
        fy = y_j + h_pen * cos_t - y_pen

        # Jacobian (partial derivatives of θ wrt x_j and y_j)
        denom = (D_mm - 2.0 * x_j) ** 2 + (2.0 * y_j) ** 2
        if denom < 1e-12:
            break
        dtheta_dxj = -2.0 * 2.0 * y_j / denom          # d(atan2)/dx_j
        dtheta_dyj = 2.0 * (D_mm - 2.0 * x_j) / denom  # d(atan2)/dy_j

        # ∂fx/∂x_j, ∂fx/∂y_j
        dfx_dxj = 1.0 - h_pen * cos_t * dtheta_dxj
        dfx_dyj = -h_pen * cos_t * dtheta_dyj
        # ∂fy/∂x_j, ∂fy/∂y_j
        dfy_dxj = h_pen * sin_t * dtheta_dxj
        dfy_dyj = 1.0 + h_pen * sin_t * dtheta_dyj

        det = dfx_dxj * dfy_dyj - dfx_dyj * dfy_dxj
        if abs(det) < 1e-15:
            break

        dx_j = -(dfy_dyj * fx - dfx_dyj * fy) / det
        dy_j = -(dfx_dxj * fy - dfy_dxj * fx) / det

        x_j += dx_j
        y_j += dy_j

        if math.hypot(dx_j, dy_j) < 1e-6:
            break

    return x_j, y_j


def xy_to_lr_calibrated(
    x_mm: float,
    y_mm: float,
    robot: RobotProfile,
    wall: WallProfile,
) -> Tuple[float, float]:
    """
    Extended forward model: pen position → commanded cable lengths,
    incorporating pen offset, nail separation, start-offset correction,
    and per-axis scale (+ optional spool non-linearity).
    """
    # 1. Apply starting-position correction
    x_w = x_mm + wall.dx_offset_mm
    y_w = y_mm + wall.dy_offset_mm

    # 2. Solve for cable-junction position
    x_j, y_j = _solve_junction(x_w, y_w, wall.D_mm, robot.h_pen_mm)

    # 3. Ideal cable lengths at the junction
    L = math.hypot(x_j, y_j)
    R = math.hypot(wall.D_mm - x_j, y_j)

    # 4. Per-axis scale + optional spool non-linearity
    L_cmd = robot.k_L * L + robot.alpha_L * L * L
    R_cmd = robot.k_R * R + robot.alpha_R * R * R

    return L_cmd, R_cmd


# ---------------------------------------------------------------------------
# G-code move helpers (ideal and calibrated variants)
# ---------------------------------------------------------------------------

def wall_xy_to_lr_delta_g1(
    cur_xy: Tuple[float, float],
    next_xy: Tuple[float, float],
    D_mm: float,
    feed: int,
    robot: Optional[RobotProfile] = None,
    wall: Optional[WallProfile] = None,
) -> Tuple[str, Tuple[float, float]]:
    """
    Emit a single incremental G1 move in (dL, -dR) space from cur_xy to next_xy.
    When robot/wall profiles are provided the extended forward model is used;
    otherwise falls back to the ideal polargraph.
    Returns (gcode_line, updated_xy).
    """
    x0, y0 = cur_xy
    x1, y1 = next_xy
    if robot is not None and wall is not None:
        L0, R0 = xy_to_lr_calibrated(x0, y0, robot, wall)
        L1, R1 = xy_to_lr_calibrated(x1, y1, robot, wall)
    else:
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
    robot: Optional[RobotProfile] = None,
    wall: Optional[WallProfile] = None,
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
        line, new_xy = wall_xy_to_lr_delta_g1(cur_xy, target_xy, D_mm, feed, robot, wall)
        return ([line], new_xy)

    n = max(1, int(math.ceil(dist / max_step_mm)))
    lines: List[str] = []
    xy = cur_xy
    for i in range(1, n + 1):
        t = i / n
        mid = (x0 + dx * t, y0 + dy * t)
        line, xy = wall_xy_to_lr_delta_g1(xy, mid, D_mm, feed, robot, wall)
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
