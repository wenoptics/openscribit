"""
sbcal — Manual calibration CLI for Scribit drawing accuracy.

Sub-commands:
  generate-pattern   Write grid5x5.gcode + grid5x5.json
  fit-robot          Guided workflow: full 9-param fit (robot-intrinsic + wall-extrinsic)
  fit-wall           Guided workflow: fast 3-param wall-only fit (robot params frozen)
  show               Print currently active robot / wall profiles

Usage overview
--------------
First time (new robot):
    sbcal generate-pattern --D_mm 1860 --out grid5x5.gcode
    # print grid5x5.gcode on the robot, measure distances, then:
    sbcal fit-robot --intent grid5x5.json --robot-out robot.json [--wall-out wall.json]

New wall (same robot):
    sbcal generate-pattern --D_mm 1860 --out grid5x5.gcode
    sbcal fit-wall  --intent grid5x5.json --robot robot.json --wall-out wall.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import date
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .calibration_profile import (
    check_robot_id_match,
    default_robot_profile_path,
    default_wall_profile_path,
    load_robot_profile,
    load_wall_profile,
    save_robot_profile,
    save_wall_profile,
)
from .calibration_pattern import grid_cross_centres, write_pattern_files
from .config import D_MM_DEFAULT
from .geometry import (
    RobotProfile,
    WallProfile,
    _solve_junction,
    xy_to_lr,
    xy_to_lr_calibrated,
)

# ---------------------------------------------------------------------------
# Prediction helpers
# ---------------------------------------------------------------------------

def _predict_distance(
    label_a: str,
    label_b: str,
    centres: Dict[str, Tuple[float, float]],
    robot: RobotProfile,
    wall: WallProfile,
) -> float:
    """Predict the on-wall Euclidean distance between two cross centres."""
    xa, ya = centres[label_a]
    xb, yb = centres[label_b]

    # Apply wall offsets to get actual wall-frame positions
    ax = xa + wall.dx_offset_mm
    ay = ya + wall.dy_offset_mm
    bx = xb + wall.dx_offset_mm
    by = yb + wall.dy_offset_mm

    # Solve for junction positions (pen offset introduces tilt)
    ajx, ajy = _solve_junction(ax, ay, wall.D_mm, robot.h_pen_mm)
    bjx, bjy = _solve_junction(bx, by, wall.D_mm, robot.h_pen_mm)

    # Actual commanded cable lengths
    aL = robot.k_L * math.hypot(ajx, ajy) + robot.alpha_L * math.hypot(ajx, ajy) ** 2
    aR = robot.k_R * math.hypot(wall.D_mm - ajx, ajy) + robot.alpha_R * math.hypot(wall.D_mm - ajx, ajy) ** 2
    bL = robot.k_L * math.hypot(bjx, bjy) + robot.alpha_L * math.hypot(bjx, bjy) ** 2
    bR = robot.k_R * math.hypot(wall.D_mm - bjx, bjy) + robot.alpha_R * math.hypot(wall.D_mm - bjx, bjy) ** 2

    # Back-project cable lengths to wall positions via ideal inverse kinematics
    # (This is what the robot actually draws: the pen lands where the cable lengths say)
    def lr_to_xy(L: float, R: float, D: float) -> Tuple[float, float]:
        x = (L * L - R * R + D * D) / (2.0 * D)
        y2 = max(0.0, L * L - x * x)
        return x, math.sqrt(y2)

    ax_actual, ay_actual = lr_to_xy(aL, aR, wall.D_mm)
    bx_actual, by_actual = lr_to_xy(bL, bR, wall.D_mm)
    return math.hypot(bx_actual - ax_actual, by_actual - ay_actual)


def _residuals(
    params: List[float],
    measurements: List[Tuple[str, str, float]],
    centres: Dict[str, Tuple[float, float]],
    fit_mode: str,  # "robot" or "wall"
    frozen_robot: Optional[RobotProfile],
    frozen_wall_D: float,
) -> List[float]:
    """
    Residual vector for scipy.optimize.least_squares.

    fit_mode="robot": params = [h_pen, k_L, k_R, D, dx, dy]
    fit_mode="wall":  params = [D, dx, dy]   (robot params from frozen_robot)
    """
    if fit_mode == "robot":
        h_pen, k_L, k_R, D, dx, dy = params
        robot = RobotProfile(h_pen_mm=h_pen, k_L=k_L, k_R=k_R)
        wall = WallProfile(D_mm=D, dx_offset_mm=dx, dy_offset_mm=dy)
    else:
        D, dx, dy = params
        assert frozen_robot is not None
        robot = frozen_robot
        wall = WallProfile(D_mm=D, dx_offset_mm=dx, dy_offset_mm=dy)

    res = []
    for label_a, label_b, measured_mm in measurements:
        predicted = _predict_distance(label_a, label_b, centres, robot, wall)
        res.append(predicted - measured_mm)
    return res


# ---------------------------------------------------------------------------
# Interactive measurement collection
# ---------------------------------------------------------------------------

_PRIORITY_MEASUREMENTS = [
    # (label_a, label_b, description)
    ("r0c0", "r0c4", "outer width — top-left to top-right"),
    ("r4c0", "r4c4", "outer width — bottom-left to bottom-right"),
    ("r0c0", "r4c0", "outer height — top-left to bottom-left"),
    ("r0c4", "r4c4", "outer height — top-right to bottom-right"),
    ("r0c0", "r4c4", "diagonal — top-left to bottom-right"),
    ("r0c4", "r4c0", "diagonal — top-right to bottom-left"),
    ("r0c0", "r0c2", "top row — col 0 to col 2 (half-width)"),
    ("r0c2", "r0c4", "top row — col 2 to col 4 (half-width)"),
    ("r2c0", "r2c4", "middle row — full width"),
    ("r4c0", "r4c2", "bottom row — col 0 to col 2"),
    ("r0c0", "r2c0", "left column — top to middle"),
    ("r2c0", "r4c0", "left column — middle to bottom"),
    ("r0c2", "r4c2", "centre column — full height"),
]

_EXTRA_MEASUREMENTS = [
    ("r1c1", "r1c3", "row 1 inner width"),
    ("r3c1", "r3c3", "row 3 inner width"),
    ("r1c1", "r3c1", "col 1 inner height"),
    ("r1c3", "r3c3", "col 3 inner height"),
]


def _ask_measurement(
    label_a: str,
    label_b: str,
    description: str,
    centres: Dict[str, Tuple[float, float]],
    existing: Dict[Tuple[str, str], float],
) -> Optional[float]:
    """Prompt user for one distance measurement. Return None to skip."""
    xa, ya = centres[label_a]
    xb, yb = centres[label_b]
    intended = math.hypot(xb - xa, yb - ya)
    key = (label_a, label_b)
    if key in existing:
        print(f"  (already have {label_a}→{label_b}: {existing[key]:.0f} mm, skipping)")
        return existing[key]

    print(f"\n  Measure: {description}")
    print(f"    {label_a} ({xa:.0f}, {ya:.0f}) → {label_b} ({xb:.0f}, {yb:.0f})")
    print(f"    Intended distance: {intended:.1f} mm")
    while True:
        raw = input("    Measured (mm) [Enter to skip]: ").strip()
        if raw == "":
            return None
        try:
            val = float(raw)
            if val <= 0:
                print("    Must be > 0. Try again.")
                continue
            return val
        except ValueError:
            print("    Not a number. Try again.")


def _collect_measurements(
    centres: Dict[str, Tuple[float, float]],
    n_target: int,
    extra_ok: bool,
) -> List[Tuple[str, str, float]]:
    """Interactively collect distance measurements from the user."""
    results: Dict[Tuple[str, str], float] = {}

    print()
    print("=" * 60)
    print("MEASUREMENT COLLECTION")
    print("=" * 60)
    print(f"Collect at least {n_target} measurements (more is better).")
    print("Use a tape measure. Measure centre-to-centre of each cross.")
    print("Press Enter without a value to skip a measurement.")
    print()

    all_pairs = _PRIORITY_MEASUREMENTS + (_EXTRA_MEASUREMENTS if extra_ok else [])

    for label_a, label_b, desc in all_pairs:
        val = _ask_measurement(label_a, label_b, desc, centres, results)
        if val is not None:
            results[(label_a, label_b)] = val
        if len(results) >= n_target + 4:
            more = input(f"\n  Have {len(results)} measurements. Continue? [y/N]: ").strip().lower()
            if more != "y":
                break

    if len(results) < n_target:
        print(f"\nWARNING: only {len(results)} measurements collected; {n_target} recommended.")
        if len(results) < 3:
            print("Too few measurements to fit. Aborting.")
            sys.exit(1)

    return [(a, b, v) for (a, b), v in results.items()]


# ---------------------------------------------------------------------------
# Fitting
# ---------------------------------------------------------------------------

def _run_fit(
    measurements: List[Tuple[str, str, float]],
    centres: Dict[str, Tuple[float, float]],
    fit_mode: str,
    frozen_robot: Optional[RobotProfile],
    D_initial: float,
) -> Tuple[RobotProfile, WallProfile, float]:
    """Run scipy least_squares and return fitted profiles + RMS residual."""
    try:
        from scipy.optimize import least_squares
    except ImportError:
        print("ERROR: scipy is required for fitting. Install with: pip install scipy", file=sys.stderr)
        sys.exit(1)

    if fit_mode == "robot":
        x0 = [50.0, 1.0, 1.0, D_initial, 0.0, 0.0]
        bounds = (
            [0.0,  0.8, 0.8, D_initial - 50, -100, -100],
            [200.0, 1.2, 1.2, D_initial + 50,  100,  100],
        )
        def fun(p): return _residuals(p, measurements, centres, "robot", None, D_initial)
    else:
        x0 = [D_initial, 0.0, 0.0]
        bounds = (
            [D_initial - 50, -100, -100],
            [D_initial + 50,  100,  100],
        )
        def fun(p): return _residuals(p, measurements, centres, "wall", frozen_robot, D_initial)

    result = least_squares(fun, x0, bounds=bounds, method="trf", ftol=1e-9, xtol=1e-9)

    residuals = result.fun
    rms = math.sqrt(sum(r * r for r in residuals) / len(residuals))

    if fit_mode == "robot":
        h_pen, k_L, k_R, D, dx, dy = result.x
        robot = RobotProfile(
            h_pen_mm=h_pen, k_L=k_L, k_R=k_R,
            fit_rms_mm=rms, n_measurements=len(measurements),
            fitted_at=str(date.today()),
        )
        wall = WallProfile(
            D_mm=D, dx_offset_mm=dx, dy_offset_mm=dy,
            fit_rms_mm=rms, n_measurements=len(measurements),
            fitted_at=str(date.today()),
        )
    else:
        D, dx, dy = result.x
        assert frozen_robot is not None
        robot = frozen_robot
        wall = WallProfile(
            D_mm=D, dx_offset_mm=dx, dy_offset_mm=dy,
            fit_rms_mm=rms, n_measurements=len(measurements),
            fitted_at=str(date.today()),
        )

    return robot, wall, rms


# ---------------------------------------------------------------------------
# Sub-command implementations
# ---------------------------------------------------------------------------

def cmd_generate_pattern(args: argparse.Namespace) -> None:
    out_gcode = Path(args.out)
    out_json = Path(args.out).with_suffix(".json")
    if args.json_out:
        out_json = Path(args.json_out)

    print(f"Generating 5×5 calibration grid for D_mm={args.D_mm:.1f} mm ...")
    write_pattern_files(
        out_gcode=out_gcode,
        out_json=out_json,
        D_mm=args.D_mm,
        pen=args.pen,
        f_travel=args.f_travel,
        f_draw=args.f_draw,
        f_z=args.f_z,
        gcode_comments=args.gcode_comments,
    )

    centres = grid_cross_centres(args.D_mm)
    w = max(centres["r0c4"][0] - centres["r0c0"][0], 0)
    h = max(centres["r4c0"][1] - centres["r0c0"][1], 0)

    print(f"Wrote G-code: {out_gcode}")
    print(f"Wrote intent: {out_json}")
    print(f"Grid spans {w:.0f} mm wide × {h:.0f} mm tall on the wall.")
    print()
    print("Next steps:")
    print("  1. Send this G-code file to your Scribit robot and let it draw the grid.")
    print("     (Use 'sbcmd draw' or your preferred method to send the file.)")
    print("  2. Once drawn, run one of:")
    print(f"       sbcal fit-robot --intent {out_json}   (first time / new robot)")
    print(f"       sbcal fit-wall  --intent {out_json} --robot robot.json  (new wall)")


def _guided_fit(
    args: argparse.Namespace,
    fit_mode: str,  # "robot" or "wall"
) -> None:
    intent_path = Path(args.intent)
    if not intent_path.exists():
        print(f"ERROR: intent file not found: {intent_path}", file=sys.stderr)
        sys.exit(1)

    intent = json.loads(intent_path.read_text(encoding="utf-8"))
    D_nominal = float(intent.get("D_mm", D_MM_DEFAULT))
    centres: Dict[str, Tuple[float, float]] = {
        label: (float(v["x_mm"]), float(v["y_mm"]))
        for label, v in intent["crosses"].items()
    }

    frozen_robot: Optional[RobotProfile] = None
    if fit_mode == "wall":
        robot_path = Path(args.robot)
        if not robot_path.exists():
            print(f"ERROR: robot profile not found: {robot_path}", file=sys.stderr)
            sys.exit(1)
        frozen_robot = load_robot_profile(robot_path)
        print(f"Loaded robot profile: {robot_path}")
        print(f"  h_pen={frozen_robot.h_pen_mm:.1f} mm  k_L={frozen_robot.k_L:.4f}  k_R={frozen_robot.k_R:.4f}")

    # --- intro banner ---
    print()
    print("=" * 60)
    if fit_mode == "robot":
        print("FULL CALIBRATION  (robot-intrinsic + wall-extrinsic)")
        print("Fits: h_pen, k_L, k_R, D, dx_offset, dy_offset")
        n_target = 12
    else:
        print("WALL CALIBRATION  (wall-extrinsic only)")
        print("Fits: D, dx_offset, dy_offset  (robot params frozen)")
        n_target = 6
    print("=" * 60)
    print()
    print("STEP 1 — Print the calibration grid")
    print()
    print(f"  The intent file describes a 5×5 grid of + crosses drawn on the wall.")
    print(f"  Make sure you have already sent the matching .gcode file to the robot")
    print(f"  and it has finished drawing.")
    print()
    input("  Press Enter once the robot has finished drawing the grid ... ")

    # --- collect measurements ---
    measurements = _collect_measurements(centres, n_target, extra_ok=(fit_mode == "robot"))

    print()
    print(f"Collected {len(measurements)} measurements. Running fit ...")

    robot_out, wall_out, rms = _run_fit(
        measurements, centres, fit_mode, frozen_robot, D_nominal
    )

    print()
    print("=" * 60)
    print("FIT RESULTS")
    print("=" * 60)
    print(f"  RMS residual: {rms:.2f} mm")
    if fit_mode == "robot":
        print(f"  h_pen_mm   = {robot_out.h_pen_mm:.2f}")
        print(f"  k_L        = {robot_out.k_L:.5f}")
        print(f"  k_R        = {robot_out.k_R:.5f}")
    print(f"  D_mm       = {wall_out.D_mm:.2f}")
    print(f"  dx_offset  = {wall_out.dx_offset_mm:.2f} mm")
    print(f"  dy_offset  = {wall_out.dy_offset_mm:.2f} mm")

    if rms > 5.0:
        print()
        print("WARNING: RMS > 5 mm — check that your measurements are centre-to-centre")
        print("         of the + crosses, and that the intent file matches the gcode.")

    # --- save outputs ---
    robot_out_path = Path(getattr(args, "robot_out", None) or "robot.json")
    wall_out_path = Path(args.wall_out)

    if fit_mode == "robot":
        save_robot_profile(robot_out, robot_out_path)
        print(f"\nWrote robot profile: {robot_out_path}")

    save_wall_profile(wall_out, wall_out_path)
    print(f"Wrote wall profile:  {wall_out_path}")

    print()
    print("Next steps:")
    if fit_mode == "robot":
        print(f"  sbplot <svg> --robot-cal {robot_out_path} --wall-cal {wall_out_path}")
    else:
        print(f"  sbplot <svg> --robot-cal {args.robot} --wall-cal {wall_out_path}")
    print()
    print("Tip: re-draw the grid and re-measure 5 distances to validate.")


def cmd_fit_robot(args: argparse.Namespace) -> None:
    _guided_fit(args, fit_mode="robot")


def cmd_fit_wall(args: argparse.Namespace) -> None:
    _guided_fit(args, fit_mode="wall")


def cmd_show(args: argparse.Namespace) -> None:
    if args.robot:
        p = Path(args.robot)
        if p.exists():
            r = load_robot_profile(p)
            print(f"Robot profile: {p}")
            print(f"  robot_id  = {r.robot_id!r}")
            print(f"  h_pen_mm  = {r.h_pen_mm:.2f}")
            print(f"  k_L       = {r.k_L:.5f}")
            print(f"  k_R       = {r.k_R:.5f}")
            print(f"  alpha_L   = {r.alpha_L:.2e}")
            print(f"  alpha_R   = {r.alpha_R:.2e}")
            print(f"  fit_rms   = {r.fit_rms_mm:.2f} mm  (n={r.n_measurements})")
            print(f"  fitted_at = {r.fitted_at}")
        else:
            print(f"Robot profile not found: {p}")

    if args.wall:
        p = Path(args.wall)
        if p.exists():
            w = load_wall_profile(p)
            print(f"Wall profile: {p}")
            print(f"  robot_id    = {w.robot_id!r}")
            print(f"  wall_id     = {w.wall_id}")
            print(f"  D_mm        = {w.D_mm:.2f}")
            print(f"  dx_offset   = {w.dx_offset_mm:.2f} mm")
            print(f"  dy_offset   = {w.dy_offset_mm:.2f} mm")
            print(f"  fit_rms     = {w.fit_rms_mm:.2f} mm  (n={w.n_measurements})")
            print(f"  fitted_at   = {w.fitted_at}")
        else:
            print(f"Wall profile not found: {p}")


# ---------------------------------------------------------------------------
# Argument parser
# ---------------------------------------------------------------------------

def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="sbcal",
        description="Scribit manual calibration — improve drawing dimensional accuracy.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = p.add_subparsers(dest="command", required=True)

    # --- generate-pattern ---
    gp = sub.add_parser(
        "generate-pattern",
        help="Generate grid5x5.gcode and grid5x5.json calibration files.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    gp.add_argument("--D_mm", type=float, default=D_MM_DEFAULT,
                    help="Nominal nail separation (mm)")
    gp.add_argument("--out", default="grid5x5.gcode",
                    help="Output G-code filename")
    gp.add_argument("--json-out", default=None,
                    help="Output JSON filename (default: same as --out with .json)")
    gp.add_argument("--pen", type=int, default=1,
                    help="Pen slot (1-4) to use for the pattern")
    gp.add_argument("--f_travel", type=int, default=600, help="Travel feed rate")
    gp.add_argument("--f_draw", type=int, default=300, help="Draw feed rate")
    gp.add_argument("--f_z", type=int, default=600, help="Carousel feed rate")
    gp.add_argument("--gcode-comments", action="store_true",
                    help="Include comments in G-code output")
    gp.set_defaults(func=cmd_generate_pattern)

    # --- fit-robot ---
    fr = sub.add_parser(
        "fit-robot",
        help="Full calibration fit: robot-intrinsic + wall-extrinsic parameters.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    fr.add_argument("--intent", required=True,
                    help="Path to grid5x5.json (intended cross positions)")
    fr.add_argument("--robot-out", default="robot.json",
                    help="Output path for robot calibration profile")
    fr.add_argument("--wall-out", default="wall.json",
                    help="Output path for wall calibration profile")
    fr.set_defaults(func=cmd_fit_robot)

    # --- fit-wall ---
    fw = sub.add_parser(
        "fit-wall",
        help="Fast wall-only fit: D, dx_offset, dy_offset (robot params frozen).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    fw.add_argument("--intent", required=True,
                    help="Path to grid5x5.json (intended cross positions)")
    fw.add_argument("--robot", required=True,
                    help="Path to existing robot.json profile (frozen during fit)")
    fw.add_argument("--wall-out", default="wall.json",
                    help="Output path for wall calibration profile")
    fw.set_defaults(func=cmd_fit_wall)

    # --- show ---
    sh = sub.add_parser("show", help="Display contents of calibration profile files.")
    sh.add_argument("--robot", default=None, help="Path to robot.json")
    sh.add_argument("--wall", default=None, help="Path to wall.json")
    sh.set_defaults(func=cmd_show)

    return p


def main() -> None:
    ap = build_argparser()
    args = ap.parse_args()
    args.func(args)
