"""CLI entry point: argument parsing and main conversion pipeline."""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import List, Optional

from .calibration_profile import (
    check_robot_id_match,
    load_robot_profile,
    load_wall_profile,
)
from .config import D_MM_DEFAULT, PEN_SLOTS_Z, STARTING_X, STARTING_Y
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
from .geometry import (
    RobotProfile,
    SvgToWallMapper,
    WallProfile,
    move_xy_segmented,
    wall_xy_to_lr_delta_g1,
)
from .path_optimizer import Stroke, optimize_strokes, total_travel
from .runtime_estimator import estimate_runtime
from .svg_loader import (
    compute_svg_bbox,
    load_drawable_paths,
    sample_path_uniform_t,
    split_into_continuous_subpaths,
)


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
    optimize_path: bool
    out_bbox: str
    out_draw: str
    robot_cal: Optional[str]
    wall_cal: Optional[str]


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
                   help="Pen-down step size along curves in wall mm")
    p.add_argument("--travel_step_mm", type=float, default=5.0,
                   help="Pen-up max step size in wall mm when repositioning")

    p.add_argument("--f_travel", type=int, default=600, help="Feed rate for pen-up travel moves")
    p.add_argument("--f_draw", type=int, default=300, help="Feed rate for pen-down drawing moves")
    p.add_argument("--f_z", type=int, default=600, help="Feed rate for Z (carousel) moves")

    p.add_argument("--dot_dwell_s", type=float, default=0.20,
                   help="Dwell time (seconds) for bbox corner dots")
    p.add_argument("--bbox_pen", type=int, default=1,
                   help="Pen slot (1..4) for bbox dots")
    p.add_argument("--default_pen", type=int, default=1,
                   help="Fallback pen slot (1..4) if pen mapping overflows")

    p.add_argument("--no_home_carousel", action="store_true",
                   help="Do NOT emit G77 + G92 Z-56 at file start (not recommended)")

    p.add_argument("--return-after-finish", dest="return_after_finish",
                   action="store_true", default=True,
                   help="Return robot to starting position after finishing")
    p.add_argument("--no-return-after-finish", dest="return_after_finish",
                   action="store_false",
                   help="Do not return to starting position after finishing")

    p.add_argument("--gcode-comments", dest="gcode_comments",
                   action="store_true", default=False,
                   help="Emit '; ---' comment lines in G-code output (off by default)")

    p.add_argument("--optimize-path", dest="optimize_path",
                   action="store_true", default=True,
                   help="Reorder and flip subpaths to minimize pen-up travel (default: on)")
    p.add_argument("--no-optimize-path", dest="optimize_path",
                   action="store_false",
                   help="Disable tool path optimization (preserve document order)")

    p.add_argument("--out_bbox", default="bbox_dots.gcode",
                   help="Output filename for bbox dots G-code")
    p.add_argument("--out_draw", default="drawing.gcode",
                   help="Output filename for drawing G-code")

    p.add_argument("--robot-cal", default=None, metavar="FILE",
                   help="Path to robot.json calibration profile (enables extended kinematics)")
    p.add_argument("--wall-cal", default=None, metavar="FILE",
                   help="Path to wall.json calibration profile (required together with --robot-cal)")

    return p


def _write_lines(path: str, lines: List[str]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main() -> None:
    ap = build_argparser()
    ns = ap.parse_args()
    ns.home_carousel = not ns.no_home_carousel

    d = vars(ns)
    d.pop("no_home_carousel", None)
    # argparse uses hyphens → underscores for dest
    d["robot_cal"] = d.pop("robot_cal", None)
    d["wall_cal"] = d.pop("wall_cal", None)
    args = Args(**d)

    args_D = float(args.D_mm)
    if args_D <= 0:
        raise SystemExit("--D_mm must be > 0")
    if args.bbox_pen not in PEN_SLOTS_Z:
        raise SystemExit("--bbox_pen must be 1..4")
    if args.default_pen not in PEN_SLOTS_Z:
        raise SystemExit("--default_pen must be 1..4")
    if not (0.0 < args.fit_frac <= 1.5):
        raise SystemExit("--fit_frac must be in (0, 1.5]")

    # --- load optional calibration profiles ---
    robot_profile: Optional[RobotProfile] = None
    wall_profile: Optional[WallProfile] = None

    if args.robot_cal or args.wall_cal:
        if not (args.robot_cal and args.wall_cal):
            raise SystemExit("--robot-cal and --wall-cal must be provided together.")
        try:
            robot_profile = load_robot_profile(args.robot_cal)
            wall_profile = load_wall_profile(args.wall_cal)
        except (FileNotFoundError, KeyError, ValueError) as e:
            raise SystemExit(f"Failed to load calibration profile: {e}")
        check_robot_id_match(robot_profile, wall_profile)
        # Use D_mm from the wall profile when calibration is active
        args_D = wall_profile.D_mm
        print(
            f"Calibration active: robot={args.robot_cal} wall={args.wall_cal} "
            f"D={args_D:.1f} h_pen={robot_profile.h_pen_mm:.1f} mm"
        )

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
    wall_bbox_left   = bbox_corners_wall[0][0]
    wall_bbox_top    = bbox_corners_wall[0][1]
    wall_bbox_right  = bbox_corners_wall[1][0]
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
            cur_xy, xy, args_D, args.f_travel, max_step_mm=args.travel_step_mm,
            robot=robot_profile, wall=wall_profile,
        )
        g_bbox += lines
        g_bbox += gcode_pen_down()
        g_bbox += gcode_dwell(args.dot_dwell_s)
        g_bbox += gcode_pen_up(pen, args.f_z, st_bbox)

    if args.return_after_finish:
        g_bbox.append("; --- return to start position after bbox dots ---")
        lines, cur_xy = move_xy_segmented(
            cur_xy, (STARTING_X, STARTING_Y), args_D, args.f_travel,
            max_step_mm=args.travel_step_mm, robot=robot_profile, wall=wall_profile,
        )
        g_bbox += lines

    _write_lines(args.out_bbox, g_bbox if args.gcode_comments else strip_comments(g_bbox))

    # ---------- (2) drawing ----------
    g_draw: List[str] = []
    g_draw += gcode_header()
    st_draw = CarouselState()
    if args.home_carousel:
        g_draw += gcode_home_carousel(st_draw)
        g_draw += gcode_home_carousel(st_draw)
    cur_xy = (STARTING_X, STARTING_Y)

    all_strokes: List[Stroke] = []
    for path, pen, svg_id in drawable:
        subpaths = split_into_continuous_subpaths(path)
        for sp in subpaths:
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
            all_strokes.append(Stroke(pen=pen, svg_id=svg_id, poly=poly_wall))

    travel_before = total_travel(all_strokes, cur_xy)
    if args.optimize_path:
        all_strokes = optimize_strokes(all_strokes, cur_xy)
    travel_after = total_travel(all_strokes, cur_xy)

    prev_pen: Optional[int] = None
    pen_down_block = 0
    for stroke in all_strokes:
        if stroke.pen != prev_pen:
            g_draw += gcode_pen_select_ccw(stroke.pen, args.f_z, st_draw)
            prev_pen = stroke.pen

        poly_wall = stroke.poly
        g_draw.append(
            f"; --- travel (pen-up) to subpath start: "
            f"({poly_wall[0][0]:.2f}, {poly_wall[0][1]:.2f}) mm ---"
        )
        lines, cur_xy = move_xy_segmented(
            cur_xy, poly_wall[0], args_D, args.f_travel,
            max_step_mm=args.travel_step_mm, robot=robot_profile, wall=wall_profile,
        )
        g_draw += lines

        pen_down_block += 1
        g_draw.append(
            f"; --- draw stroke #{pen_down_block:03d}: svg_id={stroke.svg_id} pen={stroke.pen} "
            f"pts={len(poly_wall)} start=({poly_wall[0][0]:.2f},{poly_wall[0][1]:.2f}) "
            f"end=({poly_wall[-1][0]:.2f},{poly_wall[-1][1]:.2f}) ---"
        )

        g_draw += gcode_pen_down()

        for xy in poly_wall[1:]:
            line, cur_xy = wall_xy_to_lr_delta_g1(
                cur_xy, xy, args_D, args.f_draw,
                robot=robot_profile, wall=wall_profile,
            )
            g_draw.append(line)

        g_draw += gcode_pen_up(stroke.pen, args.f_z, st_draw)

    if args.return_after_finish:
        g_draw.append("; --- return to start position after drawing ---")
        lines, cur_xy = move_xy_segmented(
            cur_xy, (STARTING_X, STARTING_Y), args_D, args.f_travel,
            max_step_mm=args.travel_step_mm, robot=robot_profile, wall=wall_profile,
        )
        g_draw += lines

    _write_lines(args.out_draw, g_draw if args.gcode_comments else strip_comments(g_draw))

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
    if args.optimize_path and travel_before > 0:
        pct = (1.0 - travel_after / travel_before) * 100.0
        print(
            f"path optimization: ON  pen-up travel "
            f"{travel_before:.0f}mm → {travel_after:.0f}mm ({pct:.1f}% reduction)"
        )
    else:
        print(f"path optimization: OFF  pen-up travel {travel_after:.0f}mm")

    est = estimate_runtime(g_draw)
    print()
    print(est.summary())
