"""Load and save robot/wall calibration profiles (JSON)."""
from __future__ import annotations

import json
from datetime import date
from pathlib import Path
from typing import Optional

from .geometry import RobotProfile, WallProfile

_VERSION = 1


def load_robot_profile(path: str | Path) -> RobotProfile:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return RobotProfile(
        robot_id=data.get("robot_id", ""),
        h_pen_mm=float(data.get("h_pen_mm", 0.0)),
        k_L=float(data.get("k_L", 1.0)),
        k_R=float(data.get("k_R", 1.0)),
        alpha_L=float(data.get("alpha_L", 0.0)),
        alpha_R=float(data.get("alpha_R", 0.0)),
        fit_rms_mm=float(data.get("fit_rms_mm", 0.0)),
        n_measurements=int(data.get("n_measurements", 0)),
        fitted_at=data.get("fitted_at", ""),
    )


def save_robot_profile(profile: RobotProfile, path: str | Path) -> None:
    Path(path).write_text(
        json.dumps(
            {
                "version": _VERSION,
                "robot_id": profile.robot_id,
                "h_pen_mm": round(profile.h_pen_mm, 3),
                "k_L": round(profile.k_L, 6),
                "k_R": round(profile.k_R, 6),
                "alpha_L": round(profile.alpha_L, 9),
                "alpha_R": round(profile.alpha_R, 9),
                "fit_rms_mm": round(profile.fit_rms_mm, 3),
                "n_measurements": profile.n_measurements,
                "fitted_at": profile.fitted_at or str(date.today()),
            },
            indent=2,
        ),
        encoding="utf-8",
    )


def load_wall_profile(path: str | Path) -> WallProfile:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return WallProfile(
        robot_id=data.get("robot_id", ""),
        wall_id=int(data.get("wall_id", 0)),
        D_mm=float(data.get("D_mm", 1860.0)),
        dx_offset_mm=float(data.get("dx_offset_mm", 0.0)),
        dy_offset_mm=float(data.get("dy_offset_mm", 0.0)),
        fit_rms_mm=float(data.get("fit_rms_mm", 0.0)),
        n_measurements=int(data.get("n_measurements", 0)),
        fitted_at=data.get("fitted_at", ""),
    )


def save_wall_profile(profile: WallProfile, path: str | Path) -> None:
    Path(path).write_text(
        json.dumps(
            {
                "version": _VERSION,
                "robot_id": profile.robot_id,
                "wall_id": profile.wall_id,
                "D_mm": round(profile.D_mm, 3),
                "dx_offset_mm": round(profile.dx_offset_mm, 3),
                "dy_offset_mm": round(profile.dy_offset_mm, 3),
                "fit_rms_mm": round(profile.fit_rms_mm, 3),
                "n_measurements": profile.n_measurements,
                "fitted_at": profile.fitted_at or str(date.today()),
            },
            indent=2,
        ),
        encoding="utf-8",
    )


def check_robot_id_match(
    robot: RobotProfile,
    wall: WallProfile,
    *,
    warn: bool = True,
) -> bool:
    """Return True if robot_ids agree (or one/both are empty). Warn otherwise."""
    if robot.robot_id and wall.robot_id and robot.robot_id != wall.robot_id:
        if warn:
            import sys
            print(
                f"WARNING: robot.json robot_id={robot.robot_id!r} does not match "
                f"wall.json robot_id={wall.robot_id!r} — profiles may be mismatched.",
                file=sys.stderr,
            )
        return False
    return True


def default_robot_profile_path(robot_id: Optional[str] = None) -> Path:
    base = Path.home() / ".scribit"
    if robot_id:
        return base / robot_id / "robot.json"
    return base / "robot.json"


def default_wall_profile_path(robot_id: Optional[str], wall_id: int) -> Path:
    base = Path.home() / ".scribit"
    if robot_id:
        return base / robot_id / "walls" / f"wall_{wall_id}.json"
    return base / "walls" / f"wall_{wall_id}.json"
