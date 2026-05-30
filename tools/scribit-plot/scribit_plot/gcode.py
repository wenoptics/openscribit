"""G-code building blocks for Scribit: header, carousel control, pen up/down, dwell."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from .config import PEN_SLOTS_Z, Z_AFTER_G77


@dataclass
class CarouselState:
    """Tracks commanded carousel Z (degrees) to enforce CCW-only pen changes."""
    z: Optional[float] = None


def ccw_only_target(current_z: Optional[float], slot_z: float) -> float:
    """Return an absolute Z target that moves CCW-only by adding +360 as needed."""
    if current_z is None:
        return float(slot_z)
    target = float(slot_z)
    while target < current_z:
        target += 360.0
    return target


def gcode_header() -> List[str]:
    return [
        "; --- header: mm units, incremental mode, motors on ---",
        "G21",
        "G91",
        "M17",
    ]


def gcode_home_carousel(st: CarouselState) -> List[str]:
    """Home carousel (G77) and set a known Z reference (G92 Z-56)."""
    st.z = Z_AFTER_G77
    return [
        "; --- home carousel: find Z home, set reference position ---",
        "G21",
        "G90",
        "M17",
        "G77",
        f"G92 Z{Z_AFTER_G77:g}",
        "G91",
    ]


def gcode_pen_select_ccw(pen: int, f_z: int, st: CarouselState) -> List[str]:
    """Select a pen slot, forcing CCW-only carousel motion via +360 wrap."""
    if pen not in PEN_SLOTS_Z:
        raise ValueError(f"pen must be 1..4, got {pen}")
    slot = float(PEN_SLOTS_Z[pen])
    target = ccw_only_target(st.z, slot)
    st.z = target
    return [
        f"; --- pen select: rotate carousel CCW to slot {pen} (Z={target:.3f} deg) ---",
        "G90",
        f"G1 Z{target:.3f} F{f_z}",
        "G91",
    ]


def gcode_pen_down() -> List[str]:
    # 30 degrees isn't enough; 3x G101 (~90 degrees total) gives a reliable latch.
    return [
        "; --- pen down: engage pen (3x G101 for reliable latch) ---",
        "G101",
        "G101",
        "G101",
    ]


def gcode_pen_up(pen: int, f_z: int, st: CarouselState) -> List[str]:
    """Pen-up: retract by returning carousel to the slot Z position (CCW-only)."""
    lines = gcode_pen_select_ccw(pen, f_z, st)
    lines[0] = f"; --- pen up: retract pen by returning carousel to slot {pen} ---"
    return lines


def gcode_dwell(seconds: float) -> List[str]:
    s = max(0.0, seconds)
    return [
        f"; --- dwell: pause {s:.3f} s (let pen mark the surface) ---",
        f"G4 S{s:.3f}",
    ]


def strip_comments(lines: List[str]) -> List[str]:
    return [l for l in lines if not l.startswith(";")]
