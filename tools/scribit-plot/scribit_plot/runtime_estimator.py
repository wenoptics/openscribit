"""Estimate total G-code execution time from a list of G-code lines.

Uses a simple command-profile approach: each G-command has a fixed overhead plus
distance-over-feedrate time for motion commands.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class GCodeProfile:
    """Naive timing constants for each Scribit G-command type."""
    # Fixed-cost commands (seconds each)
    overhead_G21: float = 0.02   # set mm units
    overhead_G90: float = 0.02   # absolute mode
    overhead_G91: float = 0.02   # incremental mode
    overhead_M17: float = 0.10   # motors on
    overhead_G92: float = 0.05   # set position reference
    overhead_G77: float = 8.0    # carousel home routine (mechanical)
    overhead_G101: float = 0.6   # pen latch (mechanical ~30 deg swing)

    # G1 XY move: feedrate F is mm/min for cord-length vector
    # time = hypot(dX, dY) / (F / 60)  [seconds]
    # (no override needed — F is read from the command)

    # G1 Z move: feedrate F is deg/min for carousel rotation
    # time = |dZ| / (F / 60)  [seconds]
    # (no override needed — F and delta are read from the command)

    # G4 dwell: time = S parameter (seconds, exact)


@dataclass
class EstimationResult:
    total_seconds: float
    draw_move_seconds: float
    travel_move_seconds: float
    z_move_seconds: float
    fixed_overhead_seconds: float
    dwell_seconds: float
    n_draw_moves: int
    n_travel_moves: int
    n_z_moves: int
    n_pen_latches: int
    n_dwells: int

    def summary(self) -> str:
        t = self.total_seconds
        h = int(t // 3600)
        m = int((t % 3600) // 60)
        s = t % 60
        time_str = f"{h}h {m:02d}m {s:05.2f}s" if h else f"{m}m {s:05.2f}s"
        latch_s = self.n_pen_latches * 0.6  # matches GCodeProfile.overhead_G101
        other_s = self.fixed_overhead_seconds - self.dwell_seconds - latch_s
        return (
            f"Estimated runtime: {time_str} ({t:.1f}s total)\n"
            f"  draw moves:    {self.draw_move_seconds:7.1f}s  ({self.n_draw_moves} moves)\n"
            f"  travel moves:  {self.travel_move_seconds:7.1f}s  ({self.n_travel_moves} moves)\n"
            f"  Z (carousel):  {self.z_move_seconds:7.1f}s  ({self.n_z_moves} moves)\n"
            f"  pen latches:   {latch_s:7.1f}s  ({self.n_pen_latches} G101 commands)\n"
            f"  dwells:        {self.dwell_seconds:7.1f}s  ({self.n_dwells} dwells)\n"
            f"  setup/other:   {other_s:7.1f}s"
        )


_RE_G1_XY = re.compile(
    r"^G1\s+X([+-]?\d*\.?\d+)\s+Y([+-]?\d*\.?\d+)\s+F(\d+)", re.IGNORECASE
)
_RE_G1_Z = re.compile(r"^G1\s+Z([+-]?\d*\.?\d+)\s+F(\d+)", re.IGNORECASE)
_RE_G4 = re.compile(r"^G4\s+S([+-]?\d*\.?\d+)", re.IGNORECASE)

# Tags injected as comments to distinguish draw vs travel moves
_DRAW_COMMENT_HINTS = {"draw stroke", "pen down"}
_TRAVEL_COMMENT_HINTS = {"travel", "return to start", "bbox corner", "move to"}


def estimate_runtime(
    lines: List[str],
    profile: Optional[GCodeProfile] = None,
) -> EstimationResult:
    """Parse G-code lines and return a timing estimate.

    Feed-rate context is tracked across lines: once F is seen on a G1 it persists
    until overridden (standard G-code modal behaviour).
    """
    if profile is None:
        profile = GCodeProfile()

    draw_s = 0.0
    travel_s = 0.0
    z_s = 0.0
    fixed_s = 0.0
    dwell_s = 0.0

    n_draw = 0
    n_travel = 0
    n_z = 0
    n_latch = 0
    n_dwell = 0

    current_feed_mm_per_min: float = 300.0  # default draw feed
    current_z: Optional[float] = None       # track absolute Z for delta computation
    in_draw_context = False
    last_comment = ""

    for raw in lines:
        line = raw.strip()
        if not line:
            continue

        # Track comment context to classify subsequent G1 XY moves
        if line.startswith(";"):
            last_comment = line
            if any(h in line.lower() for h in _DRAW_COMMENT_HINTS):
                in_draw_context = True
            elif any(h in line.lower() for h in _TRAVEL_COMMENT_HINTS):
                in_draw_context = False
            continue

        upper = line.upper()

        # --- G1 XY (cord-length) move ---
        m = _RE_G1_XY.match(line)
        if m:
            dx = float(m.group(1))
            dy = float(m.group(2))
            feed = float(m.group(3))
            current_feed_mm_per_min = feed
            dist = math.hypot(dx, dy)
            dt = dist / (feed / 60.0) if feed > 0 and dist > 0 else 0.0
            if in_draw_context:
                draw_s += dt
                n_draw += 1
            else:
                travel_s += dt
                n_travel += 1
            continue

        # --- G1 Z (carousel rotation) move ---
        m = _RE_G1_Z.match(line)
        if m:
            z_target = float(m.group(1))
            feed = float(m.group(2))
            if current_z is not None:
                delta_z = abs(z_target - current_z)
            else:
                # No prior Z known; assume a typical pen-select swing (~90 deg)
                delta_z = 90.0
            current_z = z_target
            dt = delta_z / (feed / 60.0) if feed > 0 and delta_z > 0 else 0.0
            z_s += dt
            n_z += 1
            continue

        # G92 Z sets our Z reference without motion
        if upper.startswith("G92") and "Z" in upper:
            m_z = re.search(r"Z([+-]?\d*\.?\d+)", line, re.IGNORECASE)
            if m_z:
                current_z = float(m_z.group(1))

        # --- G4 dwell ---
        m = _RE_G4.match(line)
        if m:
            dwell_s += float(m.group(1))
            fixed_s += float(m.group(1))
            n_dwell += 1
            continue

        # --- Fixed-cost commands ---
        if upper.startswith("G77"):
            fixed_s += profile.overhead_G77
        elif upper.startswith("G101"):
            fixed_s += profile.overhead_G101
            n_latch += 1
        elif upper.startswith("G21"):
            fixed_s += profile.overhead_G21
        elif upper.startswith("G90"):
            fixed_s += profile.overhead_G90
        elif upper.startswith("G91"):
            fixed_s += profile.overhead_G91
        elif upper.startswith("M17"):
            fixed_s += profile.overhead_M17
        elif upper.startswith("G92"):
            fixed_s += profile.overhead_G92

    total = draw_s + travel_s + z_s + fixed_s
    return EstimationResult(
        total_seconds=total,
        draw_move_seconds=draw_s,
        travel_move_seconds=travel_s,
        z_move_seconds=z_s,
        fixed_overhead_seconds=fixed_s,
        dwell_seconds=dwell_s,
        n_draw_moves=n_draw,
        n_travel_moves=n_travel,
        n_z_moves=n_z,
        n_pen_latches=n_latch,
        n_dwells=n_dwell,
    )
