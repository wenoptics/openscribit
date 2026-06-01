"""Tool-path optimization: reorder and flip strokes to minimize pen-up travel.

A "stroke" here is a single pen-down polyline in wall XY (mm). Between strokes
the pen is up and the robot travels in straight lines, which is overhead we
want to minimize. Strokes are non-directional (open polylines can be drawn
from either end), so the optimizer is free to reverse them.

Pen changes are also expensive (carousel rotation), so strokes are grouped by
pen before optimization. Pen groups are emitted in first-encounter order, and
nearest-neighbor + reversal runs independently inside each group.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple


XY = Tuple[float, float]


@dataclass
class Stroke:
    """A pen-down polyline in wall XY (mm), ready to be drawn."""
    pen: int
    svg_id: str
    poly: List[XY]

    @property
    def start(self) -> XY:
        return self.poly[0]

    @property
    def end(self) -> XY:
        return self.poly[-1]

    def reversed_copy(self) -> "Stroke":
        return Stroke(pen=self.pen, svg_id=self.svg_id, poly=list(reversed(self.poly)))


def _dist_sq(a: XY, b: XY) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def order_strokes_nearest_neighbor(
    strokes: Sequence[Stroke],
    start_xy: XY,
) -> List[Stroke]:
    """Greedy nearest-neighbor ordering with per-stroke reversal.

    Each iteration: from the current XY, pick the unvisited stroke whose
    nearer endpoint (start or end) is closest, and orient it so we begin at
    that endpoint. O(n²) — fine for typical SVGs.
    """
    remaining = list(strokes)
    ordered: List[Stroke] = []
    cur = start_xy
    while remaining:
        best_idx = 0
        best_d2 = float("inf")
        best_reverse = False
        for i, s in enumerate(remaining):
            d_start = _dist_sq(cur, s.start)
            if d_start < best_d2:
                best_d2 = d_start
                best_idx = i
                best_reverse = False
            d_end = _dist_sq(cur, s.end)
            if d_end < best_d2:
                best_d2 = d_end
                best_idx = i
                best_reverse = True
        chosen = remaining.pop(best_idx)
        if best_reverse:
            chosen = chosen.reversed_copy()
        ordered.append(chosen)
        cur = chosen.end
    return ordered


def optimize_strokes(
    strokes: Sequence[Stroke],
    start_xy: XY,
) -> List[Stroke]:
    """Group strokes by pen, then nearest-neighbor + reversal within each group.

    Pen groups are emitted in first-encounter order — splitting a pen group
    in two would force an extra carousel rotation back to that pen later.
    """
    pen_order: List[int] = []
    by_pen: Dict[int, List[Stroke]] = {}
    for s in strokes:
        if s.pen not in by_pen:
            by_pen[s.pen] = []
            pen_order.append(s.pen)
        by_pen[s.pen].append(s)

    result: List[Stroke] = []
    cur = start_xy
    for pen in pen_order:
        group_ordered = order_strokes_nearest_neighbor(by_pen[pen], cur)
        result.extend(group_ordered)
        if group_ordered:
            cur = group_ordered[-1].end
    return result


def total_travel(strokes: Sequence[Stroke], start_xy: XY) -> float:
    """Sum of pen-up travel: start → first.start, then end_i → start_{i+1}."""
    if not strokes:
        return 0.0
    total = 0.0
    cur = start_xy
    for s in strokes:
        dx = s.start[0] - cur[0]
        dy = s.start[1] - cur[1]
        total += math.hypot(dx, dy)
        cur = s.end
    return total


def count_pen_lifts(
    strokes: Sequence[Stroke],
    connect_eps_mm: float = 1e-3,
) -> int:
    """Number of pen-down events required to draw `strokes` in order.

    Two consecutive strokes are chained (single pen-down) when they share the
    same pen and the previous end meets the next start within `connect_eps_mm`.
    Lower bound = number of distinct pen-runs; equals len(strokes) when nothing
    chains.
    """
    if not strokes:
        return 0
    eps_sq = connect_eps_mm * connect_eps_mm
    lifts = 1
    prev = strokes[0]
    for cur in strokes[1:]:
        if cur.pen != prev.pen:
            lifts += 1
        else:
            dx = cur.start[0] - prev.end[0]
            dy = cur.start[1] - prev.end[1]
            if dx * dx + dy * dy > eps_sq:
                lifts += 1
        prev = cur
    return lifts
