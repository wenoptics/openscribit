"""Tests for tool-path optimization (subpath ordering and reversal)."""
from __future__ import annotations

import pytest

from scribit_plot.path_optimizer import (
    Stroke,
    optimize_strokes,
    order_strokes_nearest_neighbor,
    total_travel,
)


def _stroke(pen, start, end, svg_id="x"):
    """Helper: build a simple two-point stroke."""
    return Stroke(pen=pen, svg_id=svg_id, poly=[start, end])


# ----------------------------
# Stroke
# ----------------------------

class TestStroke:
    def test_start_end_properties(self):
        s = Stroke(pen=1, svg_id="a", poly=[(0, 0), (1, 2), (3, 4)])
        assert s.start == (0, 0)
        assert s.end == (3, 4)

    def test_reversed_copy_flips_polyline(self):
        s = Stroke(pen=1, svg_id="a", poly=[(0, 0), (1, 2), (3, 4)])
        r = s.reversed_copy()
        assert r.poly == [(3, 4), (1, 2), (0, 0)]
        assert r.pen == 1
        assert r.svg_id == "a"

    def test_reversed_copy_does_not_mutate_original(self):
        s = Stroke(pen=1, svg_id="a", poly=[(0, 0), (1, 2)])
        _ = s.reversed_copy()
        assert s.poly == [(0, 0), (1, 2)]


# ----------------------------
# total_travel
# ----------------------------

class TestTotalTravel:
    def test_empty_is_zero(self):
        assert total_travel([], (0, 0)) == 0.0

    def test_single_stroke_counts_only_approach(self):
        # Travel = start → stroke.start = 5. The walk along the stroke itself
        # is pen-down and not counted.
        s = _stroke(1, (3, 4), (10, 10))
        assert total_travel([s], (0, 0)) == pytest.approx(5.0)

    def test_two_strokes_sums_between_end_and_next_start(self):
        s1 = _stroke(1, (0, 0), (10, 0))    # approach 0, ends at (10, 0)
        s2 = _stroke(1, (10, 10), (0, 10))  # approach (10,0)→(10,10) = 10
        assert total_travel([s1, s2], (0, 0)) == pytest.approx(10.0)


# ----------------------------
# order_strokes_nearest_neighbor
# ----------------------------

class TestNearestNeighborOrder:
    def test_empty_returns_empty(self):
        assert order_strokes_nearest_neighbor([], (0, 0)) == []

    def test_single_stroke_unchanged(self):
        s = _stroke(1, (5, 5), (6, 6))
        out = order_strokes_nearest_neighbor([s], (0, 0))
        assert len(out) == 1
        assert out[0].poly == [(5, 5), (6, 6)]

    def test_reorders_to_pick_closest(self):
        far = _stroke(1, (100, 100), (101, 101), svg_id="far")
        near = _stroke(1, (1, 1), (2, 2), svg_id="near")
        out = order_strokes_nearest_neighbor([far, near], (0, 0))
        assert [s.svg_id for s in out] == ["near", "far"]

    def test_reverses_stroke_when_end_is_closer(self):
        # start=(100,0), end=(1,0). End is closer to origin → reverse.
        s = _stroke(1, (100, 0), (1, 0), svg_id="rev")
        out = order_strokes_nearest_neighbor([s], (0, 0))
        assert out[0].poly[0] == (1, 0)
        assert out[0].poly[-1] == (100, 0)

    def test_does_not_reverse_when_start_is_closer(self):
        s = _stroke(1, (1, 0), (100, 0), svg_id="fwd")
        out = order_strokes_nearest_neighbor([s], (0, 0))
        assert out[0].poly == [(1, 0), (100, 0)]

    def test_chooses_reversed_remote_stroke_over_unreversed_closer_one(self):
        # Both endpoints of `rev` are closer to origin than either endpoint of `fwd`.
        rev = _stroke(1, (10, 0), (2, 0), svg_id="rev")  # end at (2,0) → d=2
        fwd = _stroke(1, (5, 0), (50, 0), svg_id="fwd")  # start at (5,0) → d=5
        out = order_strokes_nearest_neighbor([fwd, rev], (0, 0))
        assert [s.svg_id for s in out] == ["rev", "fwd"]
        # rev was reversed so we begin at its closer endpoint (2,0).
        assert out[0].poly[0] == (2, 0)

    def test_three_strokes_in_a_row_picked_in_left_to_right_order(self):
        a = _stroke(1, (0, 0), (1, 0), svg_id="a")
        b = _stroke(1, (10, 0), (11, 0), svg_id="b")
        c = _stroke(1, (5, 0), (6, 0), svg_id="c")
        out = order_strokes_nearest_neighbor([a, b, c], (0, 0))
        assert [s.svg_id for s in out] == ["a", "c", "b"]

    def test_actually_reduces_travel(self):
        # Original order zigzags; NN should produce a strictly shorter path.
        a = _stroke(1, (0, 0), (1, 0), svg_id="a")
        b = _stroke(1, (10, 0), (11, 0), svg_id="b")
        c = _stroke(1, (5, 0), (6, 0), svg_id="c")
        original = total_travel([a, b, c], (0, 0))
        out = order_strokes_nearest_neighbor([a, b, c], (0, 0))
        assert total_travel(out, (0, 0)) < original

    def test_preserves_all_strokes(self):
        strokes = [
            _stroke(1, (i, 0), (i + 1, 0), svg_id=f"s{i}")
            for i in (0, 100, 50, 25, 75, 10, 90)
        ]
        out = order_strokes_nearest_neighbor(strokes, (0, 0))
        assert len(out) == len(strokes)
        assert {s.svg_id for s in out} == {s.svg_id for s in strokes}


# ----------------------------
# optimize_strokes
# ----------------------------

class TestOptimizeStrokes:
    def test_empty_input(self):
        assert optimize_strokes([], (0, 0)) == []

    def test_groups_by_pen(self):
        # Mixed pens in document order: 1, 2, 1. All pen 1 should come first.
        s1 = _stroke(1, (0, 0), (1, 0), svg_id="p1a")
        s2 = _stroke(2, (10, 0), (11, 0), svg_id="p2a")
        s3 = _stroke(1, (2, 0), (3, 0), svg_id="p1b")
        out = optimize_strokes([s1, s2, s3], (0, 0))
        assert [s.pen for s in out] == [1, 1, 2]

    def test_preserves_first_encounter_pen_order(self):
        # First pen seen is 2, then 1.
        s2 = _stroke(2, (0, 0), (1, 0))
        s1 = _stroke(1, (5, 0), (6, 0))
        out = optimize_strokes([s2, s1], (0, 0))
        assert [s.pen for s in out] == [2, 1]

    def test_does_not_interleave_pens_even_when_closer(self):
        # A pen-2 stroke sits right next to pen-1's end, but optimizer must
        # not split the pen-1 group to visit it early.
        s1a = _stroke(1, (0, 0), (1, 0), svg_id="p1a")
        s2 = _stroke(2, (2, 0), (3, 0), svg_id="p2")
        s1b = _stroke(1, (100, 0), (101, 0), svg_id="p1b")
        out = optimize_strokes([s1a, s2, s1b], (0, 0))
        pens = [s.pen for s in out]
        # All pen 1 strokes must come before any pen 2 stroke.
        assert pens == [1, 1, 2]

    def test_optimizes_within_pen_group(self):
        far = _stroke(1, (100, 0), (101, 0), svg_id="far")
        near = _stroke(1, (1, 0), (2, 0), svg_id="near")
        out = optimize_strokes([far, near], (0, 0))
        assert [s.svg_id for s in out] == ["near", "far"]

    def test_reverses_within_pen_group(self):
        # End of stroke is closer to origin than start → reverse.
        s = _stroke(1, (100, 0), (1, 0), svg_id="rev")
        out = optimize_strokes([s], (0, 0))
        assert out[0].poly[0] == (1, 0)

    def test_reduces_total_travel(self):
        strokes = [
            _stroke(1, (0, 0), (1, 0)),
            _stroke(1, (20, 0), (21, 0)),
            _stroke(1, (5, 0), (6, 0)),
            _stroke(1, (15, 0), (16, 0)),
            _stroke(1, (10, 0), (11, 0)),
        ]
        before = total_travel(strokes, (0, 0))
        out = optimize_strokes(strokes, (0, 0))
        assert total_travel(out, (0, 0)) < before

    def test_preserves_all_strokes_across_pens(self):
        strokes = [
            _stroke(1, (0, 0), (1, 0), svg_id="a"),
            _stroke(2, (10, 0), (11, 0), svg_id="b"),
            _stroke(1, (5, 0), (6, 0), svg_id="c"),
            _stroke(2, (15, 0), (16, 0), svg_id="d"),
        ]
        out = optimize_strokes(strokes, (0, 0))
        assert len(out) == 4
        assert {s.svg_id for s in out} == {"a", "b", "c", "d"}
