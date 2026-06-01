"""Tests for tool-path optimization (subpath ordering and reversal)."""
from __future__ import annotations

import pytest

from scribit_plot.path_optimizer import (
    Stroke,
    count_pen_lifts,
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


# ----------------------------
# count_pen_lifts (stroke chaining)
# ----------------------------

class TestCountPenLifts:
    def test_empty_is_zero(self):
        assert count_pen_lifts([]) == 0

    def test_single_stroke_is_one_lift(self):
        assert count_pen_lifts([_stroke(1, (0, 0), (1, 0))]) == 1

    def test_disjoint_same_pen_strokes_each_lift(self):
        s1 = _stroke(1, (0, 0), (1, 0))
        s2 = _stroke(1, (10, 0), (11, 0))  # gap of 9mm — far above eps
        assert count_pen_lifts([s1, s2]) == 2

    def test_touching_same_pen_strokes_chain(self):
        # s1 ends at (1, 0); s2 starts at (1, 0) → chain into one pen-down.
        s1 = _stroke(1, (0, 0), (1, 0))
        s2 = _stroke(1, (1, 0), (2, 0))
        assert count_pen_lifts([s1, s2]) == 1

    def test_touching_within_eps_chains(self):
        # 0.5e-3mm gap with default eps=1e-3 → still chains.
        s1 = _stroke(1, (0, 0), (1.0, 0))
        s2 = _stroke(1, (1.0005, 0), (2, 0))
        assert count_pen_lifts([s1, s2]) == 1

    def test_outside_eps_does_not_chain(self):
        # 2e-3mm gap with default eps=1e-3 → lift required.
        s1 = _stroke(1, (0, 0), (1.0, 0))
        s2 = _stroke(1, (1.002, 0), (2, 0))
        assert count_pen_lifts([s1, s2]) == 2

    def test_pen_change_always_lifts_even_if_touching(self):
        s1 = _stroke(1, (0, 0), (1, 0))
        s2 = _stroke(2, (1, 0), (2, 0))  # touches, but different pen
        assert count_pen_lifts([s1, s2]) == 2

    def test_pen_change_lifts_even_at_exactly_same_xy(self):
        # Both strokes start/end at exactly the same point but use different pens.
        # The pen switch is non-optional regardless of how close the endpoints are.
        s1 = _stroke(1, (10.0, 5.0), (20.0, 5.0))
        s2 = _stroke(2, (20.0, 5.0), (30.0, 5.0))
        assert count_pen_lifts([s1, s2]) == 2

    def test_pen_change_lifts_with_very_generous_eps(self):
        # Even with a meter of eps, a color change must still force a lift.
        s1 = _stroke(1, (0, 0), (1, 0))
        s2 = _stroke(2, (1, 0), (2, 0))
        assert count_pen_lifts([s1, s2], connect_eps_mm=1000.0) == 2

    def test_alternating_pens_at_shared_point_each_lift(self):
        # Four strokes meeting at the same XY, colors alternate 1,2,1,2.
        # Every transition crosses a color boundary → 4 lifts.
        strokes = [
            _stroke(1, (5, 5), (0, 5)),
            _stroke(2, (0, 5), (5, 0)),
            _stroke(1, (5, 0), (10, 5)),
            _stroke(2, (10, 5), (5, 10)),
        ]
        # Note: with the optimizer NOT applied, color alternates every stroke.
        assert count_pen_lifts(strokes) == 4

    def test_mixed_chain_then_color_change(self):
        # Two pen-1 strokes chain into one pen-down; pen-2 stroke after them
        # touches the chain endpoint but must still trigger a separate pen-down.
        strokes = [
            _stroke(1, (0, 0), (1, 0)),
            _stroke(1, (1, 0), (2, 0)),  # chains with prev (same pen, touching)
            _stroke(2, (2, 0), (3, 0)),  # touches but pen change → new pen-down
        ]
        assert count_pen_lifts(strokes) == 2

    def test_eps_zero_disables_chaining(self):
        # Exact match would still chain with eps=0, but anything else won't.
        s1 = _stroke(1, (0, 0), (1.0, 0))
        s2 = _stroke(1, (1.0000001, 0), (2, 0))  # sub-micron gap
        assert count_pen_lifts([s1, s2], connect_eps_mm=0.0) == 2

    def test_long_chain_collapses_to_one(self):
        # Five strokes daisy-chained head-to-tail → single pen-down.
        strokes = [
            _stroke(1, (0, 0), (1, 0)),
            _stroke(1, (1, 0), (2, 0)),
            _stroke(1, (2, 0), (3, 0)),
            _stroke(1, (3, 0), (4, 0)),
            _stroke(1, (4, 0), (5, 0)),
        ]
        assert count_pen_lifts(strokes) == 1

    def test_chain_broken_by_gap_in_middle(self):
        # a→b touches, b→c gap, c→d touches: 2 chains → 2 lifts.
        strokes = [
            _stroke(1, (0, 0), (1, 0)),
            _stroke(1, (1, 0), (2, 0)),
            _stroke(1, (10, 0), (11, 0)),
            _stroke(1, (11, 0), (12, 0)),
        ]
        assert count_pen_lifts(strokes) == 2

    def test_optimizer_output_can_produce_chains(self):
        # Two strokes that share an endpoint, fed in reverse order.
        # The optimizer should reorder (and possibly reverse) them so they chain.
        s1 = _stroke(1, (5, 0), (10, 0), svg_id="far")
        s2 = _stroke(1, (5, 0), (0, 0), svg_id="near")  # ends at (0,0) → start near origin
        out = optimize_strokes([s1, s2], (0, 0))
        # After optimization both strokes should be drawn back-to-back with no lift.
        assert count_pen_lifts(out) == 1
