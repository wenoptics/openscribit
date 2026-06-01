"""End-to-end CLI tests verifying stroke chaining respects pen-color boundaries.

These exercise the real `main()` pipeline (parse → optimize → emit G-code) on
synthetic SVGs with carefully chosen geometry. The integration coverage matters
here because the chaining decision lives inside the emit loop in cli.py, which
isn't a pure function — only an end-to-end test confirms the actual gcode that
ships to the robot keeps pen-up/pen-select between different-color strokes.
"""
from __future__ import annotations

import sys
from pathlib import Path
from textwrap import dedent

import pytest


def _run_cli(svg_path: Path, out_dir: Path, *, extra_args=()):
    """Invoke scribit_plot.cli.main() with argv patched; return gcode contents."""
    from scribit_plot.cli import main

    out_draw = out_dir / "draw.gcode"
    argv = [
        "sbplot", str(svg_path),
        "--out_bbox", str(out_dir / "bbox.gcode"),
        "--out_draw", str(out_draw),
        "--gcode-comments",
        *extra_args,
    ]
    old_argv = sys.argv
    try:
        sys.argv = argv
        main()
    finally:
        sys.argv = old_argv
    return out_draw.read_text()


# Two strokes that meet exactly at the midpoint of a viewBox but use different
# stroke colors. A correct emitter must lift the pen and rotate the carousel
# between them — even though the endpoints touch within any reasonable eps.
MULTICOLOR_TOUCHING_SVG = dedent("""\
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
      <line x1="0"  y1="40" x2="50"  y2="50" stroke="#ff0000"/>
      <line x1="50" y1="50" x2="100" y2="60" stroke="#0000ff"/>
    </svg>
""")


SAMECOLOR_TOUCHING_SVG = dedent("""\
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
      <line x1="0"  y1="40" x2="50"  y2="50" stroke="#ff0000"/>
      <line x1="50" y1="50" x2="100" y2="60" stroke="#ff0000"/>
    </svg>
""")


# Four strokes meeting at (50, 50), colors alternating red/blue/red/blue.
# Whatever order the optimizer chooses, it must group same-colored strokes
# together and never chain across the color boundary.
STAR_FOUR_COLOR_SVG = dedent("""\
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
      <line x1="50" y1="50" x2="0"   y2="40" stroke="#ff0000"/>
      <line x1="50" y1="50" x2="100" y2="40" stroke="#0000ff"/>
      <line x1="50" y1="50" x2="0"   y2="60" stroke="#ff0000"/>
      <line x1="50" y1="50" x2="100" y2="60" stroke="#0000ff"/>
    </svg>
""")


class TestMultiColorChaining:
    def test_different_color_touching_strokes_do_not_chain(self, tmp_path, capsys):
        svg = tmp_path / "multi.svg"
        svg.write_text(MULTICOLOR_TOUCHING_SVG)
        gcode = _run_cli(svg, tmp_path)

        # Two distinct colors → exactly two pen-select calls (one per pen).
        assert gcode.count("; --- pen select:") == 2
        # No chain comment may appear — the only stroke transition crosses colors.
        assert "chain stroke" not in gcode
        # Pen-up appears once per color: once when transitioning red→blue, once at end.
        assert gcode.count("; --- pen up:") == 2

    def test_pen_select_appears_between_color_change(self, tmp_path, capsys):
        """The pen-select for the second color must come AFTER the first stroke,
        not before it — i.e. the carousel actually rotates at the color boundary."""
        svg = tmp_path / "multi.svg"
        svg.write_text(MULTICOLOR_TOUCHING_SVG)
        gcode = _run_cli(svg, tmp_path)

        first_select = gcode.index("pen select: rotate carousel CCW to slot 1")
        first_draw = gcode.index("draw stroke #001")
        second_select = gcode.index("pen select: rotate carousel CCW to slot 2")
        second_draw = gcode.index("draw stroke #002")
        assert first_select < first_draw < second_select < second_draw

    def test_same_color_touching_strokes_do_chain(self, tmp_path, capsys):
        """Positive control: when colors match and endpoints touch, chaining DOES happen.
        Run with --no-optimize-path so the result depends only on the emit logic
        (the greedy NN optimizer would reverse one stroke and break the touch on
        this tiny 2-stroke input)."""
        svg = tmp_path / "single.svg"
        svg.write_text(SAMECOLOR_TOUCHING_SVG)
        gcode = _run_cli(svg, tmp_path, extra_args=["--no-optimize-path"])

        # Only one color → exactly one pen-select (carousel rotates to slot once).
        assert gcode.count("; --- pen select:") == 1
        # The two touching same-color strokes should chain into one pen-down.
        assert "chain stroke" in gcode
        # Only one pen-up: at the very end (no lift between the chained strokes).
        assert gcode.count("; --- pen up:") == 1

    def test_alternating_colors_at_star_point(self, tmp_path, capsys):
        """4 strokes meeting at one point, alternating colors. After optimization
        the strokes are grouped by pen — but within a pen group the entry-point
        XY may or may not match the previous color's exit. What matters is that
        NO chain ever crosses a pen boundary."""
        svg = tmp_path / "star.svg"
        svg.write_text(STAR_FOUR_COLOR_SVG)
        gcode = _run_cli(svg, tmp_path)

        # Two distinct pens → exactly two pen-select calls (grouped by optimizer).
        assert gcode.count("; --- pen select:") == 2

        # Scan every "chain stroke" comment: each must follow a same-color stroke.
        # We track the current pen as we walk the gcode in order.
        cur_pen = None
        for line in gcode.splitlines():
            if "pen select: rotate carousel CCW to slot" in line:
                # Extract the slot number from the comment.
                slot = int(line.split("slot")[1].split("(")[0].strip())
                cur_pen = slot
            elif "chain stroke" in line:
                # Chain comments include "pen=N"; the chained stroke MUST be on cur_pen.
                pen_field = line.split("pen=")[1].split()[0]
                chained_pen = int(pen_field)
                assert chained_pen == cur_pen, (
                    f"chain crossed color boundary: cur_pen={cur_pen}, "
                    f"chained stroke pen={chained_pen} in line: {line}"
                )

    def test_chaining_can_be_disabled_with_eps_zero(self, tmp_path, capsys):
        """When --connect-eps-mm=0, same-color touching strokes also lift —
        used as a debug switch when you suspect chaining is causing artifacts."""
        svg = tmp_path / "single.svg"
        svg.write_text(SAMECOLOR_TOUCHING_SVG)
        gcode = _run_cli(svg, tmp_path, extra_args=["--connect-eps-mm", "0"])

        # No chaining even though colors match and endpoints touch.
        assert "chain stroke" not in gcode
        # Two strokes → two pen-downs and two pen-ups.
        # (G101 appears 3x per pen-down, so count "; --- pen down: ..." comment instead.)
        assert gcode.count("; --- pen down:") == 2
        assert gcode.count("; --- pen up:") == 2
