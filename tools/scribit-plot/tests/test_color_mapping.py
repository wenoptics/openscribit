"""Tests for the SVG color → pen mapping logic in scribit_svg_to_gcode."""
from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from textwrap import dedent

import pytest

# Make the script importable as a module.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scribit_svg_to_gcode import (  # noqa: E402
    PenAssigner,
    effective_stroke,
    iter_renderable_elements,
    load_drawable_paths,
    parse_color,
    resolve_inherited_attrs,
)


# ----------------------------
# parse_color
# ----------------------------

class TestParseColor:
    def test_returns_none_for_missing(self):
        assert parse_color(None) is None
        assert parse_color("") is None

    def test_returns_none_for_none_keyword(self):
        assert parse_color("none") is None
        assert parse_color("NONE") is None
        assert parse_color("transparent") is None

    def test_short_hex_expands(self):
        assert parse_color("#f03") == "#ff0033"

    def test_long_hex_lowercased(self):
        assert parse_color("#FF2600") == "#ff2600"

    def test_rgb_function_form(self):
        assert parse_color("rgb(255, 38, 0)") == "#ff2600"


# ----------------------------
# effective_stroke
# ----------------------------

class TestEffectiveStroke:
    def test_explicit_stroke_used(self):
        assert effective_stroke({"stroke": "#ff2600"}) == "#ff2600"

    def test_no_stroke_no_fill_defaults_to_black(self):
        assert effective_stroke({}) == "#000000"

    def test_stroke_none_falls_back_to_fill_check(self):
        # stroke="none" + no fill → still draw as black (matches prior behavior).
        assert effective_stroke({"stroke": "none"}) == "#000000"

    def test_filled_shape_without_stroke_is_skipped(self):
        # Visible fill but no stroke → not a line, skip.
        assert effective_stroke({"fill": "#abcdef"}) is None

    def test_fill_none_with_no_stroke_defaults_to_black(self):
        assert effective_stroke({"fill": "none"}) == "#000000"

    def test_stroke_wins_over_fill(self):
        assert effective_stroke({"stroke": "#ff2600", "fill": "#abcdef"}) == "#ff2600"


# ----------------------------
# PenAssigner
# ----------------------------

class TestPenAssigner:
    def test_first_four_colors_get_pens_1_through_4(self):
        a = PenAssigner(default_pen=1)
        assert a.assign("#000000") == 1
        assert a.assign("#ff0000") == 2
        assert a.assign("#00ff00") == 3
        assert a.assign("#0000ff") == 4
        assert a.color_to_pen == {
            "#000000": 1, "#ff0000": 2, "#00ff00": 3, "#0000ff": 4,
        }

    def test_repeated_color_returns_same_pen(self):
        a = PenAssigner(default_pen=1)
        assert a.assign("#ff2600") == 1
        assert a.assign("#000000") == 2
        assert a.assign("#ff2600") == 1
        assert a.assign("#000000") == 2

    def test_overflow_uses_default_pen(self):
        a = PenAssigner(default_pen=3)
        for c in ("#111111", "#222222", "#333333", "#444444"):
            a.assign(c)
        # 5th distinct color overflows.
        assert a.assign("#555555") == 3
        # Overflow color is NOT recorded in the map.
        assert "#555555" not in a.color_to_pen


# ----------------------------
# resolve_inherited_attrs
# ----------------------------

class TestResolveInheritedAttrs:
    def test_inherits_stroke_from_parent(self):
        el = ET.fromstring('<polyline points="0,0 1,1"/>')
        resolved = resolve_inherited_attrs(el, {"stroke": "#ff2600"})
        assert resolved["stroke"] == "#ff2600"

    def test_own_stroke_overrides_inherited(self):
        el = ET.fromstring('<polyline points="0,0 1,1" stroke="#000000"/>')
        resolved = resolve_inherited_attrs(el, {"stroke": "#ff2600"})
        assert resolved["stroke"] == "#000000"

    def test_style_overrides_presentation_attr(self):
        # Per CSS, the style attribute wins over presentation attributes.
        el = ET.fromstring(
            '<polyline points="0,0 1,1" stroke="#000000" style="stroke:#ff2600"/>'
        )
        resolved = resolve_inherited_attrs(el, {})
        assert resolved["stroke"] == "#ff2600"


# ----------------------------
# iter_renderable_elements (the actual bug)
# ----------------------------

SVG_GROUPED_STROKE = dedent("""\
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
      <g id="reds" stroke="#ff2600">
        <polyline points="0,0 10,10"/>
        <polyline points="10,10 20,5"/>
      </g>
      <g id="blacks" stroke="#000000">
        <polyline points="20,20 30,30"/>
      </g>
    </svg>
""")


class TestIterRenderableElements:
    def test_inherits_stroke_from_group(self):
        root = ET.fromstring(SVG_GROUPED_STROKE)
        elements = list(iter_renderable_elements(root))
        strokes = [el.attrs.get("stroke") for el in elements]
        assert strokes == ["#ff2600", "#ff2600", "#000000"]

    def test_yields_one_per_renderable_element(self):
        root = ET.fromstring(SVG_GROUPED_STROKE)
        elements = list(iter_renderable_elements(root))
        assert len(elements) == 3

    def test_handles_mixed_renderable_types(self):
        svg = dedent("""\
            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
              <g stroke="#ff2600">
                <path d="M0 0 L10 10"/>
                <polyline points="0,0 5,5"/>
                <line x1="0" y1="0" x2="1" y2="1"/>
                <rect x="0" y="0" width="10" height="10"/>
              </g>
            </svg>
        """)
        root = ET.fromstring(svg)
        elements = list(iter_renderable_elements(root))
        assert len(elements) == 4
        assert all(el.attrs.get("stroke") == "#ff2600" for el in elements)


# ----------------------------
# load_drawable_paths (integration)
# ----------------------------

class TestLoadDrawablePaths:
    def test_grouped_strokes_get_distinct_pens(self, tmp_path):
        """The core regression: polylines inside <g stroke="X"> must use X."""
        svg = tmp_path / "two_colors.svg"
        svg.write_text(SVG_GROUPED_STROKE)
        drawable, color_to_pen = load_drawable_paths(str(svg), default_pen=1)

        # 2 red polylines + 1 black polyline
        assert len(drawable) == 3
        assert color_to_pen == {"#ff2600": 1, "#000000": 2}

        pens = [pen for _path, pen, _id in drawable]
        assert pens == [1, 1, 2]

    def test_skips_filled_shape_without_stroke(self, tmp_path):
        svg = tmp_path / "filled.svg"
        svg.write_text(dedent("""\
            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
              <rect x="0" y="0" width="10" height="10" fill="#abcdef"/>
              <polyline points="0,0 5,5" stroke="#ff2600"/>
            </svg>
        """))
        drawable, color_to_pen = load_drawable_paths(str(svg), default_pen=1)
        assert len(drawable) == 1
        assert color_to_pen == {"#ff2600": 1}

    def test_defaults_to_black_when_no_stroke_or_fill(self, tmp_path):
        svg = tmp_path / "bare.svg"
        svg.write_text(dedent("""\
            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
              <path d="M0 0 L10 10"/>
            </svg>
        """))
        drawable, color_to_pen = load_drawable_paths(str(svg), default_pen=1)
        assert color_to_pen == {"#000000": 1}
        assert len(drawable) == 1

    def test_overflow_color_uses_default_pen(self, tmp_path):
        svg = tmp_path / "many.svg"
        svg.write_text(dedent("""\
            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
              <path d="M0 0 L1 1" stroke="#111111"/>
              <path d="M0 0 L1 1" stroke="#222222"/>
              <path d="M0 0 L1 1" stroke="#333333"/>
              <path d="M0 0 L1 1" stroke="#444444"/>
              <path d="M0 0 L1 1" stroke="#555555"/>
            </svg>
        """))
        drawable, color_to_pen = load_drawable_paths(str(svg), default_pen=2)
        # First four colors mapped; fifth (#555555) is NOT in the map.
        assert color_to_pen == {
            "#111111": 1, "#222222": 2, "#333333": 3, "#444444": 4,
        }
        pens = [pen for _path, pen, _id in drawable]
        assert pens == [1, 2, 3, 4, 2]  # last falls back to default_pen=2


# ----------------------------
# Regression test on the real failing file (if present).
# ----------------------------

REAL_SVG = Path(__file__).resolve().parents[3] / ".try" / "060530-duo-color-flow.svg"


@pytest.mark.skipif(not REAL_SVG.exists(), reason="reference SVG not in tree")
class TestRealDuoColorFlow:
    def test_detects_both_colors(self):
        drawable, color_to_pen = load_drawable_paths(str(REAL_SVG), default_pen=1)
        assert set(color_to_pen.keys()) == {"#000000", "#ff2600"}
        # Each color should map to a distinct pen.
        assert len(set(color_to_pen.values())) == 2

    def test_pens_distributed_across_both_colors(self):
        drawable, _color_to_pen = load_drawable_paths(str(REAL_SVG), default_pen=1)
        pens = {pen for _p, pen, _id in drawable}
        assert pens == {1, 2}
