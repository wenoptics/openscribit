"""SVG parsing: extract stroked paths with inherited presentation attributes and pen assignment."""
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, Iterable, Iterator, List, Optional, Tuple

from svgpathtools import Path, parse_path
from svgpathtools.svg_to_paths import (
    ellipse2pathd,
    line2pathd,
    polygon2pathd,
    polyline2pathd,
    rect2pathd,
)


RENDERABLE_TAGS = frozenset(
    {"path", "polyline", "polygon", "line", "rect", "circle", "ellipse"}
)

# Presentation attributes that inherit from ancestor elements.
INHERITABLE_PRESENTATION_ATTRS = ("stroke", "fill", "stroke-width", "opacity")


def clamp_int(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def parse_color(s: Optional[str]) -> Optional[str]:
    """Normalize common SVG color formats. Returns None for 'none'/'transparent'/missing."""
    if not s:
        return None
    s = s.strip().lower()
    if s in ("none", "transparent"):
        return None
    if s.startswith("#"):
        if len(s) == 4:
            r, g, b = s[1], s[2], s[3]
            return f"#{r}{r}{g}{g}{b}{b}"
        if len(s) == 7:
            return s
        return s
    m = re.match(r"rgb\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\)", s)
    if m:
        r, g, b = (int(m.group(1)), int(m.group(2)), int(m.group(3)))
        return f"#{clamp_int(r,0,255):02x}{clamp_int(g,0,255):02x}{clamp_int(b,0,255):02x}"
    return s


def _local_tag(tag: str) -> str:
    """Strip XML namespace prefix (e.g. '{ns}path' -> 'path')."""
    return tag.rsplit("}", 1)[-1]


def _parse_style(style: Optional[str]) -> Dict[str, str]:
    out: Dict[str, str] = {}
    if not style:
        return out
    for piece in style.split(";"):
        if ":" in piece:
            k, v = piece.split(":", 1)
            out[k.strip()] = v.strip()
    return out


def resolve_inherited_attrs(
    el: ET.Element, inherited: Dict[str, str]
) -> Dict[str, str]:
    """Merge ancestor-inherited attrs with this element's own; style overrides presentation attrs."""
    resolved = dict(inherited)
    for k in INHERITABLE_PRESENTATION_ATTRS:
        v = el.get(k)
        if v is not None:
            resolved[k] = v
    for k, v in _parse_style(el.get("style")).items():
        resolved[k] = v
    return resolved


def _element_to_path_d(el: ET.Element) -> Optional[str]:
    tag = _local_tag(el.tag)
    if tag == "path":
        return el.get("d") or None
    if tag == "polyline":
        return polyline2pathd(el.attrib) or None
    if tag == "polygon":
        return polygon2pathd(el.attrib) or None
    if tag == "line":
        return line2pathd(el)
    if tag == "rect":
        return rect2pathd(el.attrib)
    if tag in ("circle", "ellipse"):
        return ellipse2pathd(el.attrib)
    return None


@dataclass
class RenderableElement:
    """SVG renderable element with inheritance-resolved presentation attrs."""
    attrs: Dict[str, str]
    d: str
    svg_id: str


def iter_renderable_elements(root: ET.Element) -> Iterable[RenderableElement]:
    """Yield renderable SVG elements with stroke/fill resolved through the ancestor chain."""
    counter = [0]

    def walk(el: ET.Element, inherited: Dict[str, str]) -> Iterable[RenderableElement]:
        resolved = resolve_inherited_attrs(el, inherited)
        if _local_tag(el.tag) in RENDERABLE_TAGS:
            counter[0] += 1
            d = _element_to_path_d(el)
            if d:
                svg_id = el.get("id") or f"path_{counter[0]}"
                yield RenderableElement(attrs=resolved, d=d, svg_id=svg_id)
        for child in el:
            yield from walk(child, resolved)

    yield from walk(root, {})


def effective_stroke(attrs: Dict[str, str]) -> Optional[str]:
    """Return the stroke color to draw with, or None to skip this element.

    Rules:
      - Explicit stroke (not 'none') → use it.
      - No stroke AND no fill → assume black.
      - Visible fill but no stroke → skip (filled shape, not a line).
    """
    stroke = parse_color(attrs.get("stroke"))
    if stroke is not None:
        return stroke
    fill = parse_color(attrs.get("fill"))
    if fill is None:
        return "#000000"
    return None


class PenAssigner:
    """Maps stroke colors to pen slots (1..max_pens) in first-seen order.

    Colors beyond max_pens fall back to default_pen.
    """

    def __init__(self, default_pen: int, max_pens: int = 4):
        self.default_pen = default_pen
        self.max_pens = max_pens
        self.color_to_pen: Dict[str, int] = {}
        self._next_pen = 1

    def assign(self, color: str) -> int:
        if color not in self.color_to_pen and self._next_pen <= self.max_pens:
            self.color_to_pen[color] = self._next_pen
            self._next_pen += 1
        return self.color_to_pen.get(color, self.default_pen)


def split_into_continuous_subpaths(path: Path, *, eps: float = 1e-7) -> List[Path]:
    """Split a Path into continuous subpaths (separated by implicit M commands).

    A single SVG <path d="..."> can encode multiple disjoint subpaths. Drawing
    them as one continuous stroke would cause pen-down jumps between subpaths.
    """
    f = getattr(path, "continuous_subpaths", None)
    if callable(f):
        try:
            subs = list(f())
            return [sp for sp in subs if len(sp) > 0]
        except Exception:
            pass

    if len(path) == 0:
        return []

    out: List[Path] = []
    cur: List = []
    prev_end: Optional[complex] = None

    def close_enough(a: complex, b: complex) -> bool:
        return abs(a - b) <= eps

    for seg in path:
        s = getattr(seg, "start", None)
        e = getattr(seg, "end", None)
        if s is None or e is None:
            cur.append(seg)
            prev_end = e if e is not None else prev_end
            continue
        if prev_end is not None and not close_enough(s, prev_end):
            if cur:
                out.append(Path(*cur))
            cur = [seg]
        else:
            cur.append(seg)
        prev_end = e

    if cur:
        out.append(Path(*cur))
    return out


def sample_path_uniform_t(path: Path, n: int) -> List[complex]:
    """Uniform-in-t sampling: returns n+1 points including endpoint."""
    n = max(1, n)
    return [path.point(i / n) for i in range(n + 1)]


def load_drawable_paths(
    svg_path: str, default_pen: int
) -> Tuple[List[Tuple[Path, int, str]], Dict[str, int]]:
    """Parse SVG and return list of (Path, pen_id, svg_id) and the color→pen map."""
    tree = ET.parse(svg_path)
    root = tree.getroot()

    assigner = PenAssigner(default_pen=default_pen)
    drawable: List[Tuple[Path, int, str]] = []

    for el in iter_renderable_elements(root):
        stroke = effective_stroke(el.attrs)
        if stroke is None:
            continue
        pen = assigner.assign(stroke)
        drawable.append((parse_path(el.d), pen, el.svg_id))

    return drawable, assigner.color_to_pen


def compute_svg_bbox(
    drawable: Iterable[Tuple[Path, int, str]]
) -> Tuple[float, float, float, float]:
    xmin = ymin = float("inf")
    xmax = ymax = float("-inf")
    any_path = False
    for p, _, _ in drawable:
        any_path = True
        bx0, bx1, by0, by1 = p.bbox()
        xmin = min(xmin, bx0)
        xmax = max(xmax, bx1)
        ymin = min(ymin, by0)
        ymax = max(ymax, by1)
    if not any_path:
        raise SystemExit("No drawable paths.")
    return xmin, xmax, ymin, ymax
