"""Horizontal holes as teardrops: a circle roofed to a point on its
print-up side, so the hole's crown never faces down past ``roof_deg``
from vertical. Shared by the liner's free-end fastening
(`arc_fastening.py`) and the seam bolts (`seams.py`).
"""

from __future__ import annotations

import math

from build123d import Circle, Face, Part, Plane, Polygon, Vector, extrude


def _plane(origin: tuple[float, float, float], axis: tuple[float, float, float]) -> Plane:
    """The section plane across a horizontal ``axis`` at ``origin``,
    its local +y on world +Z: x_dir = Z × axis, so y = axis × x = Z."""
    z_dir = Vector(*axis).normalized()
    assert abs(z_dir.Z) < 1e-9, "a teardrop's axis must be horizontal"
    return Plane(origin=origin, x_dir=Vector(0, 0, 1).cross(z_dir), z_dir=z_dir)


def teardrop_face(
    d: float,
    origin: tuple[float, float, float],
    axis: tuple[float, float, float],
    *,
    apex_up: bool,
    roof_deg: float,
) -> Face:
    """A teardrop section across ``axis`` at ``origin``: a circle of
    diameter ``d`` roofed to a point on the ``apex_up`` side (+Z if
    true, −Z if not), its two roof lines ``roof_deg`` from vertical —
    the overhang they print at."""
    r = d / 2.0
    t = math.radians(90.0 - roof_deg)  # tangent points' angle from the top
    s = 1.0 if apex_up else -1.0
    # Wound counter-clockwise either way up: a clockwise Polygon
    # added to the circle silently contributes nothing.
    roof = Polygon(
        (-s * r * math.sin(t), s * r * math.cos(t)),
        (s * r * math.sin(t), s * r * math.cos(t)),
        (0.0, s * r / math.cos(t)),
        align=None,
    )
    return (_plane(origin, axis) * (Circle(r) + roof)).faces()[0]


def teardrop_cutter(
    d: float,
    origin: tuple[float, float, float],
    axis: tuple[float, float, float],
    length: float,
    *,
    apex_up: bool,
    roof_deg: float,
) -> Part:
    """A teardrop hole (``teardrop_face``) from ``origin`` ``length``
    along ``axis``."""
    return extrude(teardrop_face(d, origin, axis, apex_up=apex_up, roof_deg=roof_deg), amount=length, dir=axis)
