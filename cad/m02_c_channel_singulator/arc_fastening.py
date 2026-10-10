"""The free-end fastening of the split exit sector — geometry shared by
the stator (the boss and its insert hole), the drive (the skirt's
countersunk hole) and the root (the screw stub and its claims).

On a plain-wall stage the exit opening runs the full wall height, so it
cuts the liner sector it falls in into two arcs, each held by the ear at
its own seam and free at the opening (`stator/assembly.py`
``sector_solid_count``). Owner, 2026-09-27: fasten each free end into
the skirt. Form (parameters in ``params.py`` §Free-end fastening):

  - a **boss** on the arc's outer cone face, just past the exit cut on
    the wall side: it fills the wedge of air between the liner's outer
    cone (r = R_STATOR_OUTER_BASE + z) and the skirt's inner face
    (r = SKIRT_R_IN) from ARC_BOSS_Z_LO up to SKIRT_TOP_Z, where that
    wedge closes under the liner's flange. Its outer face lies on the
    skirt's inner face — the bearing contact the screw clamps;
  - one **countersunk screw** (ISO 10642 M3) run in radially through a
    countersunk hole in the skirt, into a **heat-set insert** in a hole
    in the boss. The head seats below the skirt's outer face, so nothing
    stands outside the ring: out there, on S1's downstream side, is the
    LOAD ramp's chute.

Both horizontal holes are teardrops, roofed ARC_HOLE_ROOF_DEG from
vertical toward each part's print-up: +Z for the bowl (printed floor
down), −Z for the liner (printed inverted).

Side frame: the free end at azimuth ``sign · EXIT_ARC_DEG / 2`` (sign
+1 or −1). Geometry is built for sign +1 about +X — the cut plane is
y = 0, the wall side is +y — then mirrored in XZ for sign −1 and turned
onto the end's azimuth. The screw axis runs parallel to the cut plane
(so the boss and the countersink sit square to it), ARC_BOSS_EXIT_CLEAR
+ ARC_BOSS_W / 2 in from it at ARC_SCREW_Z; about 3° off radial.
"""

from __future__ import annotations

import math

from build123d import Axis, Box, Circle, Cone, Cylinder, Location, Part, Plane, Polygon, Pos, Rot, extrude, loft, mirror, revolve

from .params import (
    ARC_BOSS_EXIT_CLEAR,
    ARC_BOSS_W,
    ARC_BOSS_Z_LO,
    ARC_CSINK_CLEAR,
    ARC_HOLE_ROOF_DEG,
    ARC_HOLE_TIP_CLEAR,
    ARC_INSERT_HOLE_D,
    ARC_INSERT_L,
    ARC_SCREW_ACCESS_D,
    ARC_SCREW_ACCESS_L,
    ARC_SCREW_SEAT_CLEAR,
    ARC_SCREW_D,
    ARC_SCREW_HEAD_D,
    ARC_SCREW_HOLE_CLEAR,
    ARC_SCREW_L,
    ARC_SCREW_Z,
    EXIT_ARC_DEG,
    EXIT_FLOOR_T,
    R_STATOR_OUTER_BASE,
    R_STATOR_OUTER_KINK,
    SKIRT_R_IN,
    SKIRT_TOP_Z,
    STATOR_WALL_H,
    V_HALF_ANGLE_DEG,
)
from .seams import SEAM_EAR_OVERLAP


FREE_END_SIGNS = (+1, -1)

# Derived from the bought parts' dims.
ARC_SCREW_HOLE_D = ARC_SCREW_D + ARC_SCREW_HOLE_CLEAR          # 3.4 through the skirt
ARC_CSINK_D = ARC_SCREW_HEAD_D + ARC_CSINK_CLEAR               # 6.4 at the skirt's outer face
ARC_SCREW_HEAD_H = (ARC_SCREW_HEAD_D - ARC_SCREW_D) / 2.0      # 90° head: 1.5

# Where the axis runs, in the side frame.
_AXIS_Y = ARC_BOSS_EXIT_CLEAR + ARC_BOSS_W / 2.0               # 8.0 in from the cut plane
_X_SKIRT_OUT = math.sqrt(R_STATOR_OUTER_KINK**2 - _AXIS_Y**2)  # skirt outer face on the axis
_X_SKIRT_IN = math.sqrt(SKIRT_R_IN**2 - _AXIS_Y**2)            # skirt inner face = boss face
ARC_SCREW_HEAD_X = _X_SKIRT_OUT - ARC_CSINK_CLEAR / 2.0        # head top, on the axis
_X_TIP = ARC_SCREW_HEAD_X - ARC_SCREW_L
_X_HOLE_BOTTOM = _X_TIP - ARC_HOLE_TIP_CLEAR
ARC_INSERT_HOLE_DEPTH = _X_SKIRT_IN - _X_HOLE_BOTTOM           # from the boss face

# The bearing claim's floor: half the boss's designed face on the skirt
# (ARC_BOSS_W × its height), so the two holes (~14 mm² between them)
# pass and a boss standing off the skirt, or split by a bowl seam, does not.
ARC_BOSS_BEARING_MIN_MM2 = ARC_BOSS_W * (SKIRT_TOP_Z - ARC_BOSS_Z_LO) / 2.0


def free_end_name(sign: int) -> str:
    """Suffix for the free end at ``sign · EXIT_ARC_DEG / 2``."""
    return {+1: "pos", -1: "neg"}[sign]


def _to_end(shape: Part, sign: int) -> Part:
    """A side-frame body onto the free end at ``sign``."""
    if sign < 0:
        shape = mirror(shape, about=Plane.XZ)
    return Rot(0, 0, sign * EXIT_ARC_DEG / 2.0) * shape


def _teardrop_face(d: float, x: float, apex_up: bool):
    """A teardrop section across the screw axis at ``x``: a circle of
    diameter ``d`` roofed to a point on the ``apex_up`` side (+Z if
    true, −Z if not), its two roof lines ARC_HOLE_ROOF_DEG from
    vertical — the overhang they print at."""
    r = d / 2.0
    t = math.radians(90.0 - ARC_HOLE_ROOF_DEG)          # tangent points' angle from the top
    s = 1.0 if apex_up else -1.0
    # Wound counter-clockwise either way up: a clockwise Polygon
    # added to the circle silently contributes nothing.
    roof = Polygon(
        (-s * r * math.sin(t), s * r * math.cos(t)),
        (s * r * math.sin(t), s * r * math.cos(t)),
        (0.0, s * r / math.cos(t)),
        align=None,
    )
    plane = Plane(origin=(x, _AXIS_Y, ARC_SCREW_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    return (plane * (Circle(r) + roof)).faces()[0]


def _teardrop_cutter(d: float, x0: float, x1: float, apex_up: bool) -> Part:
    """A horizontal teardrop hole (``_teardrop_face``) along +X on the
    screw axis from ``x0`` to ``x1``."""
    return extrude(_teardrop_face(d, x0, apex_up), amount=x1 - x0, dir=(1, 0, 0))


def arc_boss(sign: int = +1) -> Part:
    """The boss at the free end ``sign``, in m02-local coordinates, as
    unioned onto the liner — before its insert hole is cut. It overlaps
    the cone SEAM_EAR_OVERLAP radially so the two fuse."""
    apex = ARC_INSERT_HOLE_D / 2.0 / math.cos(math.radians(90.0 - ARC_HOLE_ROOF_DEG))
    assert 0.0 < ARC_BOSS_Z_LO < ARC_SCREW_Z - apex and ARC_SCREW_Z + ARC_INSERT_HOLE_D / 2.0 < SKIRT_TOP_Z, (
        "the insert hole must sit inside the boss's height"
    )
    tan_v = math.tan(math.radians(V_HALF_ANGLE_DEG))
    cone = lambda z: R_STATOR_OUTER_BASE + z * tan_v         # noqa: E731 — the liner's outer cone
    profile = Plane.XZ * Polygon(
        (cone(ARC_BOSS_Z_LO) - SEAM_EAR_OVERLAP, ARC_BOSS_Z_LO),
        (SKIRT_R_IN, ARC_BOSS_Z_LO),
        (SKIRT_R_IN, SKIRT_TOP_Z),
        (cone(SKIRT_TOP_Z) - SEAM_EAR_OVERLAP, SKIRT_TOP_Z),
        align=None,
    )
    ring = revolve(profile, axis=Axis.Z, revolution_arc=30.0)
    slab = Pos(SKIRT_R_IN / 2.0 + 50.0, ARC_BOSS_EXIT_CLEAR + ARC_BOSS_W / 2.0, SKIRT_TOP_Z / 2.0) * Box(
        SKIRT_R_IN, ARC_BOSS_W, SKIRT_TOP_Z + 2.0,
    )
    return _to_end(ring & slab, sign)


def boss_hole(sign: int = +1) -> Part:
    """Cutter for the insert hole in the boss at ``sign``: a teardrop
    over ARC_INSERT_HOLE_D, roofed toward the liner's print-up (−Z),
    from ARC_INSERT_HOLE_DEPTH inside the boss face out past it."""
    assert ARC_INSERT_HOLE_DEPTH >= ARC_INSERT_L, (
        f"insert hole {ARC_INSERT_HOLE_DEPTH:.2f} deep is shorter than the insert ({ARC_INSERT_L})"
    )
    return _to_end(_teardrop_cutter(ARC_INSERT_HOLE_D, _X_HOLE_BOTTOM, _X_SKIRT_IN + 1.0, apex_up=False), sign)


def skirt_hole(sign: int = +1) -> Part:
    """Cutter for the skirt's countersunk clearance hole at ``sign``:
    a teardrop over ARC_SCREW_HOLE_D roofed toward the bowl's print-up
    (+Z), through the skirt, and a 90° countersink from the outer
    face ARC_CSINK_D across."""
    csink_h = (ARC_CSINK_D - ARC_SCREW_HOLE_D) / 2.0
    assert csink_h < R_STATOR_OUTER_KINK - SKIRT_R_IN, "countersink deeper than the skirt"
    hole = _teardrop_cutter(ARC_SCREW_HOLE_D, _X_SKIRT_IN - 1.0, _X_SKIRT_OUT + 1.0, apex_up=True)
    # The countersink is a teardrop too — a ruled loft from the hole's
    # section at the countersink's foot to ARC_CSINK_D at the face, so
    # its roof never faces down past ARC_HOLE_ROOF_DEG either (a plain
    # 90° cone's crown reads 45° plus tessellation); carried 2 mm
    # outside the face so the cut is clean.
    x_foot = _X_SKIRT_OUT - csink_h
    csink = loft([
        _teardrop_face(ARC_SCREW_HOLE_D, x_foot, apex_up=True),
        _teardrop_face(ARC_CSINK_D, _X_SKIRT_OUT, apex_up=True),
    ], ruled=True) + _teardrop_cutter(ARC_CSINK_D, _X_SKIRT_OUT, _X_SKIRT_OUT + 2.0, apex_up=True)
    return _to_end(hole + csink, sign)


def arc_screw() -> Part:
    """The screw stub on its own axis: head top at the origin, shank
    down −Z — a 90° head cone ARC_SCREW_HEAD_D across and a plain
    ARC_SCREW_D cylinder (the thread's major diameter: an outer
    envelope). Bought, not inspected."""
    head = Pos(0, 0, -ARC_SCREW_HEAD_H) * Cone(ARC_SCREW_D / 2.0, ARC_SCREW_HEAD_D / 2.0, ARC_SCREW_HEAD_H, align=None)
    shank = Pos(0, 0, -ARC_SCREW_L) * Cylinder(ARC_SCREW_D / 2.0, ARC_SCREW_L - ARC_SCREW_HEAD_H, align=None)
    return head + shank


def arc_screw_location(sign: int) -> Location:
    """Where ``arc_screw()`` goes for the free end at ``sign``, m02-local."""
    return Rot(0, 0, sign * EXIT_ARC_DEG / 2.0) * Pos(ARC_SCREW_HEAD_X, sign * _AXIS_Y, ARC_SCREW_Z) * Rot(0, 90, 0)


def driver_corridor(sign: int) -> Part:
    """The driver's way in to the screw at ``sign``: Ø ARC_SCREW_ACCESS_D
    out along its axis from the head top, ARC_SCREW_ACCESS_L long,
    m02-local. A keep-out, never placed."""
    return arc_screw_location(sign) * (
        Pos(0, 0, ARC_SCREW_ACCESS_L / 2.0) * Cylinder(ARC_SCREW_ACCESS_D / 2.0, ARC_SCREW_ACCESS_L)
    )


def driver_seat(sign: int) -> Part:
    """The screw's seat in the skirt at ``sign``, as a claim region on
    the bowl sector it is driven into, m02-local (the sectors are drawn
    there): a plug on the screw axis ARC_SCREW_SEAT_CLEAR wider all
    round than the driver corridor, and only as deep as the skirt —
    1 mm past each of its faces, which curve away from the axis. The
    corridor starts inside the countersink, so this much of the bowl
    cannot be held clear of it; everything else of the bowl can."""
    x0, x1 = _X_SKIRT_IN - 1.0, _X_SKIRT_OUT + 1.0
    plug = Pos((x0 + x1) / 2.0, _AXIS_Y, ARC_SCREW_Z) * Rot(0, 90, 0) * Cylinder(
        ARC_SCREW_ACCESS_D / 2.0 + ARC_SCREW_SEAT_CLEAR, x1 - x0,
    )
    return _to_end(plug, sign)


def exit_corridor() -> Part:
    """The exit opening's own volume carried outward — the wedge
    ±EXIT_ARC_DEG/2 about +X, from the tongue's underside to the wall
    top, out to 60 mm past the skirt. Measuring geometry only, never
    placed: a brick leaves through this, so the fastening keeps out of
    it."""
    h = math.radians(EXIT_ARC_DEG / 2.0)
    r = (R_STATOR_OUTER_KINK + 60.0) / math.cos(h)
    wedge = Polygon((0.0, 0.0), (r * math.cos(-h), r * math.sin(-h)), (r * math.cos(h), r * math.sin(h)), align=None)
    return Pos(0, 0, -EXIT_FLOOR_T) * extrude(wedge, amount=STATOR_WALL_H + EXIT_FLOOR_T)


def arc_screw_azimuth_deg(sign: int) -> float:
    """m02-local azimuth of the screw's head, for naming the bowl sector
    it passes through."""
    return sign * (EXIT_ARC_DEG / 2.0 + math.degrees(math.atan2(_AXIS_Y, ARC_SCREW_HEAD_X)))


def boss_span_deg(sign: int) -> tuple[float, float]:
    """The azimuths the boss at ``sign`` covers, ordered, taken at its
    innermost radius (the cone at ARC_BOSS_Z_LO), where they are widest."""
    r = R_STATOR_OUTER_BASE + ARC_BOSS_Z_LO * math.tan(math.radians(V_HALF_ANGLE_DEG)) - SEAM_EAR_OVERLAP
    near = EXIT_ARC_DEG / 2.0 + math.degrees(math.asin(ARC_BOSS_EXIT_CLEAR / r))
    far = EXIT_ARC_DEG / 2.0 + math.degrees(math.asin((ARC_BOSS_EXIT_CLEAR + ARC_BOSS_W) / r))
    return (near, far) if sign > 0 else (-far, -near)


def bearing_area_mm2(liner: Part, bowl: Part, sign: int) -> float:
    """Area over which ``liner`` and ``bowl`` share the skirt's inner
    face (the cylinder r = SKIRT_R_IN) at the free end ``sign``: the
    common of the two parts' faces lying on it, the liner's taken where
    their centre falls inside ``boss_span_deg(sign)``. Only a boss
    reaches that face from the liner below SKIRT_TOP_Z — the liner's
    own r = SKIRT_R_IN face is its cylinder zone's inner face, above
    the skirt — so this is the boss's bearing patch, holes out: zero
    if the boss stands off the skirt, or bears on another bowl sector."""
    lo, hi = boss_span_deg(sign)
    def on_skirt_face(part: Part):
        return [
            f for f in part.faces()
            # (a trimmed cylinder reports no radius; none of those is the skirt's face)
            if f.geom_type.name == "CYLINDER" and f.radius is not None and abs(f.radius - SKIRT_R_IN) < 1e-6
        ]
    area = 0.0
    for fa in on_skirt_face(liner):
        c = fa.center()
        if not lo <= math.degrees(math.atan2(c.Y, c.X)) <= hi:
            continue
        for fb in on_skirt_face(bowl):
            common = fa & fb
            area += sum(f.area for f in common.faces()) if common is not None else 0.0
    return area
