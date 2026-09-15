"""
M02 C-channel singulator — stator sub-assembly.

Fixed two-zone outer wall. The lower zone is sloped (V-cone) to
form the outer side of the V-trough where bricks ride; the upper
zone is a plain cylinder that just contains splash above the
pieces. Bricks ride in the V between the spinning rotor's flank
(inner side) and the stator wall's inner conical surface (outer
side), pressed outward against this wall as the rotor drags them
around. An angular arc on the +X side is cut away — pieces
reaching the exit fall off the outer edge under gravity into the
scan-station drop path (Module 3).

V1 is wall-only: a two-zone ring of thickness STATOR_WALL_T — V-
slope from z=0 to z=STATOR_V_H, then vertical cylinder from
STATOR_V_H to STATOR_WALL_H. The exit arc is cut through the full
wall height. The mounting interface is the skirt + foot ring of
`_notes/decisions/m02-chain.md` §5: the foot ring is the drive's
base plate (../drive/assembly.py, top at PLATE_TOP_Z = -18.5), so
the skirt — not yet modelled — drops from the kink to that plate.
No floor at z=0: the pinion runs under the wall at r 120–147.

The exit floor (`exit_floor`, a second part, 2026-09-15): before it,
a brick leaving the V at the exit had nothing under it but the base
plate 18.5 mm down — an uncut shelf out to r 155 that the brick
landed on and stayed on. The tongue is a flat sector at the V-bottom
plane spanning the exit wedge from just outside the ring gear's
tips to the skirt radius, so a brick slides out of the V and off the
stage instead of dropping into its own footprint (S1's ramp starts
flush with it, 1 mm outside; S2 / S3 deliver off its edge onto the
rotor below). An arc rib under its outer edge stands on the base
plate. Separate from the stator on purpose: printed with it, in the
stator's inverted orientation, it would be a ~100 mm bridge over the
open wedge.

The wall's V-zone sits at V_HALF_ANGLE from vertical on both
faces — a 45° overhang by the conventional build-plate metric.
Expect to print either with support on the outside, or inverted
(top-down on bed) so each layer is smaller than the one below it.
The cylindrical upper zone is trivially printable.

Coordinate frame:
    origin = rotor axis ∩ V-bottom level (shared with rotor/)
    +X     = exit-arc center azimuth; bricks drop out here
    +Z     = up
    z=0    = V-bottom. Stator wall's narrowest inner edge sits at
             this level; rotor base edge is also at this z in its
             own frame.

Standalone validation: stator alone + printability (FDM).
Cross-boundary rotor↔stator clearance runs in ../assembly.py.
"""

import math

from build123d import Axis, Box, Cylinder, Part, Plane, Polygon, Pos, Rot, extrude, revolve

from cad_khana.mechanism.assembly import Assembly

from ..materials import material_of
from ..params import (
    ADDENDUM,
    EXIT_ARC_DEG,
    PLATE_TOP_Z,
    R_STATOR_INNER_BASE,
    R_STATOR_INNER_KINK,
    R_STATOR_OUTER_BASE,
    R_STATOR_OUTER_KINK,
    RING_PITCH_D,
    ROTOR_BASE_DIAMETER,
    STATOR_V_H,
    STATOR_WALL_H,
    STATOR_WALL_T,
    V_HALF_ANGLE_DEG,
)


# --- Derived geometry --------------------------------------------------

# Wall radii (inner/outer at base and kink) and STATOR_WALL_T live in
# ..params since 2026-09-11: the drive's base plate shares the skirt
# OD (`R_STATOR_OUTER_KINK`), so they are an interface, not a
# stator-private detail.
_TAN_V = math.tan(math.radians(V_HALF_ANGLE_DEG))

# Downstream exit-arc wall: a radial paddle that plugs *one* edge of
# the exit cut — the edge pieces are moving TOWARD as the rotor drags
# them around. Without it, pieces ejected at the exit could roll back
# over the rotor's slope into the V on the downstream side of the
# opening; with it, they'd have to travel the long way around (past
# the full stator ring) to re-enter. EXIT_WALL_SIDE_SIGN picks which
# edge: +1 for CCW rotor rotation viewed from +Z (downstream =
# +exit_arc/2), -1 for CW.
#
# Shape: a radially-oriented plate whose inner edge tracks the rotor's
# nominal conical flank (r_rotor(z) = base_r − z·tan V) offset outward
# by EXIT_WALL_ROTOR_GAP. This makes the wall block *both* the V-bottom
# gap and the upper rotor hillside — a dropped piece can't sneak under
# or over it back into the V. Gap is non-zero because the faceted rotor
# varies with angle (peaks vs valleys) and the paddle's slanted face
# sits parallel to the cone, so radial gap converts to a smaller
# perpendicular distance (≈ gap/√2 at V_HALF_ANGLE=45°); picked so the
# worst-case perpendicular clearance at the rim corner stays above
# ROTOR_STATOR_GAP − GAP_CLEARANCE_TOLERANCE. Outer edge stops at the
# cylindrical wall (no tab sticking out).
EXIT_WALL_T = STATOR_WALL_T
EXIT_WALL_ROTOR_GAP = 3.0           # radial gap rotor-cone → paddle inner edge
EXIT_WALL_SIDE_SIGN = +1            # +1: CCW rotor, -1: CW rotor

WALL_MIN = 1.5

# Exit floor. Top face at z = 0 (the V-bottom plane, flush with the
# stator's inner base edge at r 122 so the brick meets no step);
# inner edge EXIT_FLOOR_GEAR_CLEAR outside the ring gear's tip
# circle (the flange's pitch cylinder + ADDENDUM — the cheap rotor
# has no teeth, so ../assembly.py measures it grown by ADDENDUM);
# outer edge at the skirt radius, where the chain's ramp cut takes
# over 1 mm further out. Angular span: the exit wedge less
# EXIT_FLOOR_SIDE_CLEAR (at the inner radius) from each face it must
# not touch — the wall's cut face on the upstream side, and on the
# downstream side the paddle's face, which stands *inside* the wedge
# EXIT_WALL_T in from the cut (the extrude goes −Y off Plane.XZ, so
# the paddle plugs the opening's edge rather than the wall's solid
# half), so that edge is a further EXIT_WALL_T in. The rib stands from
# the plate top to the tongue's underside, inboard of the skirt, 2°
# inside the tongue's edges.
EXIT_FLOOR_T = 4.0
EXIT_FLOOR_GEAR_CLEAR = 2.0
EXIT_FLOOR_R_IN = RING_PITCH_D / 2.0 + ADDENDUM + EXIT_FLOOR_GEAR_CLEAR   # 123.5
EXIT_FLOOR_R_OUT = R_STATOR_OUTER_KINK                                    # 155
EXIT_FLOOR_SIDE_CLEAR = 1.5
_UPSTREAM_EDGE_DEG = EXIT_ARC_DEG / 2.0 - math.degrees(EXIT_FLOOR_SIDE_CLEAR / EXIT_FLOOR_R_IN)          # ≈ 17.3
_DOWNSTREAM_EDGE_DEG = EXIT_ARC_DEG / 2.0 - math.degrees((EXIT_WALL_T + EXIT_FLOOR_SIDE_CLEAR) / EXIT_FLOOR_R_IN)   # ≈ 15.9
# (from, to) in degrees about +X, the paddle side per EXIT_WALL_SIDE_SIGN.
EXIT_FLOOR_SPAN_DEG = (
    (-_UPSTREAM_EDGE_DEG, _DOWNSTREAM_EDGE_DEG) if EXIT_WALL_SIDE_SIGN > 0
    else (-_DOWNSTREAM_EDGE_DEG, _UPSTREAM_EDGE_DEG)
)
EXIT_FLOOR_RIB_R_IN = 145.0
EXIT_FLOOR_RIB_R_OUT = 153.0
EXIT_FLOOR_RIB_INSET_DEG = 2.0


# --- Part functions ----------------------------------------------------

def stator(
    *,
    r_in_base: float = R_STATOR_INNER_BASE,
    r_in_kink: float = R_STATOR_INNER_KINK,
    r_out_base: float = R_STATOR_OUTER_BASE,
    r_out_kink: float = R_STATOR_OUTER_KINK,
    v_h: float = STATOR_V_H,
    height: float = STATOR_WALL_H,
    exit_arc_deg: float = EXIT_ARC_DEG,
    exit_wall_t: float = EXIT_WALL_T,
    exit_wall_rotor_gap: float = EXIT_WALL_ROTOR_GAP,
    exit_wall_side_sign: int = EXIT_WALL_SIDE_SIGN,
) -> Part:
    """Two-zone ring wall around +Z with an exit-arc wedge cut through,
    plus a radial one-way paddle at the downstream edge of that cut.

    Cross-section in the XZ plane is a 6-vertex polygon (reduces to
    4 vertices when v_h == height). Inner and outer faces slope at
    V_HALF_ANGLE from vertical up to z=v_h, then go vertical up to
    z=height. Revolved around Z to form the full wall. A radial
    wedge centered on +X (spanning ±exit_arc_deg/2) is then
    subtracted to produce the exit opening. Finally a trapezoidal
    paddle is unioned on at the downstream edge of the cut (sign
    set by exit_wall_side_sign) — its inner edge slants inward with
    z to track the rotor's nominal conical flank (offset by
    exit_wall_rotor_gap), its outer edge sits flush with the
    cylindrical outer wall.
    """
    # Guards the argument, not just the module default: the profile
    # below degenerates if the kink sits at or above the wall top.
    assert 0.0 < v_h <= height, (
        f"v_h ({v_h}) must be positive and not exceed height ({height})"
    )

    profile_pts: list[tuple[float, float]] = [
        (r_in_base, 0.0),
        (r_out_base, 0.0),
    ]
    if v_h < height:
        profile_pts += [
            (r_out_kink, v_h),
            (r_out_kink, height),
            (r_in_kink, height),
            (r_in_kink, v_h),
        ]
    else:
        # Pure-V variant: skip the cylindrical zone entirely.
        profile_pts += [
            (r_out_kink, height),
            (r_in_kink, height),
        ]
    profile = Plane.XZ * Polygon(*profile_pts, align=None)
    wall = revolve(profile, axis=Axis.Z)

    # Exit-arc cutter: an isoceles triangle in XY with apex at the
    # axis and outer chord past the wall's outer radius. `r_cut` is
    # sized so the chord's perpendicular distance from origin
    # (= r_cut·cos(h)) exceeds r_out_kink (= outermost radius) with
    # 10 mm of margin; this scales cleanly with any exit_arc_deg
    # < ~170°.
    h = math.radians(exit_arc_deg / 2.0)
    r_cut = (r_out_kink + 10.0) / math.cos(h)
    wedge_sketch = Polygon(
        (0.0, 0.0),
        (r_cut * math.cos(-h), r_cut * math.sin(-h)),
        (r_cut * math.cos(+h), r_cut * math.sin(+h)),
        align=None,
    )
    z_pad = 1.0
    wedge = Pos(0, 0, -z_pad) * extrude(
        wedge_sketch, amount=height + 2 * z_pad,
    )

    # Downstream paddle — profile in the XZ plane traces the stator's
    # own outer envelope on the outside (sloped V-face below v_h, then
    # vertical cylinder above) and the rotor's cone + gap on the
    # inside. Extruded into +Y (or −Y for CW sign) so the y=0 face
    # becomes the paddle's cut-boundary face after the Z-rotation;
    # the rest of the paddle lives in the solid half of the stator,
    # fused into a single printable part.
    r_rotor_base = ROTOR_BASE_DIAMETER / 2.0
    r_wall_inner_bot = r_rotor_base + exit_wall_rotor_gap
    r_wall_inner_top = (r_rotor_base - height * _TAN_V) + exit_wall_rotor_gap
    paddle_pts: list[tuple[float, float]] = [
        (r_wall_inner_bot, 0.0),
        (r_out_base, 0.0),
    ]
    if v_h < height:
        paddle_pts += [
            (r_out_kink, v_h),
            (r_out_kink, height),
        ]
    else:
        paddle_pts.append((r_out_kink, height))
    paddle_pts.append((r_wall_inner_top, height))
    paddle_profile = Plane.XZ * Polygon(*paddle_pts, align=None)
    paddle = extrude(paddle_profile, amount=exit_wall_t)
    if exit_wall_side_sign < 0:
        paddle = Pos(0, -exit_wall_t, 0) * paddle
    downstream_wall = Rot(0, 0, exit_wall_side_sign * exit_arc_deg / 2.0) * paddle

    return (wall - wedge) + downstream_wall


def _sector(r_in: float, r_out: float, span_deg: tuple[float, float], z0: float, h: float) -> Part:
    """Annular sector from ``span_deg[0]`` to ``span_deg[1]`` about +Z
    (degrees from +X), z from z0 to z0 + h. Built with ``align=None`` —
    a CENTER-aligned arc cylinder centres its bounding box, which moves
    the apex off the axis — and placed before the boolean, because
    ``Location * Compound`` yields a list."""
    lo, hi = span_deg
    outer = Rot(0, 0, lo) * Pos(0, 0, z0) * Cylinder(r_out, h, arc_size=hi - lo, align=None)
    inner = Pos(0, 0, z0 - 1.0) * Cylinder(r_in, h + 2.0, align=None)
    return outer - inner


def exit_floor(
    *,
    r_in: float = EXIT_FLOOR_R_IN,
    r_out: float = EXIT_FLOOR_R_OUT,
    t: float = EXIT_FLOOR_T,
    span_deg: tuple[float, float] = EXIT_FLOOR_SPAN_DEG,
    rib_r_in: float = EXIT_FLOOR_RIB_R_IN,
    rib_r_out: float = EXIT_FLOOR_RIB_R_OUT,
    rib_inset_deg: float = EXIT_FLOOR_RIB_INSET_DEG,
    z_bottom: float = PLATE_TOP_Z,
) -> Part:
    """The exit-floor tongue: a flat sector with its top face at z = 0
    spanning the exit wedge (asymmetric — see EXIT_FLOOR_SPAN_DEG), plus
    an arc rib under its outer edge from `z_bottom` (the base plate
    top) up to the tongue. About +X like the exit arc."""
    assert rib_r_out <= r_out and rib_r_in >= r_in, "rib must sit under the tongue"
    lo, hi = span_deg
    assert lo < hi and 2.0 * rib_inset_deg < hi - lo, "rib must sit inside the tongue's span"
    tongue = _sector(r_in, r_out, span_deg, -t, t)
    rib_h = -t - z_bottom
    assert rib_h > 0.0, "the plate top must be below the tongue"
    rib = _sector(rib_r_in, rib_r_out, (lo + rib_inset_deg, hi - rib_inset_deg), z_bottom, rib_h)
    return tongue + rib


# --- Assembly ----------------------------------------------------------

def build_stator() -> Assembly:
    """Stator sub-assembly in stator-local frame (= m02-local: origin
    on the rotor axis at the V-bottom): the wall and the exit floor.
    The tongue clears the wall's cut faces and the paddle laterally;
    its rests and its gear clearance are the unit root's (they involve
    the drive and the rotor)."""
    return (
        Assembly()
        .with_part("stator", stator(), material=material_of("stator"))
        .with_part("exit_floor", exit_floor(), material=material_of("exit_floor"))
        .assert_no_interference("exit_floor", "stator")
        .assert_clearance("exit_floor", "stator", min_mm=EXIT_FLOOR_SIDE_CLEAR - 0.01)
    )


assembly = build_stator()
