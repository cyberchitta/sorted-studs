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

A two-zone ring of thickness STATOR_WALL_T — V-slope from z=0 to
z=STATOR_V_H, then vertical cylinder from STATOR_V_H to
STATOR_WALL_H. The exit arc is cut through the full wall height —
except on the hopper stage, whose wall rises to HOPPER_WALL_H over an
opening that stays STATOR_WALL_H tall (``..params``). The
mounting interface (`_notes/decisions/m02-chain.md` §5, 2026-09-16):
the wall is a liner that sits on the drive's bowl — its outer cone
face stops at the skirt's inner radius at SKIRT_TOP_Z and steps out
to a flat flange (r SKIRT_R_IN…R_STATOR_OUTER_KINK) resting on the
skirt's top edge; the cone seats on that edge's inner corner, which
centres it. Above the flange the cylinder zone is the wall's own.
No floor at z=0: the pinion runs under the wall at r 120–147, over
the bowl's floor 18.5 mm down.

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

Thirds (`../seams.py`, 2026-09-17): the wall prints as three sectors,
``stator_0..2``, split at LINER_SEAM_PHASE_DEG — clear of the exit arc,
which is all the liner has to miss, so every stage's liner is the same
three parts. Each sector carries a bolted ear at both seams on the
cylinder zone's outer face, z SKIRT_TOP_Z…height, above the bowl's
skirt; the bowl's own seams are staggered from these (asserted in
``../assembly.py``).

Print orientation: inverted (top face on the bed), so the cylinder
zone rises from the bed, the flange's underside becomes an upward
step, and the V-cone hangs inward at 45° with no support — the
brick-facing inner cone is the overhung face. `printability.py`
inspects it that way (`up_axis=(0, 0, -1)`).

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
from ..seams import SEAM_COUNT, SEAM_EAR_T, angular_gap_deg, seam_azimuths, sector_index, split_ring
from ..params import (
    ADDENDUM,
    EXIT_ARC_DEG,
    EXIT_FLOOR_SIDE_CLEAR,
    EXIT_FLOOR_T,
    LINER_SEAM_PHASE_DEG,
    PLATE_TOP_Z,
    R_STATOR_INNER_BASE,
    R_STATOR_INNER_KINK,
    R_STATOR_OUTER_BASE,
    R_STATOR_OUTER_KINK,
    RING_PITCH_D,
    ROTOR_BASE_DIAMETER,
    SKIRT_R_IN,
    SKIRT_TOP_Z,
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
# EXIT_FLOOR_SIDE_CLEAR (perpendicular, from the inner corner) from
# each face it must not touch — the wall's cut face on the upstream
# side, and on the downstream side the paddle's face, which stands
# *inside* the wedge EXIT_WALL_T in from the cut (the extrude goes −Y
# off Plane.XZ, so the paddle plugs the opening's edge rather than the
# wall's solid half), so that edge is a further EXIT_WALL_T in. Both
# faces are planes, so the angle is an asin: the arc-length `d / r`
# left the paddle side 0.001 mm short. The rib stands from
# the bowl's floor to the tongue's underside, 1 mm inboard of the
# skirt's inner face (EXIT_FLOOR_RIB_SKIRT_CLEAR), 2° inside the
# tongue's edges. EXIT_FLOOR_T and EXIT_FLOOR_SIDE_CLEAR live in
# ..params: the skirt's wedge slot is cut from them.
EXIT_FLOOR_GEAR_CLEAR = 2.0
EXIT_FLOOR_R_IN = RING_PITCH_D / 2.0 + ADDENDUM + EXIT_FLOOR_GEAR_CLEAR   # 123.5
EXIT_FLOOR_R_OUT = R_STATOR_OUTER_KINK                                    # 155
_UPSTREAM_EDGE_DEG = EXIT_ARC_DEG / 2.0 - math.degrees(math.asin(EXIT_FLOOR_SIDE_CLEAR / EXIT_FLOOR_R_IN))          # ≈ 17.3
_DOWNSTREAM_EDGE_DEG = EXIT_ARC_DEG / 2.0 - math.degrees(math.asin((EXIT_WALL_T + EXIT_FLOOR_SIDE_CLEAR) / EXIT_FLOOR_R_IN))   # ≈ 15.9
# (from, to) in degrees about +X, the paddle side per EXIT_WALL_SIDE_SIGN.
EXIT_FLOOR_SPAN_DEG = (
    (-_UPSTREAM_EDGE_DEG, _DOWNSTREAM_EDGE_DEG) if EXIT_WALL_SIDE_SIGN > 0
    else (-_DOWNSTREAM_EDGE_DEG, _UPSTREAM_EDGE_DEG)
)
EXIT_FLOOR_RIB_SKIRT_CLEAR = 1.0
EXIT_FLOOR_RIB_R_OUT = SKIRT_R_IN - EXIT_FLOOR_RIB_SKIRT_CLEAR             # 151
EXIT_FLOOR_RIB_R_IN = EXIT_FLOOR_RIB_R_OUT - 8.0                          # 143
EXIT_FLOOR_RIB_INSET_DEG = 2.0

# The flange the wall rests on: its inner corner is where the outer
# cone face reaches the skirt's inner radius, at SKIRT_TOP_Z.
_FLANGE_R_IN = SKIRT_R_IN
_FLANGE_Z = SKIRT_TOP_Z


# --- Part functions ----------------------------------------------------

def stator(
    *,
    r_in_base: float = R_STATOR_INNER_BASE,
    r_in_kink: float = R_STATOR_INNER_KINK,
    r_out_base: float = R_STATOR_OUTER_BASE,
    r_out_kink: float = R_STATOR_OUTER_KINK,
    v_h: float = STATOR_V_H,
    height: float = STATOR_WALL_H,
    exit_h: float = STATOR_WALL_H,
    exit_arc_deg: float = EXIT_ARC_DEG,
    exit_wall_t: float = EXIT_WALL_T,
    exit_wall_rotor_gap: float = EXIT_WALL_ROTOR_GAP,
    exit_wall_side_sign: int = EXIT_WALL_SIDE_SIGN,
) -> Part:
    """Two-zone ring wall around +Z with an exit-arc wedge cut through,
    plus a radial one-way paddle at the downstream edge of that cut.

    Cross-section in the XZ plane is a 7-vertex polygon (5 when
    v_h == height). Inner and outer faces slope at V_HALF_ANGLE from
    vertical; the outer face stops at the skirt's inner radius
    (z = SKIRT_TOP_Z) and steps out to the flange that rests on the
    bowl's skirt, then goes vertical to z=height; the inner face
    kinks to vertical at z=v_h. Revolved around Z. A radial
    wedge centered on +X (spanning ±exit_arc_deg/2) is then
    subtracted to produce the exit opening, from the V-bottom up to
    z=exit_h — below ``height`` on the hopper stage, whose wall closes
    over the opening above that. Finally a trapezoidal
    paddle is unioned on at the downstream edge of the cut (sign
    set by exit_wall_side_sign) — its inner edge slants inward with
    z to track the rotor's nominal conical flank (offset by
    exit_wall_rotor_gap), its outer edge sits flush with the
    cylindrical outer wall. Under a lintel it continues up as a 45°
    gusset fin to the wall's inner face.
    """
    # Guards the argument, not just the module default: the profile
    # below degenerates if the kink sits at or above the wall top.
    assert 0.0 < v_h <= height, (
        f"v_h ({v_h}) must be positive and not exceed height ({height})"
    )
    # The paddle's inner edge tracks the rotor cone up to exit_h, so the
    # opening can't rise past the rotor top (the edge's radius would go
    # negative) — the hopper wall is why exit_h and height are separate.
    assert v_h <= exit_h <= min(height, ROTOR_BASE_DIAMETER / 2.0 / _TAN_V), (
        f"exit_h ({exit_h}) must lie between v_h ({v_h}) and the wall top ({height})"
    )
    # The flange sits below the kink and inside the outer kink radius:
    # the outer cone reaches r_out_base + SKIRT_TOP_Z·tan V there.
    assert 0.0 < _FLANGE_Z < v_h and _FLANGE_R_IN < r_out_kink, (
        f"flange at z {_FLANGE_Z} / r {_FLANGE_R_IN} must lie under the "
        f"kink (z {v_h}, r {r_out_kink})"
    )

    profile_pts: list[tuple[float, float]] = [
        (r_in_base, 0.0),
        (r_out_base, 0.0),
        (_FLANGE_R_IN, _FLANGE_Z),
        (r_out_kink, _FLANGE_Z),
    ]
    if v_h < height:
        profile_pts += [
            (r_out_kink, height),
            (r_in_kink, height),
            (r_in_kink, v_h),
        ]
    else:
        # Pure-V variant: no cylindrical zone above the kink.
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
    # Through the wall top when the opening is full height; stopping at
    # exit_h exactly otherwise, so the wall above is a lintel.
    wedge = Pos(0, 0, -z_pad) * extrude(
        wedge_sketch, amount=exit_h + z_pad + (z_pad if exit_h >= height else 0.0),
    )

    # Downstream paddle — profile in the XZ plane traces the stator's
    # own outer envelope on the outside (sloped V-face below v_h, then
    # vertical cylinder above) and the rotor's cone + gap on the
    # inside. `extrude` off Plane.XZ goes −Y, so for the CCW sign the
    # paddle body lies at y ∈ [−t, 0] and, after the Z-rotation to the
    # +arc/2 edge, stands *inside* the exit wedge with its far face on
    # the cut plane (the CW sign mirrors it). It plugs the opening's
    # downstream edge, one wall thickness in — the exit floor's span
    # (EXIT_FLOOR_SPAN_DEG) stops short of that face. (Until 2026-09-15
    # this comment said the paddle lived in the wall's solid half; the
    # solid says otherwise, probed at r 124 — inside from 16.6° to 18°.)
    r_rotor_base = ROTOR_BASE_DIAMETER / 2.0
    r_wall_inner_bot = r_rotor_base + exit_wall_rotor_gap
    r_wall_inner_top = (r_rotor_base - exit_h * _TAN_V) + exit_wall_rotor_gap
    # Under a lintel (exit_h < height) the paddle's top edge would start
    # mid-air in the inverted print — a 90° ledge anchored only at the
    # wall. So it rises on as a gusset fin into the hopper, its inner edge
    # climbing outward at 45° from the paddle's inner top corner to the
    # wall's inner face (owner, 2026-09-18): self-supporting, and the gap
    # to the rotor cone only widens above exit_h.
    top_z = exit_h
    if exit_h < height:
        top_z = exit_h + (r_in_kink - r_wall_inner_top) / _TAN_V
        assert top_z <= height, f"gusset top ({top_z:.1f}) above the wall top ({height})"
    # A flat plate's far corner sits at √(r² + t²): pull the outer edge
    # in so it stays inside the wall's curved outer face instead of
    # poking 0.03 mm past it (a 90° sliver ledge under a lintel).
    r_paddle_out = math.sqrt(r_out_kink ** 2 - exit_wall_t ** 2)
    paddle_pts: list[tuple[float, float]] = [
        (r_wall_inner_bot, 0.0),
        (r_out_base, 0.0),
        (_FLANGE_R_IN, _FLANGE_Z),
        (r_paddle_out, _FLANGE_Z),
        (r_paddle_out, exit_h),
    ]
    if top_z > exit_h:
        # Above exit_h the wall itself fills r_in_kink…r_out_kink, so the
        # fin stops at its inner face: a flat plate carried out to the
        # curved outer face would poke a sliver past it.
        paddle_pts += [(r_in_kink, exit_h), (r_in_kink, top_z)]
    paddle_pts.append((r_wall_inner_top, exit_h))
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
    an arc rib under its outer edge from `z_bottom` (the bowl's floor
    top) up to the tongue. About +X like the exit arc. The tongue
    passes through the skirt's wedge slot; the rib stands inboard of
    the skirt's inner face (measured at the unit root only as a
    no-interference, since the rib's rest on the floor makes the pair
    a contact pair — so the margin is guarded here)."""
    assert rib_r_out <= r_out and rib_r_in >= r_in, "rib must sit under the tongue"
    assert rib_r_out + EXIT_FLOOR_RIB_SKIRT_CLEAR <= SKIRT_R_IN + 1e-9, (
        f"rib outer radius {rib_r_out} runs into the skirt's inner face {SKIRT_R_IN}"
    )
    lo, hi = span_deg
    assert lo < hi and 2.0 * rib_inset_deg < hi - lo, "rib must sit inside the tongue's span"
    tongue = _sector(r_in, r_out, span_deg, -t, t)
    rib_h = -t - z_bottom
    assert rib_h > 0.0, "the plate top must be below the tongue"
    rib = _sector(rib_r_in, rib_r_out, (lo + rib_inset_deg, hi - rib_inset_deg), z_bottom, rib_h)
    return tongue + rib


def stator_sectors(*, phase_deg: float = LINER_SEAM_PHASE_DEG, height: float = STATOR_WALL_H) -> list[Part]:
    """The wall cut into SEAM_COUNT sectors at ``phase_deg``, ears on
    the cylinder zone above the skirt. Sector k is ``stator_k``. The
    exit opening stays STATOR_WALL_H tall whatever ``height`` is."""
    return split_ring(
        stator(height=height), phase_deg=phase_deg, r_out=R_STATOR_OUTER_KINK,
        ear_z=(SKIRT_TOP_Z, height),
    )


def stator_part(k: int) -> str:
    """Leaf name of liner sector k."""
    return f"stator_{k}"


STATOR_PARTS = tuple(stator_part(k) for k in range(SEAM_COUNT))


def sector_solid_count(k: int, height: float = STATOR_WALL_H) -> int:
    """How many pieces sector ``k`` is at wall ``height``. A seam may
    not run into the exit arc (asserted in ``build_stator``), so the
    opening always falls *inside* one sector — and on a plain wall it
    is full height, so it cuts that sector clean through: two arcs,
    each held by the ear at its own seam and free at the opening. On
    the hopper stage the wall continues above the opening and the
    sector is one piece.

    The single source for both the mechanism claim and the
    printability claim, so the two cannot drift: "three sectors"
    quietly means four printed parts at wall height (found
    2026-09-20, `_notes/state/m02-singulator.md`)."""
    split = k == sector_index(0.0, LINER_SEAM_PHASE_DEG) and height <= STATOR_WALL_H + 1e-9
    return 2 if split else 1


# --- Assembly ----------------------------------------------------------

def build_stator(height: float = STATOR_WALL_H) -> Assembly:
    """Stator sub-assembly in stator-local frame (= m02-local: origin
    on the rotor axis at the V-bottom): the wall, ``height`` tall
    (HOPPER_WALL_H on the hopper stage), and the exit floor.
    The tongue clears the wall's cut faces and the paddle laterally;
    its rests and its gear clearance are the unit root's (they involve
    the drive and the rotor)."""
    ear_deg = math.degrees(2.0 * SEAM_EAR_T / R_STATOR_OUTER_KINK)
    for seam in seam_azimuths(LINER_SEAM_PHASE_DEG):
        assert angular_gap_deg(seam, 0.0) >= EXIT_ARC_DEG / 2.0 + ear_deg, (
            f"liner seam at {seam:.1f}° runs into the exit arc"
        )
    a = Assembly()
    for k, sector in enumerate(stator_sectors(height=height)):
        a = a.with_part(stator_part(k), sector, material=material_of(stator_part(k)))
    a = a.with_part("exit_floor", exit_floor(), material=material_of("exit_floor"))
    for k, part in enumerate(STATOR_PARTS):
        a = (
            a
            .assert_no_interference("exit_floor", part)
            .assert_tangent_contact(part, STATOR_PARTS[(k + 1) % SEAM_COUNT])
        )
    # The tongue's side clearance is to the sector spanning the exit.
    exit_k = sector_index(0.0, LINER_SEAM_PHASE_DEG)
    exit_sector = stator_part(exit_k)
    # How many pieces each sector actually is — derived by
    # `sector_solid_count`, which `printability.py` claims from too.
    for k, part in enumerate(STATOR_PARTS):
        a = a.assert_solid_count(
            part, eq=sector_solid_count(k, height),
            detail="count derived by `sector_solid_count`: 2 on the "
                   "sector the exit opening cuts at wall height, 1 "
                   "elsewhere and on the hopper stage. A mismatch "
                   "means the wall height or the seam phase moved "
                   "the opening relative to the seams",
        )
    return a.assert_clearance(
        "exit_floor", exit_sector, min_mm=EXIT_FLOOR_SIDE_CLEAR,
    )


assembly = build_stator()
