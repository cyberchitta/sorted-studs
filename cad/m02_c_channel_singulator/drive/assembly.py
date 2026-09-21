"""
M02 C-channel singulator — drive sub-assembly (rim gear on a rim bearing).

The rotor is supported at its rim and driven at its rim
(`_notes/decisions/m02-rotor-drive.md`, 2026-09-11). Everything here
hangs below the V-bottom plane, under the rotor's ring-gear flange:

  - bowl_0..2  — printed, in thirds (`../seams.py`): a floor annulus, r ∈ [PLATE_R_IN, PLATE_R_OUT],
                 with a cylindrical skirt on its rim rising to
                 SKIRT_TOP_Z (`m02-chain.md` §5, 2026-09-16). The floor
                 is the stator foot ring extended inward: the turntable
                 bearing sits on its top face, the motor bolts to its
                 underside through a pilot hole + four M3 clearance
                 holes at the NEMA-17 pattern. The stator wall is a
                 liner resting on the skirt's top edge. The skirt is
                 slotted through the exit arc for the stator's tongue,
                 down to SKIRT_CUT_BOTTOM_Z; below that a stub of skirt
                 keeps the ring beam closed. Split at three radial
                 seams (phase per instance, clear of the exit arc and
                 the motor — asserted in ``build_drive``), each sector
                 carrying a bolted ear at both seams outside the skirt.
  - bearing    — bought aluminium turntable, modelled as one annulus
                 (BEARING_ID / OD / H). Upper ring bolts to the rotor
                 flange, lower ring to the plate; both bolt circles
                 are detail-layer.
  - pinion     — printed; pitch cylinder (no teeth, like the m05 base
                 floor) on a hub that reaches down to just above the
                 plate so a transverse set screw has shaft to bite.
                 Bored for the 5 mm motor shaft. Its own sub-assembly
                 with a spin joint about its axis.
  - motor      — NEMA 17 stub: body + pilot boss + shaft, front face
                 up against the plate's underside.

Coordinate frame:
    origin = rotor axis ∩ rotor base plane (shared with rotor/)
    +X     = exit-arc azimuth (the pinion sits at MOTOR_AZIMUTH_DEG
             = 180°, the -X side, away from the exit)
    +Z     = up
    z=0    = rotor base / stator V-bottom. Everything here is below.

Claims declared here (they propagate to the module root): bearing
and motor rest on the floor (tangent contact), the pinion hub clears
the floor, the pinion bore clears the shaft, and the pinion's tip
circle stays inside the skirt — recorded as a scalar, because the
measured distance pinion↔bowl is the 0.5 mm floor gap, not the
radial one. Rotor-vs-drive and stator-vs-drive claims live in
../assembly.py.

Printed parts: the three bowl sectors (floor down, no support), pinion — see
`printability.py`. The bearing and motor are bought.
"""

import math

from build123d import Axis, Box, Circle, Cylinder, Location, Part, Polygon, Pos, Rot, extrude

from cad_khana.mechanism.assembly import Assembly, RevoluteJoint

from ..materials import material_of
from ..seams import SEAM_COUNT, SEAM_EAR_T, angular_gap_deg, sector_index, seam_azimuths, split_ring
from ..params import (
    ADDENDUM,
    BEARING_BOTTOM_Z,
    BOWL_SEAM_PHASE_DEG,
    EXIT_ARC_DEG,
    BEARING_H,
    BEARING_ID,
    BEARING_OD,
    CENTER_DIST,
    GEAR_RATIO,
    MOTOR_AZIMUTH_DEG,
    MOTOR_BOTTOM_Z,
    MOTOR_FACE_Z,
    MOTOR_SHAFT_BORE_CLEARANCE,
    NEMA17_BODY_LEN,
    NEMA17_BOLT_CBORE_D,
    NEMA17_BOLT_CBORE_DEPTH,
    NEMA17_BOLT_HOLE_D,
    NEMA17_BOLT_PATTERN,
    NEMA17_PILOT_D,
    NEMA17_PILOT_H,
    NEMA17_SHAFT_D,
    NEMA17_SHAFT_LEN,
    NEMA17_SIDE,
    PINION_BOTTOM_Z,
    PINION_FACE_W,
    PINION_HUB_BOTTOM_Z,
    PINION_HUB_OD,
    PINION_HUB_PLATE_CLEAR,
    PINION_PITCH_D,
    PINION_TOP_Z,
    PLATE_BOTTOM_Z,
    PLATE_MOTOR_HOLE_D,
    PLATE_R_IN,
    PLATE_R_OUT,
    PLATE_T,
    PLATE_TOP_Z,
    SKIRT_CUT_BOTTOM_Z,
    SKIRT_R_IN,
    SKIRT_TOP_Z,
    STATOR_WALL_T,
)


WALL_MIN = 1.5

# Pinion / motor axis in drive-local XY.
_AZ = math.radians(MOTOR_AZIMUTH_DEG)
PINION_AXIS_XY = (CENTER_DIST * math.cos(_AZ), CENTER_DIST * math.sin(_AZ))

# Tip circle of the pinion — the outermost thing under the stator.
PINION_TIP_R = CENTER_DIST + PINION_PITCH_D / 2.0 + ADDENDUM   # 147.2


# --- Part functions ----------------------------------------------------

def bowl(
    *,
    r_in: float = PLATE_R_IN,
    r_out: float = PLATE_R_OUT,
    t: float = PLATE_T,
    skirt_r_in: float = SKIRT_R_IN,
    skirt_top_z: float = SKIRT_TOP_Z,
    skirt_cut_bottom_z: float = SKIRT_CUT_BOTTOM_Z,
    exit_arc_deg: float = EXIT_ARC_DEG,
    motor_xy: tuple[float, float] = PINION_AXIS_XY,
    motor_rot_deg: float = MOTOR_AZIMUTH_DEG,
    pilot_hole_d: float = PLATE_MOTOR_HOLE_D,
    bolt_pattern: float = NEMA17_BOLT_PATTERN,
    bolt_hole_d: float = NEMA17_BOLT_HOLE_D,
    bolt_cbore_d: float = NEMA17_BOLT_CBORE_D,
    bolt_cbore_depth: float = NEMA17_BOLT_CBORE_DEPTH,
) -> Part:
    """Floor annulus (top face at z=PLATE_TOP_Z) with the motor's pilot
    hole and M3 pattern cut through at ``motor_xy`` (pattern square
    aligned radially by ``motor_rot_deg``, heads counterbored flush
    from the top), and a skirt on its rim
    from the floor top to ``skirt_top_z``, slotted through the exit
    arc from ``skirt_cut_bottom_z`` up for the stator's tongue.

    Prints floor-down: the skirt is a plain vertical wall and the
    slot's faces are vertical, so nothing overhangs.
    """
    assert r_in < r_out and t > 0.0
    assert skirt_r_in < r_out and skirt_top_z > PLATE_TOP_Z, "skirt must stand on the floor's rim"
    assert PLATE_TOP_Z < skirt_cut_bottom_z < skirt_top_z, "the tongue slot must stay inside the skirt"
    assert r_out - skirt_r_in >= WALL_MIN, "skirt wall too thin"
    # Bolt holes must stay clear of the pilot hole and of each other.
    assert bolt_pattern / 2.0 - bolt_hole_d / 2.0 - pilot_hole_d / 2.0 >= WALL_MIN, (
        "motor bolt holes break into the pilot hole"
    )
    # The pattern's outboard corners (radially aligned square: corner
    # at (+half, ±half) from the axis) must keep the through-hole
    # inside the skirt's inner face and the counterbore inside the
    # floor's outer edge by a wall. The counterbore may nick the
    # skirt's foot (see params NEMA17_BOLT_CBORE_D), the hole may not.
    half = bolt_pattern / 2.0
    _corner_r = math.hypot(math.hypot(*motor_xy) + half, half)
    assert _corner_r + bolt_hole_d / 2.0 <= skirt_r_in, (
        f"motor bolt hole (edge r {_corner_r + bolt_hole_d / 2.0:.2f}) runs under the skirt (r {skirt_r_in})"
    )
    assert _corner_r + bolt_cbore_d / 2.0 + WALL_MIN <= r_out, (
        "motor bolt counterbore breaks out of the floor's rim"
    )
    assert 0.0 < bolt_cbore_depth <= t - WALL_MIN, "counterbore leaves no floor under the head"
    plate = Pos(0, 0, PLATE_BOTTOM_Z) * extrude(
        Circle(r_out) - Circle(r_in), amount=t,
    )
    skirt = Pos(0, 0, PLATE_TOP_Z) * extrude(
        Circle(r_out) - Circle(skirt_r_in), amount=skirt_top_z - PLATE_TOP_Z,
    )
    # Exit-arc slot through the skirt — the same isoceles cutter the
    # stator wall uses (duplicated, not imported: cad/CLAUDE.md
    # principle 2), from the slot bottom up past the skirt's top.
    h = math.radians(exit_arc_deg / 2.0)
    r_cut = (r_out + 10.0) / math.cos(h)
    wedge_sketch = Polygon(
        (0.0, 0.0),
        (r_cut * math.cos(-h), r_cut * math.sin(-h)),
        (r_cut * math.cos(+h), r_cut * math.sin(+h)),
        align=None,
    )
    wedge = Pos(0, 0, skirt_cut_bottom_z) * extrude(
        wedge_sketch, amount=skirt_top_z - skirt_cut_bottom_z + 1.0,
    )
    cut_h = t + 2.0
    holes = Cylinder(pilot_hole_d / 2.0, cut_h)
    for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        holes += Pos(sx * half, sy * half, 0) * Cylinder(bolt_hole_d / 2.0, cut_h)
        # Counterbore from the floor's top face down, so the head sits
        # flush; the cutter extends 1 mm above the top face.
        holes += Pos(sx * half, sy * half, t / 2.0 - bolt_cbore_depth / 2.0 + 0.5) * Cylinder(
            bolt_cbore_d / 2.0, bolt_cbore_depth + 1.0,
        )
    holes = (
        Pos(motor_xy[0], motor_xy[1], PLATE_BOTTOM_Z + t / 2.0)
        * Rot(0, 0, motor_rot_deg)
        * holes
    )
    # Holes before the skirt: the two outboard counterbores reach
    # ~0.3 mm under the skirt's foot, and the foot's first layer
    # bridging that rim is the design (printability waives it with
    # the measured area); a cutter through the standing skirt would
    # notch its face instead.
    return ((plate - holes) + skirt) - wedge


def bowl_sectors(
    *,
    phase_deg: float = BOWL_SEAM_PHASE_DEG,
    motor_azimuth_deg: float = MOTOR_AZIMUTH_DEG,
) -> list[Part]:
    """The bowl for a motor at ``motor_azimuth_deg``, cut into
    SEAM_COUNT sectors at ``phase_deg`` with ears the full height of
    floor and skirt. Sector k is ``bowl_k``."""
    az = math.radians(motor_azimuth_deg)
    whole = bowl(
        motor_xy=(CENTER_DIST * math.cos(az), CENTER_DIST * math.sin(az)),
        motor_rot_deg=motor_azimuth_deg,
    )
    return split_ring(
        whole, phase_deg=phase_deg, r_out=PLATE_R_OUT, ear_z=(PLATE_BOTTOM_Z, SKIRT_TOP_Z),
    )


def bowl_part(k: int) -> str:
    """Leaf name of bowl sector k."""
    return f"bowl_{k}"


BOWL_PARTS = tuple(bowl_part(k) for k in range(SEAM_COUNT))


def bowl_part_at(az_deg: float, phase_deg: float = BOWL_SEAM_PHASE_DEG) -> str:
    """Leaf name of the bowl sector under m02-local azimuth ``az_deg``."""
    return bowl_part(sector_index(az_deg, phase_deg))


def bearing(
    *, od: float = BEARING_OD, id_: float = BEARING_ID, h: float = BEARING_H,
) -> Part:
    """Turntable bearing stub: one annulus, bottom face at
    z=BEARING_BOTTOM_Z. Bought — not inspected."""
    assert 0.0 < id_ < od
    return Pos(0, 0, BEARING_BOTTOM_Z) * extrude(
        Circle(od / 2.0) - Circle(id_ / 2.0), amount=h,
    )


def pinion(
    *,
    pitch_d: float = PINION_PITCH_D,
    face_w: float = PINION_FACE_W,
    hub_od: float = PINION_HUB_OD,
    bore_d: float = NEMA17_SHAFT_D + 2.0 * MOTOR_SHAFT_BORE_CLEARANCE,
) -> Part:
    """Pitch cylinder z ∈ [PINION_BOTTOM_Z, PINION_TOP_Z] on a hub
    reaching down to PINION_HUB_BOTTOM_Z, bored through for the motor
    shaft. Built on its own axis (pinion-local Z), so the spin joint
    is the local Z axis."""
    assert bore_d < hub_od - 2.0 * WALL_MIN, "hub wall too thin around the bore"
    assert hub_od < pitch_d - 2.0 * ADDENDUM * 1.25, "hub wider than the root circle"
    gear = Pos(0, 0, PINION_BOTTOM_Z) * extrude(Circle(pitch_d / 2.0), amount=face_w)
    hub_h = PINION_BOTTOM_Z - PINION_HUB_BOTTOM_Z
    hub = Pos(0, 0, PINION_HUB_BOTTOM_Z) * extrude(Circle(hub_od / 2.0), amount=hub_h)
    total_h = PINION_TOP_Z - PINION_HUB_BOTTOM_Z
    bore = Pos(0, 0, PINION_HUB_BOTTOM_Z - 1.0) * extrude(
        Circle(bore_d / 2.0), amount=total_h + 2.0,
    )
    return (gear + hub) - bore


def motor() -> Part:
    """NEMA 17 stub on its own axis: square body hanging below
    MOTOR_FACE_Z, pilot boss and shaft rising from the face."""
    body = Pos(0, 0, MOTOR_BOTTOM_Z + NEMA17_BODY_LEN / 2.0) * Box(
        NEMA17_SIDE, NEMA17_SIDE, NEMA17_BODY_LEN,
    )
    pilot = Pos(0, 0, MOTOR_FACE_Z + NEMA17_PILOT_H / 2.0) * Cylinder(
        NEMA17_PILOT_D / 2.0, NEMA17_PILOT_H,
    )
    shaft = Pos(0, 0, MOTOR_FACE_Z + NEMA17_SHAFT_LEN / 2.0) * Cylinder(
        NEMA17_SHAFT_D / 2.0, NEMA17_SHAFT_LEN,
    )
    return body + pilot + shaft


# --- Assembly ----------------------------------------------------------

def build_pinion() -> Assembly:
    """The pinion in its own frame (origin on its axis at z=0). The
    parent places it at PINION_AXIS_XY and owns the spin joint."""
    return Assembly().with_part("pinion", pinion(), material=material_of("pinion"))


def pinion_axis_xy(motor_azimuth_deg: float = MOTOR_AZIMUTH_DEG) -> tuple[float, float]:
    """Pinion / motor axis in m02-local XY for a given motor azimuth."""
    az = math.radians(motor_azimuth_deg)
    return (CENTER_DIST * math.cos(az), CENTER_DIST * math.sin(az))


def build_drive(
    motor_azimuth_deg: float = MOTOR_AZIMUTH_DEG,
    seam_phase_deg: float = BOWL_SEAM_PHASE_DEG,
) -> Assembly:
    """Drive sub-assembly in drive-local frame (= m02-local).

    ``motor_azimuth_deg`` is where the motor / pinion sit around the
    rotor axis (default ``params.MOTOR_AZIMUTH_DEG``, away from the
    exit arc). It is a per-*instance* choice — the mesh is the same
    at every azimuth — that the m02 chain sets per stage to keep a
    motor clear of that stage's gantry member (S2's stem, S3's cross
    bar; ``cad/m02_chain/params.py`` §Gantry).

    ``seam_phase_deg`` is where the bowl's thirds seams fall — also
    per instance, since they must miss that stage's gantry bar ends
    (the chain's §Seams). Guarded here against the two things the
    unit itself owns: the exit arc (slot and tongue) and the motor.
    """
    ear_deg = math.degrees(2.0 * SEAM_EAR_T / PLATE_R_OUT)
    # The motor body's angular half-width about the rotor axis.
    motor_half_deg = math.degrees(math.atan2(NEMA17_SIDE / 2.0, CENTER_DIST - NEMA17_SIDE / 2.0))
    for seam in seam_azimuths(seam_phase_deg):
        assert angular_gap_deg(seam, 0.0) >= EXIT_ARC_DEG / 2.0 + ear_deg, (
            f"bowl seam at {seam:.1f}° runs into the exit arc's slot"
        )
        assert angular_gap_deg(seam, motor_azimuth_deg) >= motor_half_deg + ear_deg, (
            f"bowl seam at {seam:.1f}° runs through the motor at {motor_azimuth_deg:.1f}°"
        )
    axis_xy = pinion_axis_xy(motor_azimuth_deg)
    on_axis = Location((axis_xy[0], axis_xy[1], 0.0)) * Rot(
        0, 0, motor_azimuth_deg,
    )
    a = Assembly()
    for k, sector in enumerate(bowl_sectors(phase_deg=seam_phase_deg, motor_azimuth_deg=motor_azimuth_deg)):
        a = a.with_part(bowl_part(k), sector, material=material_of(bowl_part(k)))
    motor_sector = bowl_part_at(motor_azimuth_deg, seam_phase_deg)
    a = (
        a
        .with_part("bearing", bearing(), material=material_of("bearing"))
        .with_part("motor", motor(), location=on_axis, material=material_of("motor"))
        .with_subassembly(
            "pinion",
            build_pinion(),
            location=on_axis,
            joint=RevoluteJoint(
                axis=Axis((0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
                angle_deg=0.0,
                frame="local",
            ),
        )
        # Where the chain's rail meets us, and how far the motor hangs.
        .with_anchor("plate_top", Pos(0, 0, PLATE_TOP_Z))
        .with_anchor("plate_bottom", Pos(0, 0, PLATE_BOTTOM_Z))
        .with_anchor("skirt_top", Pos(0, 0, SKIRT_TOP_Z))
        .with_anchor("motor_bottom", Pos(axis_xy[0], axis_xy[1], MOTOR_BOTTOM_Z))
        # The motor face rests on its sector's floor underside (pilot
        # and shaft inside the hole); the bore clears the shaft.
        .assert_tangent_contact("motor", motor_sector)
        .assert_no_interference("pinion.pinion", "motor")
        .assert_distance(
            "pinion.pinion", "motor", min_mm=MOTOR_SHAFT_BORE_CLEARANCE,
        )
        # Pinion and motor stay well clear of the bearing.
        .assert_distance("pinion.pinion", "bearing", min_mm=15.0)
        .assert_distance("motor", "bearing", min_mm=PLATE_T)
        # The pinion's tip circle stays inside the skirt with 1 mm to
        # spare. A scalar, not a measured clearance: the pinion↔bowl
        # distance is the 0.5 mm hub-to-floor gap asserted below.
        .assert_scalar(
            "pinion_tip_inside_skirt",
            PINION_TIP_R,
            le=SKIRT_R_IN - 1.0,
            detail="pinion tip radius vs the skirt's inner face, minus 1 mm",
        )
        .assert_scalar("gear_ratio", GEAR_RATIO, detail="ring / pinion teeth")
    )
    for k, part in enumerate(BOWL_PARTS):
        a = (
            a
            # The bearing's lower ring spans every seam: it rests on
            # each sector (and, bolted, is what splices the floor).
            .assert_tangent_contact("bearing", part)
            # Pinion hub floats just above the floor.
            .assert_distance(
                "pinion.pinion", part, min_mm=PINION_HUB_PLATE_CLEAR,
            )
            # Each seam: the two sectors' faces and ears meet on the
            # seam plane.
            .assert_tangent_contact(part, BOWL_PARTS[(k + 1) % SEAM_COUNT])
        )
    return a


# Degenerate memoized master, so ``khana check <file>`` resolves.
assembly = build_drive()
