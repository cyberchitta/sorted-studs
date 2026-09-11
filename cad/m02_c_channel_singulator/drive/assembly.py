"""
M02 C-channel singulator — drive sub-assembly (rim gear on a rim bearing).

The rotor is supported at its rim and driven at its rim
(`_notes/decisions/m02-rotor-drive.md`, 2026-09-11). Everything here
hangs below the V-bottom plane, under the rotor's ring-gear flange:

  - base_plate — printed annulus, r ∈ [PLATE_R_IN, PLATE_R_OUT]. It is
                 the stator foot ring of `m02-chain.md` §5 extended
                 inward: the turntable bearing sits on its top face,
                 the motor bolts to its underside through a pilot hole
                 + four M3 clearance holes at the NEMA-17 pattern.
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
and motor rest on the plate (tangent contact), the pinion hub clears
the plate top, the pinion bore clears the shaft, and the pinion's tip
circle stays inside the skirt — the skirt is not a part yet, so that
one is a recorded scalar. Rotor-vs-drive and stator-vs-drive claims
live in ../assembly.py.

Printed parts: base_plate, pinion — see `printability.py`. The
bearing and motor are bought.
"""

import math

from build123d import Axis, Box, Circle, Cylinder, Location, Part, Pos, Rot, extrude

from cad_khana.mechanism.assembly import Assembly, RevoluteJoint

from ..materials import material_of
from ..params import (
    ADDENDUM,
    BEARING_BOTTOM_Z,
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
    R_STATOR_OUTER_KINK,
    STATOR_WALL_T,
)


WALL_MIN = 1.5

# Measurement tolerance for bounds constructed from the same constant
# as the geometry (cad/CLAUDE.md principle 6, corollary 2).
_MEAS_TOL = 0.01

# Pinion / motor axis in drive-local XY.
_AZ = math.radians(MOTOR_AZIMUTH_DEG)
PINION_AXIS_XY = (CENTER_DIST * math.cos(_AZ), CENTER_DIST * math.sin(_AZ))

# Tip circle of the pinion — the outermost thing under the stator.
PINION_TIP_R = CENTER_DIST + PINION_PITCH_D / 2.0 + ADDENDUM   # 147.2


# --- Part functions ----------------------------------------------------

def base_plate(
    *,
    r_in: float = PLATE_R_IN,
    r_out: float = PLATE_R_OUT,
    t: float = PLATE_T,
    motor_xy: tuple[float, float] = PINION_AXIS_XY,
    motor_rot_deg: float = MOTOR_AZIMUTH_DEG,
    pilot_hole_d: float = PLATE_MOTOR_HOLE_D,
    bolt_pattern: float = NEMA17_BOLT_PATTERN,
    bolt_hole_d: float = NEMA17_BOLT_HOLE_D,
) -> Part:
    """Flat annulus, top face at z=PLATE_TOP_Z, with the motor's pilot
    hole and M3 pattern cut through at ``motor_xy`` (pattern square
    aligned radially by ``motor_rot_deg``)."""
    assert r_in < r_out and t > 0.0
    # Bolt holes must stay clear of the pilot hole and of each other.
    assert bolt_pattern / 2.0 - bolt_hole_d / 2.0 - pilot_hole_d / 2.0 >= WALL_MIN, (
        "motor bolt holes break into the pilot hole"
    )
    plate = Pos(0, 0, PLATE_BOTTOM_Z) * extrude(
        Circle(r_out) - Circle(r_in), amount=t,
    )
    cut_h = t + 2.0
    holes = Cylinder(pilot_hole_d / 2.0, cut_h)
    half = bolt_pattern / 2.0
    for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        holes += Pos(sx * half, sy * half, 0) * Cylinder(bolt_hole_d / 2.0, cut_h)
    holes = (
        Pos(motor_xy[0], motor_xy[1], PLATE_BOTTOM_Z + t / 2.0)
        * Rot(0, 0, motor_rot_deg)
        * holes
    )
    return plate - holes


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


def build_drive() -> Assembly:
    """Drive sub-assembly in drive-local frame (= m02-local)."""
    on_axis = Location((PINION_AXIS_XY[0], PINION_AXIS_XY[1], 0.0)) * Rot(
        0, 0, MOTOR_AZIMUTH_DEG,
    )
    return (
        Assembly()
        .with_part("base_plate", base_plate(), material=material_of("base_plate"))
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
        .with_anchor("motor_bottom", Pos(PINION_AXIS_XY[0], PINION_AXIS_XY[1], MOTOR_BOTTOM_Z))
        # Rests: the bearing's lower ring on the plate top, the motor
        # face on the plate underside (pilot and shaft inside the hole).
        .assert_tangent_contact("bearing", "base_plate")
        .assert_tangent_contact("motor", "base_plate")
        # Pinion hub floats just above the plate; bore clears the shaft.
        .assert_clearance(
            "pinion.pinion", "base_plate", min_mm=PINION_HUB_PLATE_CLEAR - _MEAS_TOL,
        )
        .assert_no_interference("pinion.pinion", "motor")
        .assert_clearance(
            "pinion.pinion", "motor", min_mm=MOTOR_SHAFT_BORE_CLEARANCE - _MEAS_TOL,
        )
        # Pinion and motor stay well clear of the bearing.
        .assert_clearance("pinion.pinion", "bearing", min_mm=15.0)
        .assert_clearance("motor", "bearing", min_mm=PLATE_T - _MEAS_TOL)
        # The skirt (m02-chain.md §5) is not a part yet: its inner face
        # is R_STATOR_OUTER_KINK − STATOR_WALL_T, and the pinion's tip
        # circle must stay inside it with 1 mm to spare.
        .assert_scalar(
            "pinion_tip_inside_skirt",
            PINION_TIP_R,
            le=R_STATOR_OUTER_KINK - STATOR_WALL_T - 1.0,
            detail="pinion tip radius vs the skirt's inner face, minus 1 mm",
        )
        .assert_scalar("gear_ratio", GEAR_RATIO, detail="ring / pinion teeth")
    )


# Degenerate memoized master, so ``khana check <file>`` resolves.
assembly = build_drive()
