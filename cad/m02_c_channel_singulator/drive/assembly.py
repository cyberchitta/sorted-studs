"""
M02 C-channel singulator — drive sub-assembly.

Hidden-drive architecture: NEMA 17 stepper sits INSIDE the rotor's
interior cavity, with its shaft coupled directly to the rotor's
top-cap boss. A fixed, non-rotating stem rises from the machine
frame through the rotor's open bottom to a mount plate inside the
cavity; the motor hangs from the plate by its four M3 face bolts
(front face up, body hanging below).

Two parts in this sub-assembly:

  - stem  — fixed vertical post + top flange (the mount plate,
            modeled as a wider disc on top of the shaft). Non-
            rotating; threads into the frame below z=0.
  - motor — NEMA 17 stub: square body + 5 mm round shaft. Body is
            fixed (bolted to the stem flange). Shaft rotates with
            the rotor — for static interference checks the rotation
            is immaterial, so both are combined into one Part.

The motor's front face (shaft-exit face) sits at MOUNT_Z; body
hangs below to MOTOR_BASE_Z (negative, below the rotor base plane);
shaft extends up to SHAFT_TIP_Z, landing inside the rotor's blind
bore with ~3 mm axial margin so the shaft never bottoms out.

Coordinate frame:
    origin = rotor axis ∩ rotor base plane (shared with rotor/)
    +Z     = up
    z=0    = rotor base / stator V-bottom. Stem passes through here.

Standalone validation: both parts placed at their installed
positions with a no-interference assertion between them — captures
drift in MOUNT_Z / NEMA17_BODY_LEN that would park the motor body
inside the mount flange. Rotor-vs-drive and stator-vs-drive checks
live in the top-level ../assembly.py.
"""


from build123d import Box, Cylinder, Part, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

from ..design_systems import color_of
from ..materials import material_of
from ..params import (
    MOTOR_BASE_Z,
    MOUNT_Z,
    NEMA17_BODY_LEN,
    NEMA17_SHAFT_D,
    NEMA17_SHAFT_LEN,
    NEMA17_SIDE,
    ROTOR_BOSS_OD,
    SHAFT_TIP_Z,
    STEM_ARM_W,
    STEM_BOTTOM_Z,
    STEM_FLANGE_DIA,
    STEM_FLANGE_T,
    STEM_OD,
    STEM_OFFSET_X,
)


# --- Part functions ----------------------------------------------------

def stem() -> Part:
    """Offset rod + cantilever arm + centered mount plate.

    - Rod: vertical cylinder at (STEM_OFFSET_X, 0), OD=STEM_OD,
      spans STEM_BOTTOM_Z → MOUNT_Z. Clears the NEMA-17 body (which
      occupies the axial column) and stays inside the cavity
      apothem at every z along its length.
    - Arm: narrow bar at z ∈ [MOUNT_Z, MOUNT_Z + STEM_FLANGE_T],
      spanning x ∈ [0, STEM_OFFSET_X], joining the rod top to the
      mount plate. Only material above z=MOUNT_Z (clears the motor
      body below).
    - Mount plate: disc centered on axis, OD=STEM_FLANGE_DIA × T,
      bottom face at MOUNT_Z. Motor front face bolts into the
      plate's bottom face (tangent contact).
    """
    shaft_len = MOUNT_Z - STEM_BOTTOM_Z
    rod = Pos(STEM_OFFSET_X, 0, STEM_BOTTOM_Z + shaft_len / 2.0) * Cylinder(
        STEM_OD / 2.0, shaft_len,
    )
    arm = Pos(
        STEM_OFFSET_X / 2.0, 0, MOUNT_Z + STEM_FLANGE_T / 2.0,
    ) * Box(STEM_OFFSET_X, STEM_ARM_W, STEM_FLANGE_T)
    plate = Pos(0, 0, MOUNT_Z + STEM_FLANGE_T / 2.0) * Cylinder(
        STEM_FLANGE_DIA / 2.0, STEM_FLANGE_T,
    )
    # Clearance hole through the plate + arm. Sized to clear the
    # rotor's descending boss (OD=ROTOR_BOSS_OD) plus 1 mm, not
    # just the shaft — the boss passes down into the plate level
    # at z=MOUNT_Z and must not touch the plate's inner rim.
    shaft_clear_r = ROTOR_BOSS_OD / 2.0 + 1.0
    shaft_hole = Pos(0, 0, MOUNT_Z + STEM_FLANGE_T / 2.0) * Cylinder(
        shaft_clear_r, STEM_FLANGE_T + 2.0,
    )
    return (rod + arm + plate) - shaft_hole


def motor() -> Part:
    """NEMA 17 stub: square body + 5 mm round shaft.

    Body: NEMA17_SIDE × NEMA17_SIDE × NEMA17_BODY_LEN, front face
    (shaft-exit) at MOUNT_Z with the body hanging down.
    Shaft: NEMA17_SHAFT_D round, pointing up from MOUNT_Z to
    SHAFT_TIP_Z.
    """
    body = Pos(0, 0, MOTOR_BASE_Z + NEMA17_BODY_LEN / 2.0) * Box(
        NEMA17_SIDE, NEMA17_SIDE, NEMA17_BODY_LEN,
    )
    shaft = Pos(0, 0, MOUNT_Z + NEMA17_SHAFT_LEN / 2.0) * Cylinder(
        NEMA17_SHAFT_D / 2.0, NEMA17_SHAFT_LEN,
    )
    return body + shaft


# --- Assembly ----------------------------------------------------------

def build_drive() -> Assembly:
    """Drive sub-assembly in drive-local frame (= m02-local). Parts
    only — the stem/motor contact assert is declared at whichever
    root runs ``check()`` (nested assertions are not evaluated)."""
    return (
        Assembly()
        .with_part(
            "stem", stem(),
            material=material_of("stem"), color=color_of("stem"),
        )
        .with_part(
            "motor", motor(),
            # `motor` is tagged None in the design system (NEMA-17 cast
            # body — outside palette); falls through to the material's
            # intrinsic color.
            material=material_of("motor"),
        )
    )


# Sanity: the motor's front face sits on the stem flange's top face.
# These two faces are coincident (tangent contact), not overlapping;
# assert_no_interference is the right check — tangent contact reads
# as zero clearance but zero overlap volume.
assembly = build_drive().assert_no_interference("stem", "motor")


if __name__ == "__main__":
    check(assembly, out="outputs")
    # stem and motor are bought/standard hardware — not printed, so
    # no inspect() calls here.
