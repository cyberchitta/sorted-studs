"""
M03 scanner — rev-3 rail A/B T-junction gusset.

Printed top-side plate that stiffens the rail A ↔ rail B junction in
the sparse scanner T-frame. Rail A is split left/right around the
continuous rail B; this plate bridges the two rail-A halves and bolts
into rail B's top slot so the junction carries bending without relying
on commodity 2020 corner brackets.

The gusset sits on the rails' +Y face instead of below them. The rev-3
platform frame is coplanar with rail A/B (sharing a single bench), so
the platform-frame floor + tongue run at the same y = -220 as the
rails — a bottom-side gusset would collide with the floor / tongue.
The top face is clear at the A/B joint: mirror pad-clamps live near
x = ±207, far from this center gusset.

The Z-arm extends back from the rail A/B junction only — rail B has no
front outrigger past rail A (the rev-3 platform tongue precludes one),
so the arm's front edge sits at the rail A front face and the plate is
asymmetric in Z about the gusset's local origin.

Coordinate frame:
    origin : rail A/B junction on the rails' +Y face, at rail A's
             centerline Z
    +X     : along rail A
    +Y     : away from the gripped rail face (up in scanner world)
    +Z     : along rail B, toward the back of the rig
"""

import sys
from pathlib import Path

from build123d import Box, Cylinder, Location, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from camera_rig.params import (  # noqa: E402
    RAIL_PAD_CLAMP_BELOW_LEFT_POS,
    SKELETON_RAIL_MEMBER,
    T_FRAME_RAIL_A_TOP_Y,
    WALL_MIN,
)


# --- Parameters ---------------------------------------------------------

MEMBER = SKELETON_RAIL_MEMBER
RAIL_A_Z = RAIL_PAD_CLAMP_BELOW_LEFT_POS[2]
RAIL_A_Y = T_FRAME_RAIL_A_TOP_Y - MEMBER / 2

PLATE_T = 6.0
ARM_W = MEMBER
ARM_X_L = 150.0
ARM_Z_L = 130.0

# Z-arm front edge sits at the rail A/B front face. With no rail B
# front outrigger, the arm extends back-only; in local coordinates
# the arm spans z ∈ [_ARM_Z_FRONT, _ARM_Z_FRONT + ARM_Z_L].
_ARM_Z_FRONT = -MEMBER / 2                          # = -10
_ARM_Z_CENTER = _ARM_Z_FRONT + ARM_Z_L / 2          # = 55

BOLT_HOLE_D = 5.5
BOLT_X_HALF = 42.0
# Rail-B bolts symmetric about the (back-shifted) z-arm centerline.
_BOLT_Z_HALF = 38.0
BOLT_Z_OFFSETS = (
    _ARM_Z_CENTER - _BOLT_Z_HALF,                   # ≈ 17 (world z ≈ 45.4)
    _ARM_Z_CENTER + _BOLT_Z_HALF,                   # ≈ 93 (world z ≈ 121.4)
)

_PLATE_Y = T_FRAME_RAIL_A_TOP_Y + PLATE_T / 2
_RAIL_A_INNER_X = MEMBER / 2 + 0.3
_RAIL_A_STUB_OUTER_X = 90.0
_RAIL_A_LEFT_XC = (-_RAIL_A_STUB_OUTER_X + -_RAIL_A_INNER_X) / 2
_RAIL_A_RIGHT_XC = (_RAIL_A_INNER_X + _RAIL_A_STUB_OUTER_X) / 2
_RAIL_A_STUB_LEN = _RAIL_A_STUB_OUTER_X - _RAIL_A_INNER_X
# Rail B stub: starts flush with rail A's front face and extends back.
_RAIL_B_STUB_LEN = 150.0
_RAIL_B_ZC = (RAIL_A_Z - MEMBER / 2) + _RAIL_B_STUB_LEN / 2

GUSSET_LOCATION = Pos(0, T_FRAME_RAIL_A_TOP_Y, RAIL_A_Z)


# --- Parametric sanity --------------------------------------------------

assert ARM_X_L / 2 > BOLT_X_HALF + BOLT_HOLE_D / 2 + WALL_MIN, (
    "rail-A bolt holes need edge wall inside the X arm"
)
_ARM_Z_BACK = _ARM_Z_FRONT + ARM_Z_L
assert min(BOLT_Z_OFFSETS) > _ARM_Z_FRONT + BOLT_HOLE_D / 2 + WALL_MIN, (
    "rail-B front bolt needs edge wall inside the Z arm"
)
assert max(BOLT_Z_OFFSETS) < _ARM_Z_BACK - BOLT_HOLE_D / 2 - WALL_MIN, (
    "rail-B back bolt needs edge wall inside the Z arm"
)
assert ARM_W - BOLT_HOLE_D >= 2 * WALL_MIN, (
    f"bolt-hole-to-arm-edge wall {(ARM_W - BOLT_HOLE_D) / 2:.2f} mm < {WALL_MIN} mm"
)


# --- Part functions -----------------------------------------------------

def _through_hole_y(hole_d: float = BOLT_HOLE_D) -> Part:
    """Clearance hole through the plate into an extrusion top T-slot."""
    return Rot(90, 0, 0) * Cylinder(hole_d / 2, PLATE_T * 3)


def gusset() -> Part:
    """Single printed T plate in local coordinates."""
    x_arm = Pos(0, PLATE_T / 2, 0) * Box(ARM_X_L, PLATE_T, ARM_W)
    z_arm = Pos(0, PLATE_T / 2, _ARM_Z_CENTER) * Box(ARM_W, PLATE_T, ARM_Z_L)
    body = x_arm + z_arm

    for sx in (-1, 1):
        body -= Pos(sx * BOLT_X_HALF, PLATE_T / 2, 0) * _through_hole_y()
    for dz in BOLT_Z_OFFSETS:
        body -= Pos(0, PLATE_T / 2, dz) * _through_hole_y()

    return body


def rail_a_stub_left() -> Part:
    return Pos(_RAIL_A_LEFT_XC, RAIL_A_Y, RAIL_A_Z) * Box(
        _RAIL_A_STUB_LEN, MEMBER, MEMBER
    )


def rail_a_stub_right() -> Part:
    return Pos(_RAIL_A_RIGHT_XC, RAIL_A_Y, RAIL_A_Z) * Box(
        _RAIL_A_STUB_LEN, MEMBER, MEMBER
    )


def rail_b_stub() -> Part:
    return Pos(0, RAIL_A_Y, _RAIL_B_ZC) * Box(MEMBER, MEMBER, _RAIL_B_STUB_LEN)


def add_frame_gusset_t(
    a: Assembly,
    name: str = "gusset_t_ab",
    location: Location = GUSSET_LOCATION,
) -> Assembly:
    """Add the A/B T-junction gusset to a scanner top-level assembly."""
    return a.add(name, gusset(), location=location)


# --- Assembly -----------------------------------------------------------

assembly = (
    Assembly()
    .add("gusset", gusset(), location=GUSSET_LOCATION)
    .add("rail_a_left", rail_a_stub_left())
    .add("rail_a_right", rail_a_stub_right())
    .add("rail_b", rail_b_stub())
    .assert_no_interference("gusset", "rail_a_left")
    .assert_no_interference("gusset", "rail_a_right")
    .assert_no_interference("gusset", "rail_b")
    .assert_no_interference("rail_a_left", "rail_b")
    .assert_no_interference("rail_a_right", "rail_b")
    .assert_no_interference("rail_a_left", "rail_a_right")
)


if __name__ == "__main__":
    check(assembly, out="outputs")
    inspect(
        gusset(),
        method=FDM(up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=45.0),
        out="outputs",
        name="gusset",
    )
