"""
M03 scanner — rev-3 rail A/B T-junction gusset.

Printed top-side plate that stiffens the rail A ↔ rail B junction in
the sparse scanner T-frame. Rail A runs continuously through the
junction and rail B butts to rail A's back face with a CORNER_GAP
separator; this plate bridges the two rails by bolting into rail A's
and rail B's top T-slots, so the junction carries bending without
relying on commodity 2020 corner brackets.

The gusset sits on the rails' +Z face instead of below them. The rev-3
platform frame is coplanar with rail A/B (sharing a single bench), so
the platform-frame floor + tongue run at the same z = -220 as the
rails — a bottom-side gusset would collide with the floor / tongue.
The top face is clear at the A/B joint: mirror pad-clamps live near
x = ±207, far from this center gusset.

The Y-arm extends back from the rail A/B junction only. Rail A's
continuous body owns the front-face plane (the rev-3 platform tongue
butts that plane), so rail B starts behind rail A's back face and the
Y-arm covers rail A then rail B as it extends back — its front edge
sits at rail A's front face and the plate is asymmetric in Y about
the gusset's local origin.

Coordinate frame:
    origin : rail A/B junction on the rails' +Z face, at rail A's
             centerline Y
    +X     : along rail A
    +Y     : along rail B, toward the back of the rig
    +Z     : away from the gripped rail face (up in scanner world)
"""

import sys
from pathlib import Path

from build123d import Box, Cylinder, Location, Part, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from camera_rig.params import (  # noqa: E402
    RAIL_PAD_CLAMP_BELOW_LEFT_POS,
    SKELETON_RAIL_MEMBER,
    T_FRAME_RAIL_A_TOP_Z,
    WALL_MIN,
)


# --- Parameters ---------------------------------------------------------

MEMBER = SKELETON_RAIL_MEMBER
RAIL_A_Y = RAIL_PAD_CLAMP_BELOW_LEFT_POS[1]
RAIL_A_Z = T_FRAME_RAIL_A_TOP_Z - MEMBER / 2

PLATE_T = 6.0
ARM_W = MEMBER
ARM_X_L = 150.0
ARM_Y_L = 130.0

# Y-arm front edge sits at the rail A/B front face. With no rail B
# front outrigger, the arm extends back-only; in local coordinates
# the arm spans y ∈ [_ARM_Y_FRONT, _ARM_Y_FRONT + ARM_Y_L].
_ARM_Y_FRONT = -MEMBER / 2                          # = -10
_ARM_Y_CENTER = _ARM_Y_FRONT + ARM_Y_L / 2          # = 55

BOLT_HOLE_D = 5.5
BOLT_X_HALF = 42.0
# Rail-B bolts symmetric about the (back-shifted) y-arm centerline.
_BOLT_Y_HALF = 38.0
BOLT_Y_OFFSETS = (
    _ARM_Y_CENTER - _BOLT_Y_HALF,                   # ≈ 17 (world y ≈ 45.4)
    _ARM_Y_CENTER + _BOLT_Y_HALF,                   # ≈ 93 (world y ≈ 121.4)
)

_PLATE_Z = T_FRAME_RAIL_A_TOP_Z + PLATE_T / 2
# Rail A stub: continuous through the junction, centered at x = 0.
_RAIL_A_STUB_LEN = 180.0
# Rail B stub: butted to rail A's back face with CORNER_GAP separator.
_RAIL_B_STUB_LEN = 150.0
_RAIL_B_Y_FRONT = RAIL_A_Y + MEMBER / 2 + 0.3
_RAIL_B_YC = _RAIL_B_Y_FRONT + _RAIL_B_STUB_LEN / 2

GUSSET_LOCATION = Pos(0, RAIL_A_Y, T_FRAME_RAIL_A_TOP_Z)


# --- Parametric sanity --------------------------------------------------

assert ARM_X_L / 2 > BOLT_X_HALF + BOLT_HOLE_D / 2 + WALL_MIN, (
    "rail-A bolt holes need edge wall inside the X arm"
)
_ARM_Y_BACK = _ARM_Y_FRONT + ARM_Y_L
assert min(BOLT_Y_OFFSETS) > _ARM_Y_FRONT + BOLT_HOLE_D / 2 + WALL_MIN, (
    "rail-B front bolt needs edge wall inside the Y arm"
)
assert max(BOLT_Y_OFFSETS) < _ARM_Y_BACK - BOLT_HOLE_D / 2 - WALL_MIN, (
    "rail-B back bolt needs edge wall inside the Y arm"
)
assert ARM_W - BOLT_HOLE_D >= 2 * WALL_MIN, (
    f"bolt-hole-to-arm-edge wall {(ARM_W - BOLT_HOLE_D) / 2:.2f} mm < {WALL_MIN} mm"
)


# --- Part functions -----------------------------------------------------

def _through_hole_z(hole_d: float = BOLT_HOLE_D) -> Part:
    """Clearance hole through the plate into an extrusion top T-slot."""
    return Cylinder(hole_d / 2, PLATE_T * 3)


def gusset() -> Part:
    """Single printed T plate in local coordinates."""
    x_arm = Pos(0, 0, PLATE_T / 2) * Box(ARM_X_L, ARM_W, PLATE_T)
    y_arm = Pos(0, _ARM_Y_CENTER, PLATE_T / 2) * Box(ARM_W, ARM_Y_L, PLATE_T)
    body = x_arm + y_arm

    for sx in (-1, 1):
        body -= Pos(sx * BOLT_X_HALF, 0, PLATE_T / 2) * _through_hole_z()
    for dy in BOLT_Y_OFFSETS:
        body -= Pos(0, dy, PLATE_T / 2) * _through_hole_z()

    return body


def rail_a_stub() -> Part:
    return Pos(0.0, RAIL_A_Y, RAIL_A_Z) * Box(
        _RAIL_A_STUB_LEN, MEMBER, MEMBER
    )


def rail_b_stub() -> Part:
    return Pos(0, _RAIL_B_YC, RAIL_A_Z) * Box(MEMBER, _RAIL_B_STUB_LEN, MEMBER)


def add_frame_gusset_t(
    a: Assembly,
    name: str = "gusset_t_ab",
    location: Location = GUSSET_LOCATION,
) -> Assembly:
    """Add the A/B T-junction gusset to a scanner top-level assembly."""
    return a.add(name, gusset(), location=location, material="pla_matte")


# --- Assembly -----------------------------------------------------------

assembly = (
    Assembly()
    .add("gusset", gusset(), location=GUSSET_LOCATION, material="pla_matte")
    .add("rail_a", rail_a_stub(), material="aluminium_anodized")
    .add("rail_b", rail_b_stub(), material="aluminium_anodized")
    .assert_no_interference("gusset", "rail_a")
    .assert_no_interference("gusset", "rail_b")
    .assert_no_interference("rail_a", "rail_b")
)


if __name__ == "__main__":
    check(assembly, out="outputs")
    inspect(
        gusset(),
        method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=45.0),
        out="outputs",
        name="gusset",
    )
