"""
M03 scanner — rev-3 rail C/D L-junction gusset.

Printed side plate that stiffens the rail C ↔ rail D junction in the
sparse scanner T-frame. Rail D bolts into rail C's -Z face as a
cantilever; this gusset lands on the shared +X side face of both rails
so it reinforces the corner without occupying the back-upper mirror
clamp volume on rail C's -Z face or the camera clamp volume on rail
D's underside.

Coordinate frame:
    origin : rail C/D joint on the rails' shared +X side face, at
             rail D centerline Y and rail C centerline Z
    +X     : away from the gripped rail side face
    +Y     : up along rail C
    +Z     : toward the back of the rig; rail D runs along Z
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
    SKELETON_RAIL_MEMBER,
    T_FRAME_RAIL_C_FRONT_Z,
    T_FRAME_RAIL_D_BOTTOM_Y,
    T_FRAME_RAIL_D_Y,
    WALL_MIN,
)


# --- Parameters ---------------------------------------------------------

MEMBER = SKELETON_RAIL_MEMBER
RAIL_C_Z = T_FRAME_RAIL_C_FRONT_Z + MEMBER / 2
RAIL_D_Y = T_FRAME_RAIL_D_Y
RAIL_D_Z_BACK = T_FRAME_RAIL_C_FRONT_Z - 0.3

PLATE_T = 6.0
ARM_W = MEMBER
ARM_C_Y_L = 150.0
ARM_D_Z_L = 160.0

BOLT_HOLE_D = 5.5
C_BOLT_Y_OFFSETS = (-45.0, 45.0)
D_BOLT_Z_OFFSETS = (-115.0, -55.0)

_PLATE_X = MEMBER / 2 + PLATE_T / 2
_RAIL_C_YC = RAIL_D_Y
_RAIL_C_STUB_LEN = 180.0
_RAIL_D_ZC = RAIL_D_Z_BACK - ARM_D_Z_L / 2

GUSSET_LOCATION = Pos(MEMBER / 2, RAIL_D_Y, RAIL_C_Z)


# --- Parametric sanity --------------------------------------------------

assert ARM_W - BOLT_HOLE_D >= 2 * WALL_MIN, (
    f"bolt-hole-to-arm-edge wall {(ARM_W - BOLT_HOLE_D) / 2:.2f} mm < {WALL_MIN} mm"
)
assert ARM_C_Y_L / 2 > max(abs(y) for y in C_BOLT_Y_OFFSETS) + BOLT_HOLE_D / 2 + WALL_MIN, (
    "rail-C bolt holes need edge wall inside the vertical arm"
)
assert ARM_D_Z_L > abs(min(D_BOLT_Z_OFFSETS)) + BOLT_HOLE_D / 2 + WALL_MIN, (
    "rail-D bolt holes need edge wall inside the horizontal arm"
)


# --- Part functions -----------------------------------------------------

def _through_hole_x(hole_d: float = BOLT_HOLE_D) -> Part:
    """Clearance hole through the side plate into an extrusion side T-slot."""
    return Rot(0, 90, 0) * Cylinder(hole_d / 2, PLATE_T * 3)


def gusset() -> Part:
    """Single printed L plate in local coordinates."""
    c_arm = Pos(PLATE_T / 2, 0, 0) * Box(PLATE_T, ARM_C_Y_L, ARM_W)
    d_arm = Pos(PLATE_T / 2, 0, RAIL_D_Z_BACK - RAIL_C_Z - ARM_D_Z_L / 2) * Box(
        PLATE_T, ARM_W, ARM_D_Z_L
    )
    body = c_arm + d_arm

    for dy in C_BOLT_Y_OFFSETS:
        body -= Pos(PLATE_T / 2, dy, 0) * _through_hole_x()
    for dz in D_BOLT_Z_OFFSETS:
        body -= (
            Pos(PLATE_T / 2, 0, RAIL_D_Z_BACK - RAIL_C_Z + dz)
            * _through_hole_x()
        )

    return body


def rail_c_stub() -> Part:
    return Pos(0, _RAIL_C_YC, RAIL_C_Z) * Box(MEMBER, _RAIL_C_STUB_LEN, MEMBER)


def rail_d_stub() -> Part:
    return Pos(0, RAIL_D_Y, _RAIL_D_ZC) * Box(MEMBER, MEMBER, ARM_D_Z_L)


def add_frame_gusset_l(
    a: Assembly,
    name: str = "gusset_l_cd",
    location: Location = GUSSET_LOCATION,
) -> Assembly:
    """Add the C/D L-junction gusset to a scanner top-level assembly."""
    return a.add(name, gusset(), location=location)


# --- Assembly -----------------------------------------------------------

assembly = (
    Assembly()
    .add("gusset", gusset(), location=GUSSET_LOCATION)
    .add("rail_c", rail_c_stub())
    .add("rail_d", rail_d_stub())
    .assert_no_interference("gusset", "rail_c")
    .assert_no_interference("gusset", "rail_d")
    .assert_no_interference("rail_c", "rail_d")
)


if __name__ == "__main__":
    check(assembly, out="outputs")
    inspect(
        gusset(),
        method=FDM(up_axis=(1, 0, 0), wall_min_mm=WALL_MIN, overhang_max_deg=45.0),
        out="outputs",
        name="gusset",
    )
