"""
M03 scanner — rev-3 rail C/D L-junction gusset.

Printed side plate that stiffens the rail C ↔ rail D junction in the
sparse scanner T-frame. Rail D bolts into rail C's -Y face as a
cantilever; this gusset lands on the shared +X side face of both rails
so it reinforces the corner without occupying the back-upper mirror
clamp volume on rail C's -Y face or the camera clamp volume on rail
D's underside.

Coordinate frame:
    origin : rail C/D joint on the rails' shared +X side face, at
             rail D centerline Z and rail C centerline Y
    +X     : away from the gripped rail side face
    +Y     : toward the back of the rig; rail D runs along Y
    +Z     : up along rail C
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
    T_FRAME_RAIL_C_FRONT_Y,
    T_FRAME_RAIL_D_BOTTOM_Z,
    T_FRAME_RAIL_D_Z,
    WALL_MIN,
)
from design_systems import color_of  # noqa: E402
from materials import material_of  # noqa: E402


# --- Parameters ---------------------------------------------------------

MEMBER = SKELETON_RAIL_MEMBER
RAIL_C_Y = T_FRAME_RAIL_C_FRONT_Y + MEMBER / 2
RAIL_D_Z = T_FRAME_RAIL_D_Z
RAIL_D_Y_BACK = T_FRAME_RAIL_C_FRONT_Y - 0.3

PLATE_T = 6.0
ARM_W = MEMBER
ARM_C_Z_L = 150.0
ARM_D_Y_L = 160.0

BOLT_HOLE_D = 5.5
C_BOLT_Z_OFFSETS = (-45.0, 45.0)
D_BOLT_Y_OFFSETS = (-115.0, -55.0)

_PLATE_X = MEMBER / 2 + PLATE_T / 2
_RAIL_C_ZC = RAIL_D_Z
_RAIL_C_STUB_LEN = 180.0
_RAIL_D_YC = RAIL_D_Y_BACK - ARM_D_Y_L / 2

GUSSET_LOCATION = Pos(MEMBER / 2, RAIL_C_Y, RAIL_D_Z)


# --- Parametric sanity --------------------------------------------------

assert ARM_W - BOLT_HOLE_D >= 2 * WALL_MIN, (
    f"bolt-hole-to-arm-edge wall {(ARM_W - BOLT_HOLE_D) / 2:.2f} mm < {WALL_MIN} mm"
)
assert ARM_C_Z_L / 2 > max(abs(z) for z in C_BOLT_Z_OFFSETS) + BOLT_HOLE_D / 2 + WALL_MIN, (
    "rail-C bolt holes need edge wall inside the vertical arm"
)
assert ARM_D_Y_L > abs(min(D_BOLT_Y_OFFSETS)) + BOLT_HOLE_D / 2 + WALL_MIN, (
    "rail-D bolt holes need edge wall inside the horizontal arm"
)


# --- Part functions -----------------------------------------------------

def _through_hole_x(hole_d: float = BOLT_HOLE_D) -> Part:
    """Clearance hole through the side plate into an extrusion side T-slot."""
    return Rot(0, 90, 0) * Cylinder(hole_d / 2, PLATE_T * 3)


def gusset() -> Part:
    """Single printed L plate in local coordinates."""
    c_arm = Pos(PLATE_T / 2, 0, 0) * Box(PLATE_T, ARM_W, ARM_C_Z_L)
    d_arm = Pos(PLATE_T / 2, RAIL_D_Y_BACK - RAIL_C_Y - ARM_D_Y_L / 2, 0) * Box(
        PLATE_T, ARM_D_Y_L, ARM_W
    )
    body = c_arm + d_arm

    for dz in C_BOLT_Z_OFFSETS:
        body -= Pos(PLATE_T / 2, 0, dz) * _through_hole_x()
    for dy in D_BOLT_Y_OFFSETS:
        body -= (
            Pos(PLATE_T / 2, RAIL_D_Y_BACK - RAIL_C_Y + dy, 0)
            * _through_hole_x()
        )

    return body


def rail_c_stub() -> Part:
    return Pos(0, RAIL_C_Y, _RAIL_C_ZC) * Box(MEMBER, MEMBER, _RAIL_C_STUB_LEN)


def rail_d_stub() -> Part:
    return Pos(0, _RAIL_D_YC, RAIL_D_Z) * Box(MEMBER, ARM_D_Y_L, MEMBER)


def add_frame_gusset_l(
    a: Assembly,
    name: str = "gusset_l_cd",
    location: Location = GUSSET_LOCATION,
) -> Assembly:
    """Add the C/D L-junction gusset to a scanner top-level assembly."""
    return a.add(
        name, gusset(), location=location,
        material=material_of(name), color=color_of(name),
    )


# --- Assembly -----------------------------------------------------------

assembly = (
    Assembly()
    .add("gusset", gusset(), location=GUSSET_LOCATION, material="pla_matte")
    .add("rail_c", rail_c_stub(), material="aluminium_anodized")
    .add("rail_d", rail_d_stub(), material="aluminium_anodized")
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
