"""
M03 scanner — rev-3 rail C/D L-junction gusset.

Printed side plate that stiffens the rail C ↔ rail D junction in the
sparse scanner T-frame. Rail D bolts into rail C's -Y face as a
cantilever; this gusset lands on a rail X-side face (one instance per
side, mirrored about x = 0) and reinforces the corner without
occupying the back-upper camera clamp volume on rail C's -Y face or
the camera clamp volume on rail D's underside.

The strut-era gusset straddled rail D's centerline Z (75 mm of arm
above and below the joint). With the strut and the column extension
gone, rail C now terminates flush with rail D's top face — the C arm
runs only downward from the joint (its top edge flush with rail C's
top end-face), and the bolt pattern shifts down in lockstep so both
rail-C bolts land in solid extrusion below rail D.

Two instances cover the joint (mirrored about x = 0) instead of rev-3's
single +X-side plate. Symmetric load path takes the C/D moment as a
balanced couple; the back-upper camera mount's wedge body fills the
remaining lower-corner L volume between them.

Coordinate frame (per instance):
    origin : rail C/D joint on the gripped rail X-side face, at rail
             D centerline Z and rail C centerline Y
    +X     : away from the gripped rail side face (so plate body
             extends in +X regardless of which world-X side the
             instance grips — `side=±1` flips the placement, not
             the body)
    +Y     : toward the back of the rig; rail D runs along Y
    +Z     : up along rail C
"""

import sys
from pathlib import Path

from build123d import Box, Cylinder, Location, Part, Plane, Pos, Rot, mirror

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
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

# C arm centered ARM_C_Z_L/2 below the L corner, top edge flush with
# the rail-C/rail-D top face (local z = +MEMBER/2; rail C's end face
# and rail D's top side face are coplanar at the L corner).
ARM_C_Z_TOP = MEMBER / 2                            # = +10 (rail D top face in local)
ARM_C_Z_CTR = ARM_C_Z_TOP - ARM_C_Z_L / 2           # = -65

BOLT_HOLE_D = 5.5
# Rail-C bolts sit 30 and 90 mm below rail D's centerline (i.e., 40
# and 100 mm below rail C's top end-face). Both land in solid rail C
# extrusion well clear of the rail-D end and the gusset arm edges.
C_BOLT_Z_OFFSETS = (-90.0, -30.0)
D_BOLT_Y_OFFSETS = (-115.0, -55.0)

_PLATE_X = MEMBER / 2 + PLATE_T / 2
_RAIL_C_ZC = RAIL_D_Z
_RAIL_C_STUB_LEN = 180.0
_RAIL_D_YC = RAIL_D_Y_BACK - ARM_D_Y_L / 2


def gusset_location(side: int = +1) -> Location:
    """World placement for the +X (`side=+1`) or -X (`side=-1`)
    instance. Plate body extends in plate-local +X regardless of side;
    `side=-1` mirrors via `Mirror` in the world frame so the plate
    grows away from rail C's -X face."""
    return Pos(side * MEMBER / 2, RAIL_C_Y, RAIL_D_Z)


GUSSET_LOCATION = gusset_location(+1)               # legacy alias


# --- Parametric sanity --------------------------------------------------

assert ARM_W - BOLT_HOLE_D >= 2 * WALL_MIN, (
    f"bolt-hole-to-arm-edge wall {(ARM_W - BOLT_HOLE_D) / 2:.2f} mm < {WALL_MIN} mm"
)
_arm_c_lo = ARM_C_Z_CTR - ARM_C_Z_L / 2
_arm_c_hi = ARM_C_Z_CTR + ARM_C_Z_L / 2
for _dz in C_BOLT_Z_OFFSETS:
    assert _arm_c_lo + BOLT_HOLE_D / 2 + WALL_MIN <= _dz <= _arm_c_hi - BOLT_HOLE_D / 2 - WALL_MIN, (
        f"rail-C bolt at local z={_dz} needs edge wall inside the "
        f"vertical arm (z ∈ [{_arm_c_lo}, {_arm_c_hi}])"
    )
assert ARM_D_Y_L > abs(min(D_BOLT_Y_OFFSETS)) + BOLT_HOLE_D / 2 + WALL_MIN, (
    "rail-D bolt holes need edge wall inside the horizontal arm"
)


# --- Part functions -----------------------------------------------------

def _through_hole_x(hole_d: float = BOLT_HOLE_D) -> Part:
    """Clearance hole through the side plate into an extrusion side T-slot."""
    return Rot(0, 90, 0) * Cylinder(hole_d / 2, PLATE_T * 3)


def gusset() -> Part:
    """Single printed L plate in local coordinates. Body extends in
    plate-local +X; the `add_frame_gusset_l(side=...)` placement
    mirrors world-X for the -X-side instance."""
    c_arm = Pos(PLATE_T / 2, 0, ARM_C_Z_CTR) * Box(PLATE_T, ARM_W, ARM_C_Z_L)
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
    name: str = "gusset_l_cd_pos_x",
    side: int = +1,
) -> Assembly:
    """Add one C/D L-junction gusset instance. ``side=+1`` grips
    rail's +X side face; ``side=-1`` grips the -X face (body
    mirrored about world x = 0)."""
    body = gusset()
    if side == -1:
        body = mirror(body, about=Plane.YZ)
    elif side != +1:
        raise ValueError(f"side must be ±1, got {side}")
    return a.add(
        name, body, location=gusset_location(side),
        material=material_of(name), color=color_of(name),
    )


# --- Assembly -----------------------------------------------------------

assembly = (
    Assembly()
    .add("gusset", gusset(), location=gusset_location(+1), material="pla_matte")
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
