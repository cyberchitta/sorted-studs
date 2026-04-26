"""
M03 scanner — rev-3 turret frame (slice a of the 4-station turret).

Standalone freestanding 2020 floor frame on the shared bench
(y = -220) replacing the rev-3 platform_frame's role as the
M2 → M3 → M5 mechanical interface. The single-platform stack
(`../platform_frame/`, `../platform_hinge_bearing/`,
`../platform_acrylic/`) is being superseded by the 4-station turret
described in `../turret_design_brief.md`; this slice delivers just
the floor frame, with the hub + drive, arms, platform cartridges and
cam ring landing in later slices.

Members (5 total, all stock 2020 aluminum extrusion at y = -220):
  * floor_back_x   — back X-rail butting the camera rig's rail A
                     front face (CORNER_GAP separator).
  * floor_front_x  — front X-rail at z = TURRET_FRAME_FRONT_Z (under
                     the RETURN station centroid).
  * floor_left_z   — left Z-rail at x = -TURRET_FRAME_SIDE_X.
  * floor_right_z  — right Z-rail at x = +TURRET_FRAME_SIDE_X.
  * floor_cross_x  — center X-rail at z = TURRET_HUB_Z. Mounting line
                     shared by the slice (b) hub bearing (x = 0), the
                     M2 singulator (LOAD, x = +130) and the M5
                     diverter (DUMP, x = -130).

Coordinate frame = world frame (matches `../params.py`):
    origin = brick centroid on the IMAGE platform top face
    +X     = operator's right
    +Y     = up
    +Z     = away from operator (toward the back of the machine)

Clearance to camera rig: floor rails are coplanar with the rig's
rail A / rail B (centerline y = -220, both share a single bench).
The back rail butts rail A's front face with a CORNER_GAP separator;
side rails extend forward to the front rail. Front-half of the cam
ring (around LOAD/RETURN/DUMP) projects past the camera-rig front
face by design — that's the open-front operator-access region the
rig's rev-3 architecture preserves.

Members are stock 2020 aluminum extrusion — bought, not printed —
so no `inspect()` calls. Butt-jointed members end CORNER_GAP (0.3 mm)
short of each neighbour's face so the cross-boundary interference
checks read a clean zero volume at every joint. Printed corner
brackets at the rail joints land in a later slice.
"""

import sys
from pathlib import Path

from build123d import Box, Part, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    T_FRAME_RAIL_A_FRONT_Z,
    TURRET_ARM_R,
    TURRET_FRAME_CROSS_Z,
    TURRET_FRAME_FLOOR_Y,
    TURRET_HUB_Z,
)


# --- Frame perimeter geometry (private to this sub-assembly) ------------

# Back X-rail centerline Z. The rail's +Z face sits CORNER_GAP forward
# of the camera rig's rail A front face (z = T_FRAME_RAIL_A_FRONT_Z ≈
# +18.44), so the centerline is back-face minus MEMBER/2.
TURRET_FRAME_BACK_Z = T_FRAME_RAIL_A_FRONT_Z - 0.3 - 20.0 / 2    # ≈ +8.14

# Front X-rail centerline Z. Under the RETURN station centroid
# (z = -260). The RETURN platform's outer edge cantilevers 90 mm
# forward of this rail (to z = -350); accepted because the platform
# is light, the cam ring (slice e) carries the swing-load, and
# stopping the floor here keeps the bench footprint bounded.
TURRET_FRAME_FRONT_Z = TURRET_HUB_Z - TURRET_ARM_R               # = -260

# Side Z-rails X. ±150 sits 20 mm outboard of the LOAD/DUMP centroid
# lines at x = ±130 — gives the singulator and diverter mounting room
# on the cross-rail's outboard end without the side rail's outer face
# crashing the swept platform footprint at LOAD/DUMP (LOAD platform's
# outboard X face at +220 is at y = 0, well above the y = -220 side
# rail).
TURRET_FRAME_SIDE_X = 150.0


# --- Member cross-section ----------------------------------------------

MEMBER = 20.0                   # 2020 extrusion
CORNER_GAP = 0.3                # butt-joint gap


# --- Derived dimensions ------------------------------------------------

# Perimeter X-rails (back, front) overhang each side rail's centerline
# by half a member so the rail end face is flush with the side rail's
# outer face — same convention as platform_frame.
_PERIMETER_X_LEN = 2 * TURRET_FRAME_SIDE_X + MEMBER

# Side Z-rails fit between the back and front X-rails with a
# CORNER_GAP separator at each end.
_SIDE_Z_FRONT = TURRET_FRAME_FRONT_Z + MEMBER / 2 + CORNER_GAP
_SIDE_Z_BACK = TURRET_FRAME_BACK_Z - MEMBER / 2 - CORNER_GAP
_SIDE_Z_LEN = _SIDE_Z_BACK - _SIDE_Z_FRONT
_SIDE_Z_CENTER = (_SIDE_Z_FRONT + _SIDE_Z_BACK) / 2

# Center cross X-rail fits between the side rails (all rails share
# y = -220 so the cross can't pass through them — same constraint
# the side rails apply to the front/back X-rails).
_CROSS_X_LEFT = -TURRET_FRAME_SIDE_X + MEMBER / 2 + CORNER_GAP
_CROSS_X_RIGHT = +TURRET_FRAME_SIDE_X - MEMBER / 2 - CORNER_GAP
_CROSS_X_LEN = _CROSS_X_RIGHT - _CROSS_X_LEFT


# --- Sanity asserts ----------------------------------------------------

assert _SIDE_Z_LEN > 0, (
    f"side rail length {_SIDE_Z_LEN:.1f} mm ≤ 0; front "
    f"({TURRET_FRAME_FRONT_Z}) must sit forward of back "
    f"({TURRET_FRAME_BACK_Z}) by more than MEMBER + 2·GAP"
)

# Cross-rail Z must lie strictly between front and back rail Z so it
# can fit between the side rails as a third X-rail.
assert TURRET_FRAME_FRONT_Z + MEMBER / 2 < TURRET_FRAME_CROSS_Z < TURRET_FRAME_BACK_Z - MEMBER / 2, (
    f"cross-rail Z ({TURRET_FRAME_CROSS_Z}) must sit strictly between "
    f"front ({TURRET_FRAME_FRONT_Z}) and back ({TURRET_FRAME_BACK_Z}) "
    f"with at least MEMBER/2 clearance to each"
)

# Back rail must clear the camera rig's rail A front face. Both share
# y = -220 so a z-overlap would crash in 3D (same fault mode as the
# platform_frame tongue ↔ rail B reconciliation).
_BACK_RAIL_BACK_FACE_Z = TURRET_FRAME_BACK_Z + MEMBER / 2
assert _BACK_RAIL_BACK_FACE_Z + CORNER_GAP <= T_FRAME_RAIL_A_FRONT_Z, (
    f"back rail back face ({_BACK_RAIL_BACK_FACE_Z}) needs at least "
    f"{CORNER_GAP} mm clearance from rail A's front face "
    f"({T_FRAME_RAIL_A_FRONT_Z:.2f}); they share y = -220 and would crash"
)


# --- Member helpers ----------------------------------------------------

def rail_x(x_centre: float, x_len: float, y: float, z: float) -> Part:
    """X-direction rail (long axis along world +X)."""
    return Pos(x_centre, y, z) * Box(x_len, MEMBER, MEMBER)


def rail_z(x: float, y: float, z_centre: float, z_len: float) -> Part:
    """Z-direction rail (long axis along world +Z)."""
    return Pos(x, y, z_centre) * Box(MEMBER, MEMBER, z_len)


# --- Members -----------------------------------------------------------

_MEMBERS = {
    "floor_back_x": rail_x(
        0.0, _PERIMETER_X_LEN, TURRET_FRAME_FLOOR_Y, TURRET_FRAME_BACK_Z
    ),
    "floor_front_x": rail_x(
        0.0, _PERIMETER_X_LEN, TURRET_FRAME_FLOOR_Y, TURRET_FRAME_FRONT_Z
    ),
    "floor_left_z": rail_z(
        -TURRET_FRAME_SIDE_X, TURRET_FRAME_FLOOR_Y, _SIDE_Z_CENTER, _SIDE_Z_LEN
    ),
    "floor_right_z": rail_z(
        +TURRET_FRAME_SIDE_X, TURRET_FRAME_FLOOR_Y, _SIDE_Z_CENTER, _SIDE_Z_LEN
    ),
    "floor_cross_x": rail_x(
        0.0, _CROSS_X_LEN, TURRET_FRAME_FLOOR_Y, TURRET_FRAME_CROSS_Z
    ),
}

TURRET_FRAME_PART_NAMES = tuple(_MEMBERS.keys())


# --- Assembly builder --------------------------------------------------

def add_turret_frame(a: Assembly) -> Assembly:
    """Add all 2020 turret-frame members in world coordinates and
    return the updated assembly. Exposed so the top-level scanner
    assembly composes via the public surface instead of reaching into
    module-private state.
    """
    for name, part in _MEMBERS.items():
        a = a.add(name, part)
    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_turret_frame(Assembly())

# Every pair of members must be non-overlapping. Butt-joints are
# separated by CORNER_GAP (0.3 mm); printed corner brackets bridge the
# gaps in reality and are not modelled in this slice.
for _i, _a in enumerate(TURRET_FRAME_PART_NAMES):
    for _b in TURRET_FRAME_PART_NAMES[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Members are stock 2020 aluminum extrusion — no per-part
    # printability pass.
