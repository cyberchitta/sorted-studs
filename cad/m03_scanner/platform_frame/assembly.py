"""
M03 scanner — rev-3 platform frame.

Standalone freestanding 2020 box that drops into the camera rig's
open front (decoupled from `../skeleton_t_frame/`). Carries the
future hinged-quad platform on a top cross-rail; the platform_frame's
optical job is to host the ~180 × 180 mm acrylic platform centred at
the world origin without obstructing the below-mirror sightlines or
the camera-rig structure.

Members (5 total):
  * floor_front_x — X-axis rail joining the two front-post bottoms;
    sets the front-of-frame contact line.
  * floor_tongue_z — Z-axis rail running back from the front rail's
    +Z face to a single back foot. Combines with the front rail to
    give 3-point footprint stability (two front-post bottoms + tongue
    tip), no rocking on an uneven bench.
  * post_left, post_right — vertical posts at the front corners,
    rising from the front rail to the cross-rail bottom.
  * cross_rail_top_x — top X-axis rail spanning the two posts. Sits
    on the post tops (T-junction). Hosts the slice (b) hinge bearings
    that carry the platform's hinge pin.

Coordinate frame = world frame (matches `../params.py`):
    origin = brick centroid on the platform top face
    +X     = operator's right
    +Y     = up
    +Z     = away from operator (toward the back of the machine)

Clearance to camera rig: floor rails at y = -245 (top face -235) sit
5 mm below the rev-3 rig's rail A / rail B bottom faces at y = -230,
so the tongue passes under rail B at z ∈ [-30, +100] in 3D without
interference. Front posts at z = -100 are 70 mm forward of rail A
(z = +28); cross-rail at z = -100 is also clear of rail A.

All members are stock 2020 aluminum extrusion — bought, not printed —
so no `inspect()` calls. Butt-jointed members end CORNER_GAP (0.3 mm)
short of each neighbour's face so the cross-boundary interference
checks read a clean zero volume at every joint. Printed corner
brackets at the post-to-rail joints land in a later slice.

Load cells are deferred. Rev-3 is optical-only at this slice, so the
slice (b) hinge bearings are simple printed bushing blocks; adding
load cells later replaces them with a load-cell-pocket variant at the
same bolt pattern, with no change to platform_frame.
"""

import sys
from pathlib import Path

from build123d import Box, Part, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    PLATFORM_FRAME_CROSS_RAIL_Y,
    PLATFORM_FRAME_FLOOR_Y,
    PLATFORM_FRAME_POST_X,
    PLATFORM_FRAME_POST_Z,
    PLATFORM_FRAME_TONGUE_Z_BACK,
)


# --- Member cross-section ----------------------------------------------

MEMBER = 20.0                   # 2020 extrusion
CORNER_GAP = 0.3                # butt-joint gap


# --- Derived dimensions ------------------------------------------------

# Front rail and cross-rail span both posts and overhang each post
# centerline by half a member, so the rail end face is flush with the
# post outer face. Same convention as skeleton_t_frame's rail-D
# free-end.
FLOOR_FRONT_X_LEN = 2 * PLATFORM_FRAME_POST_X + MEMBER
CROSS_RAIL_X_LEN = FLOOR_FRONT_X_LEN

# Tongue runs along Z from the front rail's +Z face to the tongue tip;
# CORNER_GAP-separated at the front-rail joint.
_TONGUE_Z_FRONT = PLATFORM_FRAME_POST_Z + MEMBER / 2 + CORNER_GAP
_TONGUE_Z_LEN = PLATFORM_FRAME_TONGUE_Z_BACK - _TONGUE_Z_FRONT
_TONGUE_Z_CENTER = (_TONGUE_Z_FRONT + PLATFORM_FRAME_TONGUE_Z_BACK) / 2

# Posts: bottom CORNER_GAP above front-rail +Y face, top CORNER_GAP
# below cross-rail -Y face.
_POST_BOTTOM_Y = PLATFORM_FRAME_FLOOR_Y + MEMBER / 2 + CORNER_GAP
_POST_TOP_Y = PLATFORM_FRAME_CROSS_RAIL_Y - MEMBER / 2 - CORNER_GAP
_POST_LEN = _POST_TOP_Y - _POST_BOTTOM_Y
_POST_CENTER_Y = (_POST_BOTTOM_Y + _POST_TOP_Y) / 2


# --- Sanity asserts ----------------------------------------------------

assert _POST_LEN > 0, (
    f"post length {_POST_LEN:.1f} mm ≤ 0; cross-rail Y "
    f"({PLATFORM_FRAME_CROSS_RAIL_Y}) must sit above floor "
    f"({PLATFORM_FRAME_FLOOR_Y}) by more than MEMBER + 2·GAP"
)

assert _TONGUE_Z_LEN > 0, (
    f"tongue length {_TONGUE_Z_LEN:.1f} mm ≤ 0; tongue back "
    f"({PLATFORM_FRAME_TONGUE_Z_BACK}) must extend past front rail "
    f"+Z face ({_TONGUE_Z_FRONT})"
)

assert PLATFORM_FRAME_POST_Z + MEMBER / 2 < 18.44, (
    f"front rail +Z face ({PLATFORM_FRAME_POST_Z + MEMBER / 2}) is not "
    f"forward of the camera rig's rail A front face (z ≈ 18.44); the "
    f"frame's vertical posts would crash rail A in 3D"
)


# --- Member helpers ----------------------------------------------------

def rail_x(x_centre: float, x_len: float, y: float, z: float) -> Part:
    """X-direction rail (long axis along world +X)."""
    return Pos(x_centre, y, z) * Box(x_len, MEMBER, MEMBER)


def rail_y(x: float, y_centre: float, y_len: float, z: float) -> Part:
    """Y-direction rail / vertical post (long axis along world +Y)."""
    return Pos(x, y_centre, z) * Box(MEMBER, y_len, MEMBER)


def rail_z(x: float, y: float, z_centre: float, z_len: float) -> Part:
    """Z-direction rail (long axis along world +Z)."""
    return Pos(x, y, z_centre) * Box(MEMBER, MEMBER, z_len)


# --- Members -----------------------------------------------------------

_MEMBERS = {
    "floor_front_x": rail_x(
        0.0, FLOOR_FRONT_X_LEN, PLATFORM_FRAME_FLOOR_Y, PLATFORM_FRAME_POST_Z
    ),
    "floor_tongue_z": rail_z(
        0.0, PLATFORM_FRAME_FLOOR_Y, _TONGUE_Z_CENTER, _TONGUE_Z_LEN
    ),
    "post_left": rail_y(
        -PLATFORM_FRAME_POST_X, _POST_CENTER_Y, _POST_LEN, PLATFORM_FRAME_POST_Z
    ),
    "post_right": rail_y(
        +PLATFORM_FRAME_POST_X, _POST_CENTER_Y, _POST_LEN, PLATFORM_FRAME_POST_Z
    ),
    "cross_rail_top_x": rail_x(
        0.0, CROSS_RAIL_X_LEN, PLATFORM_FRAME_CROSS_RAIL_Y, PLATFORM_FRAME_POST_Z
    ),
}

PLATFORM_FRAME_PART_NAMES = tuple(_MEMBERS.keys())


# --- Assembly builder --------------------------------------------------

def add_platform_frame(a: Assembly) -> Assembly:
    """Add all 2020 platform-frame members in world coordinates and
    return the updated assembly. Exposed so the top-level scanner
    assembly composes via the public surface instead of reaching
    into module-private state.
    """
    for name, part in _MEMBERS.items():
        a = a.add(name, part)
    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_platform_frame(Assembly())

# Every pair of members must be non-overlapping. Butt-joints are
# separated by CORNER_GAP (0.3 mm); printed corner brackets bridge the
# gaps in reality and are not modelled in this slice.
for _i, _a in enumerate(PLATFORM_FRAME_PART_NAMES):
    for _b in PLATFORM_FRAME_PART_NAMES[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Members are stock 2020 aluminum extrusion — no per-part
    # printability pass.
