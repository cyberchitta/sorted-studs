"""
M03 scanner — rev-3 sparse T-frame skeleton.

Successor to the rev-2 12-edge cube (`../skeleton/`). The cube's redundant
edges pulled no structural load once each mirror and camera clamped to a
single rail; rev 3 drops them and keeps only the members that carry load.
The remaining 4-rail T-frame plus a diagonal truss meets the same
stiffness target at roughly one third the member count, and leaves the
scan volume and eject path open by construction.

Rails (see `../NEXT_STEPS.md` for the full design rationale):

  * rail A — X-span at the below-mirror mount back-face plane
    (y ≈ -145, z ≈ +28). Both below-mirror pad-on-rail clamps grip
    directly, zero beam reach. Split into left/right halves at the
    T-junction so rail B runs continuously through.
  * rail B — Z-span at the same plane, continuous through the T.
    Forms the backbone from rail A's centre back to rail C, with
    outriggers past each junction for fore-aft base stability.
  * rail C — vertical post at the back-upper mount back-face plane
    (x = 0, z ≈ +180). Rises from rail B's back end past the back-
    upper mirror to rail D's height. Back-upper pad-on-post clamp
    grips directly.
  * rail D — Z-cantilever over the camera, sitting atop rail C.
    Camera pad-on-cantilever-underside clamp hangs below.

Plus two ~45° diagonal braces, one at each L-junction (rail B ↔ rail C
and rail C ↔ rail D). Each bridges its two rails with mitered ends
that sit flush (CORNER_GAP away) against the adjacent rail faces, so
they actually triangulate the joint rather than floating near it.
Both braces sit just forward of rail C at the back of the rig, so
the interior stays clear for the platform frame to slide in — the
central point of the rev-3 decoupled-platform architecture.

A single long diagonal from rail D (along the cantilever) down to rail
B would reduce camera-tip deflection more aggressively (~10×, by
converting rail D to a propped cantilever) but cuts straight through
the platform frame's volume. Joint-stiffening braces give a ~2×
reduction while keeping the interior open. If that factor turns out
insufficient once the rig is built and instrumented, the escape
hatch is a king-post above rail C with a tension strut to rail D's
tip, which stays entirely above y = 240 and so still clears the
platform.

Coordinate frame = world frame (matches `../params.py`):
    origin = brick centroid on the platform top face
    +X     = operator's right
    +Y     = up
    +Z     = away from operator (toward the back of the machine)

All members are stock 2020 T-slot aluminum extrusion — bought, not
printed, so no `inspect()` calls. Butt-jointed rails end CORNER_GAP
(0.3 mm) short of each neighbour's face so the cross-boundary
interference check reads a clean zero volume at every joint. Printed
gussets at the rail A ↔ B T-junction and the rail C ↔ D L-junction
land in a later slice (see `../NEXT_STEPS.md`); each gusset spans the
gap via heat-set inserts + T-nuts and carries the joint's bending
load. The diagonal truss attaches via the same gussets at its two
ends, with its body offset from the adjacent rails by ~10-20 mm so
the extrusion volumes don't intersect at this rev.

Rev-3 vs rev-2 differences:
  * No floor rails — rails A and B sit at the below-mirror mount plane
    and each below-mirror clamp has zero beam reach. Floor-standing
    feet are a follow-up slice.
  * No ceiling rails — rail D's height is set by clearance over the
    back-upper mount plate, not a uniform ceiling plane.
  * No separate enclosure; the camera rig and platform frame are
    decoupled, with the scan path open in every direction.
"""

import math
import sys
from pathlib import Path

from build123d import Box, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    CAMERA_POS,
    MIRROR_BACK_UPPER_NORMAL,
    MIRROR_BACK_UPPER_POS,
    MIRROR_BELOW_LEFT_NORMAL,
    MIRROR_BELOW_LEFT_POS,
    MIRROR_MOUNT_BACK_OFFSET,
)


# --- Member cross-section -----------------------------------------------

MEMBER = 20.0                   # 2020 extrusion cross-section
CORNER_GAP = 0.3                # butt-joint gap at each rail endpoint


# --- World-frame references --------------------------------------------
#
# Rails A/B sit at the below-mirror mount back-face plane (pad-on-rail
# convention); rail C sits at the back-upper back-face plane (pad-on-
# post). Both are derived from mount pose + back-offset so a viewpoint-
# optimiser update in `params.py` flows through automatically.

def _back_face_centre(pos, normal):
    return tuple(
        pos[i] - MIRROR_MOUNT_BACK_OFFSET * normal[i] for i in range(3)
    )


_BL_BACK = _back_face_centre(MIRROR_BELOW_LEFT_POS, MIRROR_BELOW_LEFT_NORMAL)
_BU_BACK = _back_face_centre(MIRROR_BACK_UPPER_POS, MIRROR_BACK_UPPER_NORMAL)


# --- Rail A — X-span at the below-mirror plane --------------------------

RAIL_A_Y = _BL_BACK[1]                           # ≈ -145.03
RAIL_A_Z = _BL_BACK[2]                           # ≈ 28.44
RAIL_A_X_HALF = 270.0                            # outrigger past mirror x=±207

# Half-rails end CORNER_GAP inboard of rail B's nearest face.
RAIL_A_INNER_X = MEMBER / 2 + CORNER_GAP         # = 10.3


# --- Rail B — Z-span at the below-mirror plane, continuous through T ----

RAIL_B_Y = RAIL_A_Y                              # coplanar with rail A
RAIL_B_Z_FRONT = -30.0                           # front outrigger past rail A junction (z≈28)
RAIL_B_Z_BACK = _BU_BACK[2] + 40.0               # ≈ 219.78, back outrigger past rail C


# --- Rail C — vertical post under the back-upper mirror -----------------

RAIL_C_Z = _BU_BACK[2]                           # ≈ 179.78
# Sits on rail B's +Y face with CORNER_GAP separator.
RAIL_C_Y_BOTTOM = RAIL_B_Y + MEMBER / 2 + CORNER_GAP    # ≈ -134.73
# Top set high enough to clear the back-upper mount plate's upper corner
# (y ≤ 227.2 for the 110 × 110 plate at the back-upper pose) and to
# present the L-junction elevation for rail D.
RAIL_C_Y_TOP = 240.0


# --- Rail D — Z-cantilever over the camera ------------------------------

RAIL_D_Y = RAIL_C_Y_TOP + CORNER_GAP + MEMBER / 2   # ≈ 250.3 (sits on rail C top)
RAIL_D_Z_BACK = RAIL_C_Z + MEMBER / 2            # ≈ 189.78 (L-junction aligned with rail C +Z face)
RAIL_D_Z_FRONT = -280.0                          # ~53 mm past camera (z ≈ -226.6)


# --- Corner braces — ~45° diagonals at the two L-junctions ------------
#
# Two short struts, one per L-junction, each bridging its two rails at
# ~45°. Both ends are mitered (cut parallel to the adjacent rail face
# minus CORNER_GAP) so the brace body sits flush against each rail.
#
# A single long diagonal from rail D (along the cantilever) down to
# rail B would drop camera-tip deflection by ~10× rather than the ~2×
# the corner braces give, but it crosses the rig's interior at x = 0
# — exactly the volume the platform frame slides into, which is the
# whole rev-3 premise. Corner braces keep the interior clear.
#
# If 2× isn't enough once the rig is instrumented, the escape hatch
# is a king-post above rail C plus a tension strut to rail D's tip,
# entirely above y = 240 so still clear of the platform.

# Each brace is specified by its two attach points — (signed) distances
# from the L-junction corner along each adjacent rail face. A 60 mm leg
# on each side gives a ~45° brace roughly 85 mm along its axis. Ends
# are mitered (cut flush to the adjacent rail face minus CORNER_GAP)
# so the brace body contacts both rails cleanly rather than floating
# in free space.

# B ↔ C brace — inside the lower-back L (rail B +Y face ↔ rail C -Z face).
BRACE_BC_ON_B_Z = 110.0           # attach on rail B: 60 mm forward of rail C -Z face
BRACE_BC_ON_C_Y = -75.0           # attach on rail C: 60 mm above rail B +Y face

# C ↔ D brace — inside the upper-back L (rail C -Z face ↔ rail D -Y face).
BRACE_CD_ON_C_Y = 180.0           # attach on rail C: 60 mm below rail D -Y face
BRACE_CD_ON_D_Z = 110.0           # attach on rail D: 60 mm forward of rail C -Z face


# --- Parametric sanity -------------------------------------------------

assert RAIL_A_X_HALF > abs(MIRROR_BELOW_LEFT_POS[0]), (
    f"rail A outrigger half-span (x=±{RAIL_A_X_HALF}) does not extend "
    f"past the below-mirror X position (x=±{abs(MIRROR_BELOW_LEFT_POS[0])})"
)

assert RAIL_B_Z_FRONT < RAIL_A_Z and RAIL_B_Z_BACK > RAIL_C_Z, (
    f"rail B must extend past both rail A junction (z={RAIL_A_Z:.2f}) "
    f"and rail C junction (z={RAIL_C_Z:.2f}); "
    f"got z ∈ [{RAIL_B_Z_FRONT}, {RAIL_B_Z_BACK}]"
)

assert RAIL_C_Y_TOP > MIRROR_BACK_UPPER_POS[1], (
    f"rail C top (y={RAIL_C_Y_TOP}) must clear the back-upper mirror "
    f"centre (y={MIRROR_BACK_UPPER_POS[1]}); mount plate upper corner "
    f"is a further ~52 mm above the centre"
)

assert RAIL_D_Z_FRONT < CAMERA_POS[2], (
    f"rail D front end (z={RAIL_D_Z_FRONT}) must reach past the "
    f"camera (z={CAMERA_POS[2]}) so the camera clamp can hang below"
)


# --- Member construction -----------------------------------------------

def rail_x(x_centre: float, x_len: float, y: float, z: float) -> Part:
    """X-direction rail (long axis along world +X)."""
    return Pos(x_centre, y, z) * Box(x_len, MEMBER, MEMBER)


def rail_y(x: float, y_centre: float, y_len: float, z: float) -> Part:
    """Y-direction rail / vertical post (long axis along world +Y)."""
    return Pos(x, y_centre, z) * Box(MEMBER, y_len, MEMBER)


def rail_z(x: float, y: float, z_centre: float, z_len: float) -> Part:
    """Z-direction rail (long axis along world +Z)."""
    return Pos(x, y, z_centre) * Box(MEMBER, MEMBER, z_len)


_BRACE_OVERRUN = 20.0           # extra length past each endpoint before mitering
_HALF_SPACE_SIZE = 2000.0       # big cube used as a half-space for cutting


def _half_space(axis: str, plane: float) -> Part:
    """Big box covering one side of an axis-aligned plane — used as a
    boolean cutter to miter a brace end flush with an adjacent rail.
    `axis` ∈ {'y_min','y_max','z_min','z_max'}; 'y_min' means the
    half-space y ≤ plane (i.e., material with y < plane is removed
    when this is subtracted).
    """
    s = _HALF_SPACE_SIZE
    if axis == "y_min":
        return Pos(0, plane - s / 2, 0) * Box(s, s, s)
    if axis == "y_max":
        return Pos(0, plane + s / 2, 0) * Box(s, s, s)
    if axis == "z_min":
        return Pos(0, 0, plane - s / 2) * Box(s, s, s)
    if axis == "z_max":
        return Pos(0, 0, plane + s / 2) * Box(s, s, s)
    raise ValueError(f"unknown axis: {axis}")


def mitered_brace_yz(
    p1: tuple[float, float, float],
    p2: tuple[float, float, float],
    cutters: list[Part],
) -> Part:
    """Rectangular-cross-section brace from p1 to p2 in the YZ plane
    (both endpoints at x = 0), mitered at each end by subtracting the
    supplied half-space cutters. The unmitered box is built with
    `_BRACE_OVERRUN` mm of extra length past each endpoint so the
    cutters have material to remove on both sides of the attach plane.

    Cross-section is MEMBER × MEMBER perpendicular to the axis; the
    box is built +Y-oriented, rotated by Rx(tilt) to align with the
    axis direction, translated to the midpoint, then cut.
    """
    assert abs(p1[0]) < 1e-9 and abs(p2[0]) < 1e-9, (
        "mitered_brace_yz assumes both endpoints lie on x = 0"
    )
    dy, dz = p2[1] - p1[1], p2[2] - p1[2]
    length = math.hypot(dy, dz)
    tilt_deg = math.degrees(math.atan2(dz, dy))
    midpoint = (
        (p1[0] + p2[0]) / 2,
        (p1[1] + p2[1]) / 2,
        (p1[2] + p2[2]) / 2,
    )
    brace = (
        Pos(*midpoint)
        * Rot(tilt_deg, 0, 0)
        * Box(MEMBER, length + 2 * _BRACE_OVERRUN, MEMBER)
    )
    for c in cutters:
        brace = brace - c
    return brace


# --- Assembly ----------------------------------------------------------

_rail_a_left_xc = (-RAIL_A_X_HALF + (-RAIL_A_INNER_X)) / 2
_rail_a_left_len = -RAIL_A_INNER_X - (-RAIL_A_X_HALF)

_rail_a_right_xc = (RAIL_A_INNER_X + RAIL_A_X_HALF) / 2
_rail_a_right_len = RAIL_A_X_HALF - RAIL_A_INNER_X

_rail_b_zc = (RAIL_B_Z_FRONT + RAIL_B_Z_BACK) / 2
_rail_b_len = RAIL_B_Z_BACK - RAIL_B_Z_FRONT

_rail_c_yc = (RAIL_C_Y_BOTTOM + RAIL_C_Y_TOP) / 2
_rail_c_len = RAIL_C_Y_TOP - RAIL_C_Y_BOTTOM

_rail_d_zc = (RAIL_D_Z_FRONT + RAIL_D_Z_BACK) / 2
_rail_d_len = RAIL_D_Z_BACK - RAIL_D_Z_FRONT


_MEMBERS = {
    "rail_a_left": rail_x(_rail_a_left_xc, _rail_a_left_len, RAIL_A_Y, RAIL_A_Z),
    "rail_a_right": rail_x(_rail_a_right_xc, _rail_a_right_len, RAIL_A_Y, RAIL_A_Z),
    "rail_b": rail_z(0.0, RAIL_B_Y, _rail_b_zc, _rail_b_len),
    "rail_c": rail_y(0.0, _rail_c_yc, _rail_c_len, RAIL_C_Z),
    "rail_d": rail_z(0.0, RAIL_D_Y, _rail_d_zc, _rail_d_len),
    "brace_bc": mitered_brace_yz(
        # Attach on rail B +Y face and rail C -Z face.
        (0.0, RAIL_B_Y + MEMBER / 2, BRACE_BC_ON_B_Z),
        (0.0, BRACE_BC_ON_C_Y, RAIL_C_Z - MEMBER / 2),
        cutters=[
            # Miter against rail B +Y face — CORNER_GAP clearance so
            # the boolean intersection reads a clean zero volume.
            _half_space("y_min", RAIL_B_Y + MEMBER / 2 + CORNER_GAP),
            # Miter against rail C -Z face.
            _half_space("z_max", RAIL_C_Z - MEMBER / 2 - CORNER_GAP),
        ],
    ),
    "brace_cd": mitered_brace_yz(
        # Attach on rail C -Z face and rail D -Y face.
        (0.0, BRACE_CD_ON_C_Y, RAIL_C_Z - MEMBER / 2),
        (0.0, RAIL_D_Y - MEMBER / 2, BRACE_CD_ON_D_Z),
        cutters=[
            # Miter against rail C -Z face.
            _half_space("z_max", RAIL_C_Z - MEMBER / 2 - CORNER_GAP),
            # Miter against rail D -Y face.
            _half_space("y_max", RAIL_D_Y - MEMBER / 2 - CORNER_GAP),
        ],
    ),
}

assembly = Assembly()
for _name, _part in _MEMBERS.items():
    assembly = assembly.add(_name, _part)

# Every pair of members must be non-overlapping. Butt-joints are
# separated by CORNER_GAP (0.3 mm); printed gussets bridge the gap in
# reality and are not modelled in this slice.
_names = list(_MEMBERS.keys())
for _i, _a in enumerate(_names):
    for _b in _names[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Members are stock 2020 aluminum extrusion — no per-part
    # printability pass.
