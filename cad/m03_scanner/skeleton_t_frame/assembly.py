"""
M03 scanner — rev-3 sparse T-frame skeleton.

Successor to the rev-2 12-edge cube (`../skeleton/`). The cube's redundant
edges pulled no structural load once each mirror and camera clamped to a
single rail; rev 3 drops them and keeps only the members that carry load.
The remaining 4-rail T-frame plus a diagonal truss meets the same
stiffness target at roughly one third the member count, and leaves the
scan volume and eject path open by construction.

Rails (see `../NEXT_STEPS.md` for the full design rationale):

  * rail A — X-span just below the below-mirror mount plates
    (y = -220, z ≈ +28), close enough to the mount back-face plane
    that the printed clamp is a short stub rather than the rev-2
    long beam. The "pad-on-rail" goal (zero beam reach) was
    incompatible with the tilted back face crossing the rail body,
    so rail A drops just far enough that the clamp base clears the
    mount plate's lowest world-Y point with a few mm margin.
    Split into left/right halves at the T-junction so rail B runs
    continuously through.
  * rail B — Z-span at the same Y, continuous through the T.
    Forms the backbone from rail A's front face back past rail C
    (back outrigger only). The rev-3 platform's Z-tongue lies
    coplanar with rail A/B (single shared bench at y = -230), so
    rail B can't extend past rail A toward the front without
    clashing with the tongue — the front outrigger was removed.
  * rail C — vertical column behind the back-upper mount plate
    (x = 0, z = +240). Same logic as rail A — pushed out of the
    mount plate's volume so a short stub clamp can host the back-
    upper mirror without a long beam. Single continuous extrusion
    from rail B's back end up past rail D to the strut anchor at
    the column top.
  * rail D — Z-cantilever over the camera, bolted to rail C's -Z
    face (T-junction). Camera pad-on-cantilever-underside clamp
    hangs below.

The rail B ↔ rail C L-junction carries a single ~45° corner brace
inside the L (mitered at both ends, CORNER_GAP from each adjacent
rail face). The interior stays clear for the platform frame to slide
in — the central point of the rev-3 decoupled-platform architecture.

The rail C ↔ rail D corner can't carry the same kind of corner
brace: the back-upper mirror mount sits in that corner, and any
diagonal from rail C's -Z face to rail D's -Y face crashes through
the mount + pad-clamp volume. Instead, rail C extends straight up
past rail D as a single column, and a strut runs from near the
column top down-and-forward to rail D's free end. The strut +
extended-rail-C + rail-D triangle converts rail D from a bare
~510 mm cantilever to a propped beam — nominally ~10× tip-
deflection reduction, vs ~2× for the corner brace that would have
lived in the closed L. Same structural payoff as a long
rail-B-to-D diagonal would give, but routed entirely above the
rig interior.

Rail D bolts to rail C's -Z face (T-junction, bracketed by the
rail C ↔ D printed gusset) rather than sitting on top of rail C —
rail C runs continuously, removing one bolted joint from the load
path between the strut anchor and rail B and replacing a stack of
three extrusions (rail D on rail C, king post on rail D) with a
single column.

The strut sits on the +Y side of rail D's plane and on the -Z side
of rail C's plane, so the back-upper mount + pad clamp + back-
upper mirror can occupy the lower L volume undisturbed.

Coordinate frame = world frame (matches `../params.py`):
    origin = brick centroid on the platform top face
    +X     = operator's right
    +Y     = up
    +Z     = away from operator (toward the back of the machine)

All members are stock 2020 T-slot aluminum extrusion — bought, not
printed, so no `inspect()` calls. Butt-jointed rails end CORNER_GAP
(0.3 mm) short of each neighbour's face so the cross-boundary
interference check reads a clean zero volume at every joint. Printed
gussets at the rail A ↔ B T-junction and the rail C ↔ D T-junction
land in a later slice (see `../NEXT_STEPS.md`); each gusset spans the
gap via heat-set inserts + T-nuts and carries the joint's bending
load. The diagonal truss attaches via the same gussets at its two
ends, with its body offset from the adjacent rails by ~10-20 mm so
the extrusion volumes don't intersect at this rev.

Rev-3 vs rev-2 differences:
  * No floor rails — rails A and B sit just below the below-mirror
    mount plates, with the printed clamp a short stub instead of a
    rev-2 long tilted beam. Floor-standing feet are a follow-up
    slice.
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
# Rails A/B sit just below the below-mirror mount plates; rail C sits
# behind the back-upper plate. Each rail is offset from its mount's
# back-face centre along the back-face outward normal by enough that
# the clamp base clears the mount plate's solid volume — see
# `MOUNT_CLEARANCE` derivation below.

def _back_face_centre(pos, normal):
    return tuple(
        pos[i] - MIRROR_MOUNT_BACK_OFFSET * normal[i] for i in range(3)
    )


_BL_BACK = _back_face_centre(MIRROR_BELOW_LEFT_POS, MIRROR_BELOW_LEFT_NORMAL)
_BU_BACK = _back_face_centre(MIRROR_BACK_UPPER_POS, MIRROR_BACK_UPPER_NORMAL)


# --- Rail offsets from mount back-face centres -------------------------
#
# The clamp base is BASE_H ≈ 15 mm tall and sits on the rail face that
# faces the mount. The mount plate (110 × 110 × 5 mm, tilted) bulges
# from its back-face centre by ~48 mm toward world -Y for the below-
# mirrors (and ~26 mm toward world +Z for the back-upper). To keep
# the clamp base clear of that plate volume with a few mm margin, the
# rail face must sit (HALF_PLATE_PROJ + BASE_H + MARGIN) further along
# the back-face outward normal than the back-face centre.
#
# Concretely for below-left:
#   plate min world-Y ≈ -188.4 (= mirror_pos.y - 55·|m_local_Y.y| - 8·|m_normal.y|)
#   rail +Y face must be ≤ -188.4 - margin_5 = -193.4
#   rail centerline ≤ -193.4 - 10 (rail half) = -203.4
#   include BASE_H so base top ≤ -193.4: rail centerline ≤ -218.4
# Round to -220 → 6.6 mm gap from base top to plate volume.
#
# For back-upper the same calculation lands rail centerline at
# z ≥ 235.4; round to 240 → 9.6 mm gap.

RAIL_A_Y = -220.0                                # base top y = -195, plate min y ≈ -188.4 → 6.6 mm gap
RAIL_A_Z = _BL_BACK[2]                           # ≈ 28.44; under mount-back centre Z so beam straight up
RAIL_A_X_HALF = 270.0                            # outrigger past mirror x=±207

# Half-rails end CORNER_GAP inboard of rail B's nearest face.
RAIL_A_INNER_X = MEMBER / 2 + CORNER_GAP         # = 10.3


# --- Rail B — Z-span at the same Y as rail A, continuous through T -----

RAIL_B_Y = RAIL_A_Y                              # coplanar with rail A
# Rail B's front face is flush with rail A's front face — no front
# outrigger. The rev-3 platform tongue lives at the same Y and butts
# against this face, precluding any forward extension here.
RAIL_B_Z_FRONT = RAIL_A_Z - MEMBER / 2           # = 18.44


# --- Rail C — vertical column behind the back-upper mirror -------------
#
# Single continuous extrusion from rail B's +Y face up past rail D
# to the strut anchor at the column top. RAIL_C_Y_TOP is derived
# below, after rail D, since the column extension above rail D is
# the design driver for the column's height.

RAIL_C_Z = 240.0                                 # base top z = 215, plate max z ≈ 205.4 → 9.6 mm gap
# Sits on rail B's +Y face with CORNER_GAP separator.
RAIL_C_Y_BOTTOM = RAIL_B_Y + MEMBER / 2 + CORNER_GAP    # ≈ -209.7

# Rail B back outrigger sits 40 mm past rail C's back face.
RAIL_B_Z_BACK = RAIL_C_Z + 40.0                  # = 280.0


# --- Rail D — Z-cantilever over the camera ------------------------------
#
# Bolts to rail C's -Z face (T-junction); centerline Y set directly
# by clearance over the back-upper mount plate corner (y ≤ 228.4)
# plus a few mm margin, not by sitting on rail C's top.

RAIL_D_Y = 250.3                                 # bottom face y = 240.3 → 11.9 mm gap above mount corner
RAIL_D_Z_BACK = RAIL_C_Z - MEMBER / 2 - CORNER_GAP   # = 229.7 (butted to rail C -Z face)
RAIL_D_Z_FRONT = -280.0                          # ~53 mm past camera (z ≈ -226.6)


# --- Rail C top — strut anchor elevation -------------------------------
#
# Column rises COLUMN_EXT_ABOVE_D mm above rail D's top face for the
# propped-beam triangle (extended rail C / strut / rail D). The 150
# mm pick gives a strut tilt of atan(150 / ~510) ≈ 16° below
# horizontal — reasonable mechanical advantage against vertical load
# on rail D's tip without driving the column unreasonably tall.

COLUMN_EXT_ABOVE_D = 150.0
RAIL_C_Y_TOP = RAIL_D_Y + MEMBER / 2 + COLUMN_EXT_ABOVE_D   # = 410.3


# --- Rail B ↔ rail C corner brace --------------------------------------
#
# Single ~45° diagonal inside the lower-back L. 60 mm legs along each
# adjacent rail face → ~85 mm hypotenuse, mitered at each end so the
# brace body sits flush against rail B's +Y face and rail C's -Z face
# (CORNER_GAP from each).

_BRACE_LEG = 60.0                 # leg length along each adjacent rail face

BRACE_BC_ON_B_Z = RAIL_C_Z - MEMBER / 2 - _BRACE_LEG     # = 170.0 (60 mm forward of rail C -Z face)
BRACE_BC_ON_C_Y = RAIL_B_Y + MEMBER / 2 + _BRACE_LEG     # = -150.0 (60 mm above rail B +Y face)


# --- Strut — props rail D's cantilever from the column top ------------
#
# Single mitered diagonal from rail C's column top corner (-Z face
# meets +Y face) down-and-forward to rail D's free-end corner (+Y
# face meets -Z face). The centerline reaches both rail-tip corners,
# and four half-space cutters trim the surrounding box so the strut
# body sits flush within both corners — no overhang past either
# rail tip and no gap inboard of either tip.
#
# Cuts (four half-spaces, all CORNER_GAP from a rail face):
#   * z=RAIL_C_Z - MEMBER/2 - GAP      — strut sits against rail C -Z face
#   * y=RAIL_C_Y_TOP - GAP             — strut top flush with column top
#   * y=RAIL_D_Y + MEMBER/2 + GAP      — strut sits on rail D +Y face
#   * z=RAIL_D_Z_FRONT + GAP           — strut tip flush with rail D's free end
# The first two miter the column-side corner; the second two miter
# the rail-D-side corner.


# --- Parametric sanity -------------------------------------------------

assert RAIL_A_X_HALF > abs(MIRROR_BELOW_LEFT_POS[0]), (
    f"rail A outrigger half-span (x=±{RAIL_A_X_HALF}) does not extend "
    f"past the below-mirror X position (x=±{abs(MIRROR_BELOW_LEFT_POS[0])})"
)

assert RAIL_B_Z_FRONT == RAIL_A_Z - MEMBER / 2 and RAIL_B_Z_BACK > RAIL_C_Z, (
    f"rail B must start flush with rail A's front face (no front "
    f"outrigger; expected z={RAIL_A_Z - MEMBER / 2:.2f}) and extend "
    f"past rail C junction (z={RAIL_C_Z:.2f}); "
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
    "strut_d": mitered_brace_yz(
        # Centerline reaches both rail-tip corners; four half-space
        # cutters trim the box to fit flush within both corners.
        (0.0, RAIL_C_Y_TOP, RAIL_C_Z - MEMBER / 2),
        (0.0, RAIL_D_Y + MEMBER / 2, RAIL_D_Z_FRONT),
        cutters=[
            # Column-side corner (rail C -Z face × rail C +Y face).
            _half_space("z_max", RAIL_C_Z - MEMBER / 2 - CORNER_GAP),
            _half_space("y_max", RAIL_C_Y_TOP - CORNER_GAP),
            # Rail-D-side corner (rail D +Y face × rail D -Z face).
            _half_space("y_min", RAIL_D_Y + MEMBER / 2 + CORNER_GAP),
            _half_space("z_min", RAIL_D_Z_FRONT + CORNER_GAP),
        ],
    ),
}

SKELETON_T_FRAME_PART_NAMES = tuple(_MEMBERS.keys())


# --- Assembly builder ---------------------------------------------------

def add_skeleton_t_frame(a: Assembly) -> Assembly:
    """Add all T-frame skeleton members in world coordinates and
    return the updated assembly. Exposed so the top-level scanner
    assembly composes via the public surface instead of reaching
    into module-private state.
    """
    for name, part in _MEMBERS.items():
        a = a.add(name, part)
    return a


# --- Standalone assembly ------------------------------------------------

assembly = add_skeleton_t_frame(Assembly())

# Every pair of members must be non-overlapping. Butt-joints are
# separated by CORNER_GAP (0.3 mm); printed gussets bridge the gap in
# reality and are not modelled in this slice.
for _i, _a in enumerate(SKELETON_T_FRAME_PART_NAMES):
    for _b in SKELETON_T_FRAME_PART_NAMES[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Members are stock 2020 aluminum extrusion — no per-part
    # printability pass.
