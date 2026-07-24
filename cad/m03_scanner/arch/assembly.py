"""
M03 scanner — arch (Phase-2 integrated frame).

Vertical 2020 posts standing on the back outriggers behind the back
rail, carrying rail D as a Y-axis cantilever overhead. Successor to
the rev-3 rig's rail B + rail C + B↔C corner brace + the C/D L-
junction. Together with the shared back rail (now hosted in
`../frame/`) and rail D, this is the integrated rig+turret
structural frame described in `../integrated_frame_handoff.md`.

Members (5 total — all stock 2020 aluminum extrusion):
  * arch_post_pos_x — vertical post at m03-local x = +ARCH_POST_X
                      (= +TURRET_FRAME_SIDE_X = +145), y = ARCH_POST_Y
                      = +55 (well behind the back rail). Bottom face
                      sits CORNER_GAP above the back outrigger's top
                      face; top face flush with rail D's top face.
                      Post body footprint Y ∈ [+45, +65] sits on the
                      outrigger top (`TURRET_FRAME_OUTRIG_Y_BACK`
                      grew 170 → 190 in turret-local in the same
                      slice). Post body shares X range with the
                      below-* clamp body but separates in Y (post
                      Y ∈ [+45, +65] vs clamp Y ∈ [+7, +27]) and
                      clears the below-* mount plate's +Y reach
                      (≈ +41.9 — see plate-clearance assert below).
                      The Y-shift puts the post (a) outside every
                      lens cone (≈ 54° off the back-upper lens axis,
                      the binding "in FOV?" test; well outside the
                      below-* and front-low cones too) and (b)
                      outside the indexing platform sweep envelope
                      (post inner corner ~217 mm from hub vs sweep
                      envelope 194.5 mm — sweep-clearance assert
                      below).
  * arch_post_neg_x — X-mirror.
  * arch_stub_pos_x — short X-axis stub at z = ARCH_CROSSBAR_Z
                      (= rail D centerline), y = ARCH_CROSSBAR_Y
                      (= post Y = +55) running from the +X post's
                      inner face to rail D's +X face (CORNER_GAP each).
                      Plays the structural role of the demolished
                      rail C top + L-gusset corner.
  * arch_stub_neg_x — X-mirror.
  * rail_d          — Y-axis cantilever at x = 0 (m03-local) spanning
                      y ∈ [T_FRAME_RAIL_D_Y_FRONT, T_FRAME_RAIL_D_Y_BACK]
                      = [-150, +150]. With the arch crossbar at
                      y = +55, both camera clamps hang FORWARD of the
                      arch on a 205 mm front cantilever: front-low at
                      y ≈ -150 (tip), back-upper at y ≈ +115.6
                      (60 mm forward of the arch). The 95 mm tail
                      past the arch (to y = +150) carries the back-
                      upper clamp's BASE_L = 60 mm footprint that
                      extends past the plate-back (+115.6 → +145.6)
                      with 4.4 mm of bolt-engagement margin past it.

Rail D is a continuous member centred on x = 0; the two arch stubs
butt its east/west faces (CORNER_GAP separator). Posts and rail D are
not co-planar in X (posts at x = ±145, rail D at x = 0), so they
never share volume — only the stubs do the bridging.

The "arch top" detail (full crossbar vs L-brackets vs one-sided
L-arm — see the handoff's `Open in Phase 2` block) is realised here
as **two inward stubs**: a full crossbar would require splitting rail
D into two pieces or stacking the crossbar above rail D, neither of
which is clean. The two-stub layout keeps rail D continuous through
the arch-crossbar Y plane and stays out of the below-* camera FOVs
(the inward stubs sit at z = +1188 world, well above the brick
centroid at z = +1030).

Internal printed gussets at the post-to-side-rail and post-to-stub
joints are deferred to a follow-up slice. T-slot bolts via standard
2020 corner brackets cover the joints in the meantime; the `khana
check` butt-joint clearance reads zero volume at every interface
without modelled gussets.

Coordinate frame = m03-local (rig idiom):
    origin = brick centroid on the IMAGE platform top face
    +X     = operator's right
    +Y     = away from operator
    +Z     = up

Members are stock 2020 aluminum extrusion — bought, not printed —
no `inspect()` calls.
"""

import math
import sys
from pathlib import Path

from build123d import Box, Part, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    ARCH_CROSSBAR_Y,
    ARCH_CROSSBAR_Z,
    ARCH_POST_X,
    ARCH_POST_Y,
    ARCH_POST_Z_BOTTOM,
    ARCH_POST_Z_TOP,
    CAMERA_MOUNT_PLATE_T,
    PLATFORM_ACRYLIC_SIZE,
    SHARED_BACK_RAIL_Y,
    SKELETON_RAIL_MEMBER,
    T_FRAME_RAIL_D_Y_BACK,
    T_FRAME_RAIL_D_Y_FRONT,
    T_FRAME_RAIL_D_Z,
    TURRET_ARM_R,
    VERTEX_BACK_UPPER_POS,
    VERTEX_BELOW_LEFT_POS,
    VERTEX_BELOW_RIGHT_POS,
    VERTEX_FRONT_LOW_POS,
    lens_axis_unit,
)
from design_systems import color_of  # noqa: E402
from materials import material_of  # noqa: E402

# Plate edge-grip half-extent — half the bonded mount plate's longest
# in-plane dimension. The plate is square at PLATE_SIZE = 50 (private
# to camera_mount_esp32cam); we mirror the half-size here to drive the
# below-* plate-clearance assert below. If PLATE_SIZE in
# camera_mount_esp32cam ever changes, update this mirror in lockstep.
_MIRROR_PLATE_HALF = 25.0


# --- Member cross-section ----------------------------------------------

MEMBER = SKELETON_RAIL_MEMBER       # = 20.0
CORNER_GAP = 0.3                    # butt-joint gap


# --- Derived dimensions ------------------------------------------------

# Posts: bottom face at ARCH_POST_Z_BOTTOM (= back-rail top face +
# CORNER_GAP), top face at ARCH_POST_Z_TOP (= rail D top face).
_POST_LEN = ARCH_POST_Z_TOP - ARCH_POST_Z_BOTTOM
_POST_ZC = (ARCH_POST_Z_BOTTOM + ARCH_POST_Z_TOP) / 2

# Inward stubs: span between rail D's east/west face (with CORNER_GAP)
# and the post's inner X face (with CORNER_GAP).
_STUB_X_INNER = MEMBER / 2 + CORNER_GAP                       # = 10.3 (rail D outer face + GAP)
_STUB_X_OUTER = ARCH_POST_X - MEMBER / 2 - CORNER_GAP         # = 134.7 (post inner face - GAP)
_STUB_LEN = _STUB_X_OUTER - _STUB_X_INNER                     # = 124.4
_STUB_XC_POS = (_STUB_X_INNER + _STUB_X_OUTER) / 2            # = +72.5
_STUB_XC_NEG = -_STUB_XC_POS                                  # = -72.5

# Rail D: continuous Y-axis member centred on x = 0.
_RAIL_D_LEN = T_FRAME_RAIL_D_Y_BACK - T_FRAME_RAIL_D_Y_FRONT  # = 300
_RAIL_D_YC = (T_FRAME_RAIL_D_Y_FRONT + T_FRAME_RAIL_D_Y_BACK) / 2   # = 0


# --- Parametric sanity -------------------------------------------------

assert _POST_LEN > 0, (
    f"arch post length {_POST_LEN:.1f} mm ≤ 0; check ARCH_POST_Z_TOP "
    f"({ARCH_POST_Z_TOP}) > ARCH_POST_Z_BOTTOM ({ARCH_POST_Z_BOTTOM})"
)

assert _STUB_LEN > 0, (
    f"arch stub length {_STUB_LEN:.1f} mm ≤ 0; arch posts at "
    f"x = ±{ARCH_POST_X} too close to rail D (x = 0)"
)

# Arch post must clear the back rail body (butt joint with CORNER_GAP)
# AND every below-* / back-upper lens cone. After the 2026-05-07 post-
# flip slice the post sits BEHIND the back rail (post Y = +37.3 vs
# back-rail Y = +17) on the back outrigger; pushing the post deeper in
# +Y both keeps it out of every camera's FOV (post inner corner ≥ 76°
# off every lens axis) and outside the platform sweep envelope (the
# separate sweep-clearance assertion below). The Y-separation from
# the back rail is therefore the structural butt-joint condition; the
# FOV consequence and the sweep-clearance consequence both follow.
_FP_EPS = 1e-9
assert ARCH_POST_Y - MEMBER / 2 + _FP_EPS >= SHARED_BACK_RAIL_Y + MEMBER / 2 + CORNER_GAP, (
    f"arch post -Y face ({ARCH_POST_Y - MEMBER/2}) must sit "
    f"≥ CORNER_GAP behind the back rail's +Y face "
    f"({SHARED_BACK_RAIL_Y + MEMBER/2}); post Y = {ARCH_POST_Y} "
    f"is too far forward. The +Y Y-shift behind the back rail keeps "
    f"the post outside every camera FOV cone and outside the "
    f"indexing platform sweep envelope."
)

# Platform sweep clearance. The 4-station turret rotates each platform
# (PLATFORM_ACRYLIC_SIZE square, centred on radial = TURRET_ARM_R)
# through 360° around the hub axis at m03-local (0, -TURRET_ARM_R).
# A platform's outer corners trace a circle of radius
# √((R + P/2)² + (P/2)²) around the hub; the post inner corner
# (closest face of post to the hub) must sit outside that circle plus
# a clearance margin. Pre-flip (post Y = -3.3) this failed: post
# inner corner sat 175.2 mm from hub vs 194.5 mm sweep envelope
# (≈ 19 mm interference). Post-flip (post Y = +55) the post inner
# corner sits ~217 mm from hub, ~22 mm clear.
_SWEEP_R = math.sqrt(
    (TURRET_ARM_R + PLATFORM_ACRYLIC_SIZE / 2) ** 2
    + (PLATFORM_ACRYLIC_SIZE / 2) ** 2
)
_HUB_M03_Y = -TURRET_ARM_R
_POST_INNER_HUB_DIST = math.sqrt(
    (ARCH_POST_X - MEMBER / 2) ** 2
    + (ARCH_POST_Y - MEMBER / 2 - _HUB_M03_Y) ** 2
)
SWEEP_CLEARANCE = 5.0
assert _POST_INNER_HUB_DIST >= _SWEEP_R + SWEEP_CLEARANCE, (
    f"arch post inner corner sits {_POST_INNER_HUB_DIST:.1f} mm from "
    f"the hub axis at m03-local (0, {_HUB_M03_Y}); must be "
    f"≥ {_SWEEP_R + SWEEP_CLEARANCE:.1f} mm (sweep envelope "
    f"{_SWEEP_R:.1f} mm + {SWEEP_CLEARANCE} mm clearance) so the "
    f"indexing turret platforms don't crash the post"
)

# Below-* mount plate clearance. The 50 × 50 below-left/right mount
# plates sit perpendicular to a tilted lens axis; their +Y reach in
# world (= m03-local for the m03 unit at world identity for rig parts)
# overshoots the back rail's +Y face by ~24 mm. The arch post -Y face
# must clear that overshoot plus CORNER_GAP. Bound the +Y reach
# conservatively with the tightest enclosing rectangle: plate centre
# at vertex - 0.5*plate_t*axis, then add (plate_half * sqrt(1-axis_y²)
# + 0.5*plate_t*|axis_y|) along world +Y. Same formula for both
# below-left and below-right since they share |axis_y|.
def _below_plate_y_reach(vertex_pos):
    axis = lens_axis_unit(vertex_pos)
    plate_centre_y = vertex_pos[1] - 0.5 * CAMERA_MOUNT_PLATE_T * axis[1]
    in_plane_y = math.sqrt(max(0.0, 1.0 - axis[1] * axis[1]))
    return (
        plate_centre_y
        + _MIRROR_PLATE_HALF * in_plane_y
        + 0.5 * CAMERA_MOUNT_PLATE_T * abs(axis[1])
    )

_PLATE_Y_REACH = max(
    _below_plate_y_reach(VERTEX_BELOW_LEFT_POS),
    _below_plate_y_reach(VERTEX_BELOW_RIGHT_POS),
)
assert ARCH_POST_Y - MEMBER / 2 >= _PLATE_Y_REACH + CORNER_GAP - _FP_EPS, (
    f"arch post -Y face ({ARCH_POST_Y - MEMBER/2:.3f}) must sit "
    f"≥ CORNER_GAP behind the below-* mount plate's +Y reach "
    f"({_PLATE_Y_REACH:.3f}); the 50×50 plate at the tilted below-* "
    f"vertex projects ~25 mm in world +Y from its centre, well past "
    f"a naive 'just behind the back rail' post placement. Move the "
    f"post further in +Y or shrink/reshape the mount plate."
)

assert _RAIL_D_LEN > 0, (
    f"rail D length {_RAIL_D_LEN:.1f} mm ≤ 0; check rail D Y span "
    f"({T_FRAME_RAIL_D_Y_FRONT}, {T_FRAME_RAIL_D_Y_BACK})"
)

# Front-low / back-upper clamps both sit on rail D's -Z face. The
# clamp body's BASE_L = 60 mm footprint (±30 mm along rail) hangs
# off the rail at each end by design (the clamp's bolt pattern is
# narrower than BASE_L, so the bolts still engage rail D's T-slot);
# the assertions below check the looser convention used by the
# rev-3 rig — rail D extends past each vertex by ≥ RAIL_D_FRONT_
# OVERHANG_MM (5 mm).
assert T_FRAME_RAIL_D_Y_FRONT < VERTEX_FRONT_LOW_POS[1], (
    f"rail D front end (y = {T_FRAME_RAIL_D_Y_FRONT}) must reach past "
    f"the front-low vertex (y = {VERTEX_FRONT_LOW_POS[1]}) so the "
    f"front-low clamp's bolts engage rail D's T-slot"
)
# Back-upper plate-back centre lives ~5 mm forward of the vertex
# along the lens axis; rail D's back face must extend at least to
# the plate-back to host the clamp.
_BU_PLATE_BACK_Y_NOMINAL = VERTEX_BACK_UPPER_POS[1] + 5.0
assert T_FRAME_RAIL_D_Y_BACK > _BU_PLATE_BACK_Y_NOMINAL, (
    f"rail D back end (y = {T_FRAME_RAIL_D_Y_BACK}) must reach past "
    f"the back-upper plate-back (y ≈ {_BU_PLATE_BACK_Y_NOMINAL}) so "
    f"the back-upper clamp's bolts engage rail D's T-slot"
)


# --- Member helpers ----------------------------------------------------

def rail_x(x_centre: float, x_len: float, y: float, z: float) -> Part:
    """X-direction stub (long axis along +X)."""
    return Pos(x_centre, y, z) * Box(x_len, MEMBER, MEMBER)


def rail_y(x: float, y_centre: float, y_len: float, z: float) -> Part:
    """Y-direction rail (long axis along +Y)."""
    return Pos(x, y_centre, z) * Box(MEMBER, y_len, MEMBER)


def rail_z(x: float, y: float, z_centre: float, z_len: float) -> Part:
    """Z-direction post (long axis along +Z)."""
    return Pos(x, y, z_centre) * Box(MEMBER, MEMBER, z_len)


# --- Members -----------------------------------------------------------

_MEMBERS = {
    "arch_post_pos_x": rail_z(+ARCH_POST_X, ARCH_POST_Y, _POST_ZC, _POST_LEN),
    "arch_post_neg_x": rail_z(-ARCH_POST_X, ARCH_POST_Y, _POST_ZC, _POST_LEN),
    "arch_stub_pos_x": rail_x(
        _STUB_XC_POS, _STUB_LEN, ARCH_CROSSBAR_Y, ARCH_CROSSBAR_Z
    ),
    "arch_stub_neg_x": rail_x(
        _STUB_XC_NEG, _STUB_LEN, ARCH_CROSSBAR_Y, ARCH_CROSSBAR_Z
    ),
    "rail_d": rail_y(0.0, _RAIL_D_YC, _RAIL_D_LEN, T_FRAME_RAIL_D_Z),
}

ARCH_PART_NAMES = tuple(_MEMBERS.keys())


# --- Assembly builder --------------------------------------------------

def build_arch() -> Assembly:
    """Arch sub-assembly. Members are in m03-local coordinates (the
    rig is placed at m03 identity, so no separate local frame)."""
    a = Assembly()
    for name, part in _MEMBERS.items():
        a = a.with_part(
            name, part,
            material=material_of(name), color=color_of(name),
        )
    return a


# --- Standalone assembly -----------------------------------------------

# Every pair of members must be non-overlapping — butt joints are
# separated by CORNER_GAP. Printed gussets at the post-to-back-rail
# and post-to-stub corners land in a follow-up slice.
assembly = build_arch().assert_no_interference_within(ARCH_PART_NAMES)


if __name__ == "__main__":
    check(assembly, out="outputs")
