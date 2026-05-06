"""
M03 scanner — arch (Phase-2 integrated frame).

Vertical 2020 posts at the back-rail/side-rail corners that carry
rail D as a Y-axis cantilever centred on the arch crossbar plane.
Successor to the rev-3 rig's rail B + rail C + B↔C corner brace +
the C/D L-junction. Together with the shared back rail (now hosted
in `../frame/`) and the new rail D position, this is the integrated
rig+turret structural frame described in
`../integrated_frame_handoff.md`.

Members (5 total — all stock 2020 aluminum extrusion):
  * arch_post_pos_x — vertical post at m03-local x = +ARCH_POST_X,
                      y = SHARED_BACK_RAIL_Y. The handoff's nominal
                      x = ±145 (= back-rail/side-rail corner) clashes
                      with the below-* clamps' BASE_L footprint along
                      the back rail; posts move inboard to x = ±90 to
                      clear it (see ARCH_POST_X in params.py for the
                      derivation). Bottom face sits CORNER_GAP above
                      the shared back rail's top face; top face flush
                      with rail D's top face.
  * arch_post_neg_x — X-mirror.
  * arch_stub_pos_x — short X-axis stub at z = ARCH_CROSSBAR_Z
                      (= rail D centerline) running from the +X post's
                      inner face to rail D's +X face (CORNER_GAP each).
                      Plays the structural role of the demolished
                      rail C top + L-gusset corner.
  * arch_stub_neg_x — X-mirror.
  * rail_d          — Y-axis cantilever at x = 0 (m03-local) spanning
                      y ∈ [T_FRAME_RAIL_D_Y_FRONT, T_FRAME_RAIL_D_Y_BACK]
                      = [-150, +150]. Front cantilever 167 mm hosts the
                      front-low clamp at y ≈ -150; back cantilever 133
                      mm hosts the back-upper clamp at y ≈ +115.6.

Rail D is a continuous member centred on x = 0; the two arch stubs
butt its east/west faces (CORNER_GAP separator). Posts and rail D are
not co-planar in X (posts at x = ±145, rail D at x = 0), so they
never share volume — only the stubs do the bridging.

The "arch top" detail (full crossbar vs L-brackets vs one-sided
L-arm — see the handoff's `Open in Phase 2` block) is realised here
as **two inward stubs**: a full crossbar would require splitting rail
D into two pieces or stacking the crossbar above rail D, neither of
which is clean. The two-stub layout keeps rail D continuous through
y = +17 and stays out of the below-* camera FOVs (the inward stubs
sit at z = +1188 world, well above the brick centroid at z = +1030).

Internal printed gussets at the post-to-back-rail and post-to-stub
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
    SKELETON_RAIL_MEMBER,
    T_FRAME_RAIL_D_Y_BACK,
    T_FRAME_RAIL_D_Y_FRONT,
    T_FRAME_RAIL_D_Z,
    VERTEX_BACK_UPPER_POS,
    VERTEX_FRONT_LOW_POS,
)
from design_systems import color_of  # noqa: E402
from materials import material_of  # noqa: E402


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

def add_arch(a: Assembly) -> Assembly:
    """Add every arch member in m03-local coordinates."""
    for name, part in _MEMBERS.items():
        a = a.add(
            name, part,
            material=material_of(name), color=color_of(name),
        )
    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_arch(Assembly())

# Every pair of members must be non-overlapping — butt joints are
# separated by CORNER_GAP. Printed gussets at the post-to-back-rail
# and post-to-stub corners land in a follow-up slice.
for _i, _a in enumerate(ARCH_PART_NAMES):
    for _b in ARCH_PART_NAMES[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
