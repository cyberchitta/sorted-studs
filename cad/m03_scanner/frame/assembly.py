"""
M03 scanner — turret floor frame (Phase-2 integrated topology).

Standalone freestanding 2020 floor frame on the m05 deck top face
(world z = +890; turret-local z = -130 after the c7 floor rise), the
mechanical interface for the 4-station turret described in
`_notes/decisions/m03-turret.md` *and* the rig-side anchor for the
camera-rig below-* clamps + arch posts after the Phase-2 integrate
(see `../integrated_frame_handoff.md`).

Members (6 total — option a' dropped the front X-rail in 2026-05-05;
Phase 2 bumped the back rail +20 mm in Y and added the back-side
outriggers; all stock 2020 aluminum extrusion):
  * floor_back_x         — shared back rail at y = TURRET_FRAME_BACK_Y
                           = +142. The new "rail A" of the integrated
                           frame: below-* camera clamps grip its +Z
                           face and the two arch posts (`../arch/`)
                           rise from its +Z face at x = ±145.
  * floor_left_y         — left Y-rail at x = -TURRET_FRAME_SIDE_X.
  * floor_right_y        — right Y-rail at x = +TURRET_FRAME_SIDE_X.
  * floor_cross_x        — center X-rail at y = TURRET_FRAME_CROSS_Y
                           (= hub axis Y = 0 in turret-local). Mounting
                           line shared by the hub bearing (x = 0) and
                           the IDLE/LOAD neighbour-module anchors at
                           x = ±R.
  * floor_outrig_pos_x   — back outrigger at x = +TURRET_FRAME_SIDE_X,
                           cantilevered past the back rail's +Y face to
                           land the m03 unit's support footprint past
                           the arch post line. Bolted to the deck via
                           a printed foot in a follow-up slice.
  * floor_outrig_neg_x   — X-mirror of the +X outrigger.

Topology change in option a' (2026-05-05): the front X-rail at
y = -R (under the new DUMP station at heading 270°) was dropped
because, with the turret placed at world Pos(0, +ARM_R, 0), that
rail would sit at world y = 0 over the diverter top column body and
intercept the brick drop path from DUMP into the diverter column.
Frame is a U-shape closed at the back; side rails cantilever
~115 mm forward of the cross-rail to where the front rail used to
terminate. Acceptable because all 4 platforms hang off the spider
arm at the hub (not the floor frame), the cross-rail still carries
the hub housing, and the back perimeter (back X-rail + 2 side stubs
back of cross) remains rigid.

Phase-2 outriggers (2026-05-06): the rig+turret integrate moves
mass back-of-back-rail (rail D's back cantilever + the back-upper
camera mass at world y = +362) such that the structure's CoM trends
toward the deck edge at world y ≈ +330. Two 2020 outriggers butt
the back rail's +Y face at x = ±TURRET_FRAME_SIDE_X and reach past
the deck edge (turret-local y = +TURRET_FRAME_OUTRIG_Y_BACK = +190,
world y = +315; deck cylinder edge at world y ≈ +296.4 at x=±145).
Outriggers carry the arch posts (now standing **behind** the back
rail at turret-local y = +180 = m03-local y = +55) and host the
deck-foot mounts inboard of mid-span (typically at turret-local
y ≤ +171, world y ≤ +296.4 — at or inside the deck cylinder edge).
Post-flip slice (2026-05-07) grew the outrigger 17.7 → 37.7 mm
along Y so the post body (turret-local y ∈ [+170, +190]) lands
flush on the outrigger top; the post-Y move was driven by below-*
mount plate clearance, not just back-rail butt clearance.

Coordinate frame = turret-local frame (matches `../params.py`):
    origin = hub axis at the cross-rail's top face Z plane
    +X     = operator's right            (parallel to world +X)
    +Y     = away from operator          (parallel to world +Y)
    +Z     = up                          (parallel to world +Z)

Clearance to camera rig: floor rails are coplanar with the rig's
rail A / rail B (both share the z = -220 bench). The back rail
butts rail A's front face with a CORNER_GAP separator after the
unit is placed in world; that cross-unit assertion lives in
`../../assembly.py`. Front-half of the cam ring (around
LOAD/RETURN/DUMP) projects past the camera-rig front face by
design — the open-front operator-access region the rig's rev-3
architecture preserves.

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
    TURRET_FRAME_BACK_Y,
    TURRET_FRAME_CROSS_Y,
    TURRET_FRAME_FLOOR_Z,
    TURRET_FRAME_FRONT_Y,
    TURRET_FRAME_SIDE_X,
)
from design_systems import color_of  # noqa: E402
from materials import material_of  # noqa: E402


# --- Member cross-section ----------------------------------------------

MEMBER = 20.0                   # 2020 extrusion
CORNER_GAP = 0.3                # butt-joint gap


# --- Phase-2 outrigger reach -------------------------------------------
#
# Back-of-back-rail outrigger end Y in turret-local. The deck (m05
# top_acrylic) is a Ø660 disk at world (0, 0); at world x = ±145
# (= turret world x with turret centred on the hub axis) the deck's
# Y boundary is √(330² - 145²) ≈ ±296.4. Outrigger end at turret-
# local +190 = world y = +315 — the outrigger end face overhangs
# the deck edge at x = ±145 by ~18.6 mm. Sized to give the arch post
# (now sitting BEHIND the back rail at m03-local y = +55, post body
# turret-local y ∈ [+170, +190]) a flush landing footprint over its
# entire 20 mm width. The 2026-05-07 post-flip slice grew this
# 170 → 190 in two steps:
#   * 170 → 172.3 (post body originally Y ∈ [+27.3, +47.3] m03-local,
#     just behind the back rail);
#   * 172.3 → 190 (post body grew to Y ∈ [+45, +65] m03-local to
#     clear the below-* mount plate's +Y reach at ~+41.9 m03-local —
#     the binding constraint that the "just behind the back rail"
#     placement missed; see arch/assembly.py's plate-clearance
#     assert).
# The printed deck-foot in a follow-up slice bolts the outrigger
# inboard of mid-span (turret-local Y ≤ +171.2 = world y ≤ +296.4,
# AT the deck cylinder boundary at x = ±145) — typically two bolts
# at e.g. turret-local Y = +160 and +170 (world +285 and +295,
# both well inside). The ~18 mm of outrigger past the deck edge
# is inert: tip-over resistance is set by the bolted foot's
# position, and the m03 unit's mass-inboard restoring moment
# vastly exceeds the small mass-past-deck (rail D back tail +
# arch posts + back-upper camera) tipping moment.
TURRET_FRAME_OUTRIG_Y_BACK = 190.0


# --- Derived dimensions ------------------------------------------------

# Back perimeter X-rail overhangs each side rail's centerline by half
# a member so the rail end face is flush with the side rail's outer
# face. (Front X-rail dropped in option a', 2026-05-05.)
_PERIMETER_X_LEN = 2 * TURRET_FRAME_SIDE_X + MEMBER

# Side Y-rails: back end abuts the back X-rail with a CORNER_GAP
# separator; front end is a free cantilever (no front X-rail to abut)
# at the Y where the front rail used to terminate, so the side rails'
# forward reach is unchanged from the rev-4 closed-rectangle frame.
_SIDE_Y_FRONT = TURRET_FRAME_FRONT_Y + MEMBER / 2 + CORNER_GAP
_SIDE_Y_BACK = TURRET_FRAME_BACK_Y - MEMBER / 2 - CORNER_GAP
_SIDE_Y_LEN = _SIDE_Y_BACK - _SIDE_Y_FRONT
_SIDE_Y_CENTER = (_SIDE_Y_FRONT + _SIDE_Y_BACK) / 2

# Outriggers: butt the back rail's +Y face with a CORNER_GAP separator
# and run back to TURRET_FRAME_OUTRIG_Y_BACK. Anchored at x = ±SIDE_X
# so they line up with the side rails on the back rail's other face,
# making each (side-rail end / outrigger / back rail) joint a single-
# X column the printed deck-foot can clamp.
_OUTRIG_Y_FRONT = TURRET_FRAME_BACK_Y + MEMBER / 2 + CORNER_GAP
_OUTRIG_Y_BACK = TURRET_FRAME_OUTRIG_Y_BACK
_OUTRIG_Y_LEN = _OUTRIG_Y_BACK - _OUTRIG_Y_FRONT
_OUTRIG_Y_CENTER = (_OUTRIG_Y_FRONT + _OUTRIG_Y_BACK) / 2

# Center cross X-rail fits between the side rails (all rails share
# z = -220 so the cross can't pass through them — same constraint
# the side rails apply to the front/back X-rails).
_CROSS_X_LEFT = -TURRET_FRAME_SIDE_X + MEMBER / 2 + CORNER_GAP
_CROSS_X_RIGHT = +TURRET_FRAME_SIDE_X - MEMBER / 2 - CORNER_GAP
_CROSS_X_LEN = _CROSS_X_RIGHT - _CROSS_X_LEFT


# --- Sanity asserts ----------------------------------------------------

assert _SIDE_Y_LEN > 0, (
    f"side rail length {_SIDE_Y_LEN:.1f} mm ≤ 0; front "
    f"({TURRET_FRAME_FRONT_Y}) must sit forward of back "
    f"({TURRET_FRAME_BACK_Y}) by more than MEMBER + 2·GAP"
)

assert _OUTRIG_Y_LEN > 0, (
    f"outrigger length {_OUTRIG_Y_LEN:.1f} mm ≤ 0; outrigger end "
    f"({TURRET_FRAME_OUTRIG_Y_BACK}) must sit aft of back rail "
    f"+Y face by more than MEMBER + 2·GAP"
)

# Cross-rail Y must lie strictly between front and back rail Y so it
# can fit between the side rails as a third X-rail.
assert TURRET_FRAME_FRONT_Y + MEMBER / 2 < TURRET_FRAME_CROSS_Y < TURRET_FRAME_BACK_Y - MEMBER / 2, (
    f"cross-rail Y ({TURRET_FRAME_CROSS_Y}) must sit strictly between "
    f"front ({TURRET_FRAME_FRONT_Y}) and back ({TURRET_FRAME_BACK_Y}) "
    f"with at least MEMBER/2 clearance to each"
)

# Back rail back face Z-clearance vs the camera rig's rail A front
# face is a cross-unit boundary asserted in `../../assembly.py` after
# the turret is placed in world. Within the unit we only check the
# self-consistency of the frame geometry.


# --- Member helpers ----------------------------------------------------

def rail_x(x_centre: float, x_len: float, y: float, z: float) -> Part:
    """X-direction rail (long axis along world +X)."""
    return Pos(x_centre, y, z) * Box(x_len, MEMBER, MEMBER)


def rail_y(x: float, y_centre: float, y_len: float, z: float) -> Part:
    """Y-direction rail (long axis along world +Y, depth)."""
    return Pos(x, y_centre, z) * Box(MEMBER, y_len, MEMBER)


# --- Members -----------------------------------------------------------

_MEMBERS = {
    "floor_back_x": rail_x(
        0.0, _PERIMETER_X_LEN, TURRET_FRAME_BACK_Y, TURRET_FRAME_FLOOR_Z
    ),
    "floor_left_y": rail_y(
        -TURRET_FRAME_SIDE_X, _SIDE_Y_CENTER, _SIDE_Y_LEN, TURRET_FRAME_FLOOR_Z
    ),
    "floor_right_y": rail_y(
        +TURRET_FRAME_SIDE_X, _SIDE_Y_CENTER, _SIDE_Y_LEN, TURRET_FRAME_FLOOR_Z
    ),
    "floor_cross_x": rail_x(
        0.0, _CROSS_X_LEN, TURRET_FRAME_CROSS_Y, TURRET_FRAME_FLOOR_Z
    ),
    "floor_outrig_pos_x": rail_y(
        +TURRET_FRAME_SIDE_X, _OUTRIG_Y_CENTER, _OUTRIG_Y_LEN,
        TURRET_FRAME_FLOOR_Z,
    ),
    "floor_outrig_neg_x": rail_y(
        -TURRET_FRAME_SIDE_X, _OUTRIG_Y_CENTER, _OUTRIG_Y_LEN,
        TURRET_FRAME_FLOOR_Z,
    ),
}

TURRET_FRAME_PART_NAMES = tuple(_MEMBERS.keys())


# --- Assembly builder --------------------------------------------------

def build_turret_frame() -> Assembly:
    """2020 turret-frame members sub-assembly, in turret-local
    coordinates; the caller places it. Exposed so the top-level
    scanner assembly composes via the public surface instead of
    reaching into module-private state.
    """
    a = Assembly()
    for name, part in _MEMBERS.items():
        a = a.with_part(
            name, part,
            material=material_of(name), color=color_of(name),
        )
    return a


# --- Standalone assembly -----------------------------------------------

# Every pair of members must be non-overlapping. Butt-joints are
# separated by CORNER_GAP (0.3 mm); printed corner brackets bridge the
# gaps in reality and are not modelled in this slice.
assembly = build_turret_frame().assert_no_interference_within(
    TURRET_FRAME_PART_NAMES
)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Members are stock 2020 aluminum extrusion — no per-part
    # printability pass.
