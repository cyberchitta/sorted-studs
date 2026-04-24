"""
M03 scanner — frame skeleton sub-assembly (rev 1, 2020-extrusion cube).

Successor to the rev 1 six-panel enclosure (`../enclosure/`) as the
scanner's primary structural chassis. Twelve 2020 T-slot aluminum
extrusions form a rectangular parallelepiped sized to envelop every
currently placed mount (below-mirror brackets, back-upper wall bracket,
camera bracket + adapter, load-cell standoffs, platform). Mounts will
migrate in rev 2 from panel-bolted wedges to extrusion-clamped tabs —
see `../NEXT_STEPS.md`.

Why extrusion instead of printed rods:
  * stock material, stiffer per unit mass than PLA;
  * T-slot lets clamp positions slide freely along each member without
    re-printing the part;
  * corners join with stock L-brackets (not modelled at rev 1; they
    live in the slot and don't affect cross-boundary interference).

Why a plain 12-edge cube at rev 1 (vs. the minimal 6-member variant
the spec sketch suggested — bottom rectangle plus two posts at front/
back mid-edges):
  * Below-mirror mounts sit at (x ≈ ±207, y ≈ -145, z ≈ 28); the
    back-upper at (0, 180, 180); the camera at (0, 108, -221). None of
    these align with a plain 4-corner rectangle's vertices, so the
    minimal variant forces long cantilever tabs from corners to mount
    planes anyway.
  * The full cube lets rev 2 add *intermediate* cross-members at the
    mount planes (e.g. a top-back transverse at z ≈ 180 for the back-
    upper, a top-front transverse at z ≈ -221 for the camera, a mid-
    height front transverse at y ≈ -10 for the platform-hinge pillow
    blocks); each clamp ends up with a short tab rather than a long
    reach. Members we find redundant after the mount clamps land can
    be deleted in a single rev-2 trim pass.

Coordinate frame = world frame of the scanner (matches `../params.py`):
    origin = brick centroid on the platform top face
    +X     = operator's right
    +Y     = up
    +Z     = away from operator (toward the back of the machine)

Frame member layout:
    4 vertical posts at (x_sign·X_POST, *, z_sign·Z_POST) for
        x_sign, z_sign ∈ {-1, +1}
    4 floor rails at y = Y_FLOOR_RAIL (two along X, two along Z)
    4 ceiling rails at y = Y_CEILING_RAIL (two along X, two along Z)
    12 members total; no diagonal bracing at rev 1

All members are 20×20 mm stock — bought, not printed, so no `inspect()`
calls. Rails end CORNER_GAP (0.3 mm) inboard of each post's inner face
so the cross-boundary interference check reads clean zero-volume joints
(the stock L-brackets fill the gap in reality).

What's deferred to rev 2:
  * Intermediate cross-members at each mount plane (see above).
  * Per-mount clamp parts (split-ring clamp on the extrusion + angled
    tab carrying the existing rev-1 four-corner mount-plate bolt
    pattern). Replaces the below-mirror floor brackets, back-upper
    wall bracket, and camera-bracket adapter — three printed parts
    collapse to three (much smaller) clamps.
  * Front-edge platform hinge: pillow blocks on a mid-height front
    cross-member, front-edge pivot so the swing dumps forward toward
    the Module 5 diverter. Load cells (2×) at the hinge axis; solenoid
    or servo pin at the back edge holds the platform level until
    released. Drops the three-point load-cell tripod.
  * Chute entry opening (back, upper) and eject opening (front, lower)
    become empty regions of the frame — no enclosure panel means no
    aperture to cut.
  * Opaque shroud (fabric or printed panels) over the frame when
    imaging needs ambient-light rejection. Shroud is structural-
    neutral; the frame carries all loads.
  * Integration into the top-level `../assembly.py` (which currently
    composes the six-panel enclosure + every mount). The skeleton
    replaces the enclosure; the mount wedges/adapters stay until
    rev-2 clamps land, then they come out too.
"""

from build123d import Box, Part, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

# Standalone-run import: params lives one directory up. Matches
# `../enclosure/assembly.py`, `../below_mirror_floor_bracket/assembly.py`, etc.
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    ENCLOSURE_INNER_X_MAX,
    ENCLOSURE_INNER_Y_MAX,
    ENCLOSURE_INNER_Y_MIN,
    ENCLOSURE_INNER_Z_MAX,
    ENCLOSURE_INNER_Z_MIN,
)


# --- Skeleton dimensions ------------------------------------------------
#
# 20×20 mm 2020 T-slot aluminum extrusion. Outer face of each corner
# post sits at the listed outer bound; member centerline sits MEMBER/2
# inboard. The cube bounds envelop every existing placed sub-assembly:
#
# - Below-mirror bracket footprint in world (120 × 90 base, heading-
#   rotated by ~61° about +Y) reaches outboard corner at |x| ≈ 275.4;
#   post inner face at |x| = X_POST - MEMBER/2 = 280 clears by ~4.6 mm.
#   Tight; noted as a rev-2 trim target since the bracket is going
#   away when the clamp replaces it.
# - Camera bracket + adapter (at the current front wall, z ≈ -295)
#   extends to z ≈ -297; post inner face at z = -(Z_POST - MEMBER/2)
#   = -280 is ~17 mm forward of the adapter's front face — the
#   adapter/bracket will be re-placed onto a rev-2 top-front cross-
#   member inside this envelope.
# - Back-upper wall bracket (at the current back wall, z ≈ +295)
#   extends to z ≈ +297; same relocation story on the +Z side.
# - Load-cell standoffs at radius 220 reach |x| ≤ 191, |z| ≤ 220; well
#   inside the posts regardless.
# - Back-upper mirror face max world-Y ≈ 195; ceiling rail bottom face
#   at y = Y_CEILING_RAIL - MEMBER/2 = 320 clears by 125 mm.

MEMBER = 20.0                   # 2020 extrusion cross-section
CORNER_GAP = 0.3                # MOUNT_GAP convention reused at rail ends

# Outer faces of the four corner posts.
X_OUTER = 300.0
Z_OUTER = 300.0
Y_BOT_OUTER = -240.0            # matches the current enclosure floor
Y_TOP_OUTER = +340.0            # 50 mm above the current enclosure ceiling

# Post centerlines (MEMBER/2 inboard of outer faces).
X_POST = X_OUTER - MEMBER / 2                   # 290
Z_POST = Z_OUTER - MEMBER / 2                   # 290

# Rail centerlines (floor / ceiling, MEMBER/2 inboard of outer faces).
Y_FLOOR_RAIL = Y_BOT_OUTER + MEMBER / 2         # -230
Y_CEILING_RAIL = Y_TOP_OUTER - MEMBER / 2       # +330

# Post geometry. Posts span the full outer Y extent (so rails butt
# against post sides, not stack on post tops).
POST_LEN = Y_TOP_OUTER - Y_BOT_OUTER            # 580
Y_POST_CENTER = (Y_BOT_OUTER + Y_TOP_OUTER) / 2  # +50

# Rail length. Rails sit between posts and end CORNER_GAP inboard of
# each post's inner face. Inner face at |X_POST - MEMBER/2| = X_OUTER -
# MEMBER = 280; rail endpoint at 280 - CORNER_GAP.
RAIL_LEN_X = 2 * (X_OUTER - MEMBER) - 2 * CORNER_GAP   # 2*280 - 0.6 = 559.4
RAIL_LEN_Z = 2 * (Z_OUTER - MEMBER) - 2 * CORNER_GAP   # 559.4


# --- Parametric sanity --------------------------------------------------

assert X_POST - MEMBER / 2 > ENCLOSURE_INNER_X_MAX - MEMBER, (
    f"post inner face at |x|={X_POST - MEMBER / 2} is not outboard of "
    f"the existing interior X-extent {ENCLOSURE_INNER_X_MAX}; mounts "
    f"would foul the frame"
)
assert Y_BOT_OUTER <= ENCLOSURE_INNER_Y_MIN, (
    f"floor rail outer face at y={Y_BOT_OUTER} rises above the current "
    f"interior floor y={ENCLOSURE_INNER_Y_MIN}; floor-mounted parts "
    f"(load-cell standoffs, below-mirror brackets) would collide"
)
assert Y_TOP_OUTER >= ENCLOSURE_INNER_Y_MAX, (
    f"ceiling rail outer face at y={Y_TOP_OUTER} drops below the "
    f"current interior ceiling y={ENCLOSURE_INNER_Y_MAX}"
)
assert Z_POST - MEMBER / 2 > -ENCLOSURE_INNER_Z_MIN - MEMBER, (
    f"post inner face at |z|={Z_POST - MEMBER / 2} is not outboard of "
    f"the existing interior Z-extent {ENCLOSURE_INNER_Z_MIN}..{ENCLOSURE_INNER_Z_MAX}"
)
assert RAIL_LEN_X > 0 and RAIL_LEN_Z > 0 and POST_LEN > 0, (
    "skeleton dimensions degenerate"
)


# --- Member construction ------------------------------------------------

def post(x_sign: int, z_sign: int) -> Part:
    """One of four corner posts. `x_sign`, `z_sign` ∈ {-1, +1} pick the
    corner; centerline at (x_sign·X_POST, Y_POST_CENTER, z_sign·Z_POST),
    cross-section MEMBER × MEMBER, length POST_LEN along world Y."""
    return (
        Pos(x_sign * X_POST, Y_POST_CENTER, z_sign * Z_POST)
        * Box(MEMBER, POST_LEN, MEMBER)
    )


def rail_x(y_center: float, z_sign: int) -> Part:
    """X-direction horizontal rail at `y_center`, z side `z_sign` ∈
    {-1, +1}. Spans from the left-post inner face to the right-post
    inner face with a CORNER_GAP at each end (length RAIL_LEN_X);
    cross-section MEMBER × MEMBER."""
    return (
        Pos(0.0, y_center, z_sign * Z_POST)
        * Box(RAIL_LEN_X, MEMBER, MEMBER)
    )


def rail_z(y_center: float, x_sign: int) -> Part:
    """Z-direction horizontal rail at `y_center`, x side `x_sign` ∈
    {-1, +1}. Spans from the front-post inner face to the back-post
    inner face with a CORNER_GAP at each end (length RAIL_LEN_Z);
    cross-section MEMBER × MEMBER."""
    return (
        Pos(x_sign * X_POST, y_center, 0.0)
        * Box(MEMBER, MEMBER, RAIL_LEN_Z)
    )


# --- Assembly -----------------------------------------------------------

_MEMBERS = {
    # Four corner posts.
    "post_fl": post(-1, -1),   # front-left
    "post_fr": post(+1, -1),   # front-right
    "post_bl": post(-1, +1),   # back-left
    "post_br": post(+1, +1),   # back-right
    # Four floor rails (y = Y_FLOOR_RAIL).
    "rail_floor_front": rail_x(Y_FLOOR_RAIL, -1),
    "rail_floor_back":  rail_x(Y_FLOOR_RAIL, +1),
    "rail_floor_left":  rail_z(Y_FLOOR_RAIL, -1),
    "rail_floor_right": rail_z(Y_FLOOR_RAIL, +1),
    # Four ceiling rails (y = Y_CEILING_RAIL).
    "rail_ceil_front":  rail_x(Y_CEILING_RAIL, -1),
    "rail_ceil_back":   rail_x(Y_CEILING_RAIL, +1),
    "rail_ceil_left":   rail_z(Y_CEILING_RAIL, -1),
    "rail_ceil_right":  rail_z(Y_CEILING_RAIL, +1),
}

assembly = Assembly()
for _name, _part in _MEMBERS.items():
    assembly = assembly.add(_name, _part)

# All 12 member pairs must be non-overlapping. Rail ends sit CORNER_GAP
# inboard of each post's inner face — no volumetric overlap at joints
# (stock L-brackets fill the gap in reality and are not modelled here).
_names = list(_MEMBERS.keys())
for _i, _a in enumerate(_names):
    for _b in _names[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Members are stock 2020 aluminum extrusion — bought, not printed.
    # No `inspect()` calls; the per-part printability pass doesn't apply.
