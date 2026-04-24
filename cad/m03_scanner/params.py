"""
M03 scanner — shared world-frame constants and placement helpers.

The world frame is the frame of the top-level assembly (`assembly.py`)
and of the `enclosure` sub-assembly:

    origin = brick centroid at the scan position, on the platform top face
    +X     = operator's right
    +Y     = up
    +Z     = away from operator (toward the back of the machine)

Each sub-assembly (`platform/`, `camera_bracket/`, `mirror_mount/`,
`mirror_mount_kinematic/`) carries its own local frame; the top-level
assembly composes each into world space using `Pos(...)` and a rotation
— either `Rot(...)` directly or `rotation_for_normal(...)` for the
mirrors where the world normal drives the rotation.

Position and normal values are copied from
`docs/m03-multi-view-scanner.md` (derived in
`klods-syn/nbs/200_viewpoints.md` at tilt 65°, phase 90°,
camera_dist 250 mm). Update this file and the reference doc together
whenever any value changes.
"""

import math

from build123d import Rot


# --- Camera (direct viewpoint) ------------------------------------------

# Camera PCB rear face in world. This is the plane where the
# camera_bracket's local origin lands.
CAMERA_POS = (0.0, 105.7, -226.6)

# Pure X rotation placing the camera_bracket: local +Z (lens axis) maps
# to the unit vector from camera toward brick origin
# ≈ (0, -sin(25°), cos(25°)). Rot(25, 0, 0) delivers this.
CAMERA_ROT_DEG = (25.0, 0.0, 0.0)


# --- Camera bracket ⇄ adapter interface ---------------------------------
#
# External interface of `camera_bracket/` that the adjacent
# `camera_bracket_adapter/` must agree on. Each of these duplicates an
# internal constant in camera_bracket/assembly.py; change either side
# and the top-level cross-boundary assertion will catch the drift.

# Square plate dimension (PLATE_W == PLATE_H in camera_bracket).
CAMERA_BRACKET_PLATE_SIZE = 68.0

# Plate thickness along the bracket's local Z.
CAMERA_BRACKET_PLATE_T = 5.0

# Corner-bolt spacing (PLATE_W - 2·BOLT_INSET in camera_bracket, with
# BOLT_INSET = 8.0). Same 52 mm square pattern is reused on the
# adapter's tilted front face so four bolts clamp bracket→adapter with
# a single matched hole pattern.
CAMERA_BRACKET_BOLT_SPACING = 52.0


# --- Camera bracket adapter ⇄ enclosure front wall interface ------------
#
# External interface of `camera_bracket_adapter/` that the enclosure's
# `wall_front` panel must agree on. Four M4 wall bolts drive through
# the front panel from outside the enclosure into the adapter's back
# face; the wall carries a matched through-hole pattern so the bolt
# axes are coaxial with the adapter's holes. Duplicates an internal
# constant in camera_bracket_adapter/assembly.py; changing either side
# without the other surfaces as misalignment at first assembly.

# Full XY spacing of the adapter's wall-bolt square pattern (18 mm
# half-spacing in the adapter's module → 36 mm centre-to-centre
# between adjacent bolts).
ADAPTER_WALL_BOLT_SPACING = 36.0

# Through-hole clearance diameter for the M4 wall bolts. Same 4.5 mm
# used by every M4 clearance hole in the scanner (mirror mount,
# camera bracket, adapter bracket-bolt group).
ADAPTER_WALL_BOLT_HOLE_D = 4.5


# --- Mirrors ------------------------------------------------------------

# Physical mirror centres in world (centre of reflective face).
MIRROR_BACK_UPPER_POS = (0.0, 176.0, 172.7)
MIRROR_BELOW_LEFT_POS = (-201.4, -140.1, 25.4)
MIRROR_BELOW_RIGHT_POS = (201.4, -140.1, 25.4)

# Mirror outward normals (unit). The reflective face normal points
# along +normal toward the camera/brick; each mirror_mount's local +Z
# maps onto this direction under `rotation_for_normal(...)`.
MIRROR_BACK_UPPER_NORMAL = (0.0, -0.466, -0.885)
MIRROR_BELOW_LEFT_NORMAL = (0.690, 0.616, -0.380)
MIRROR_BELOW_RIGHT_NORMAL = (-0.690, 0.616, -0.380)


# --- Mirror mount ⇄ floor bracket interface -----------------------------
#
# External interface of `mirror_mount/` that the adjacent
# `below_mirror_floor_bracket/` must agree on. Each of these duplicates
# an internal constant in mirror_mount/assembly.py; change either side
# and the top-level cross-boundary tangent-contact assertion will catch
# the drift.

# Mirror glass thickness.
MIRROR_MOUNT_MIRROR_T = 3.0

# Printed plate thickness (rigid rev 1).
MIRROR_MOUNT_PLATE_T = 5.0

# Offset from mirror reflective face to plate back face along mount-
# local -Z (= outward-normal direction reversed). Bracket top face
# lands on this plane.
MIRROR_MOUNT_BACK_OFFSET = MIRROR_MOUNT_MIRROR_T + MIRROR_MOUNT_PLATE_T

# Square plate size (PLATE_W == PLATE_H in mirror_mount).
MIRROR_MOUNT_PLATE_SIZE = 110.0

# Corner-bolt spacing (PLATE_SIZE - 2·BOLT_INSET in mirror_mount, with
# BOLT_INSET = 8.0). Same 94 mm square pattern is reused on the floor
# bracket's top face so four bolts clamp mount → bracket with a single
# matched hole pattern.
MIRROR_MOUNT_BOLT_SPACING = 94.0


def rotation_for_normal(
    normal: tuple[float, float, float],
    roll_deg: float = 0.0,
) -> Rot:
    """Rotation that carries a sub-assembly's local +Z onto `normal`.

    Used to orient each mirror_mount: the mount's local +Z is the
    mirror outward normal, and this returns the `Rot(...)` that places
    that direction correctly in world space.

    Roll about the target normal is unconstrained for the square mirror
    + threefold adjuster triangle at this rev. `roll_deg` sets the
    `Rz` component of the Euler triple; callers that need mount-local
    +X to fall on a specific world direction (e.g. the horizontal
    bracket-local tilt axis for the below-mirror floor brackets)
    supply the value computed by `horizontal_x_roll_deg(normal)`.
    """
    nx, ny, nz = normal
    n_mag = math.sqrt(nx * nx + ny * ny + nz * nz)
    nx, ny, nz = nx / n_mag, ny / n_mag, nz / n_mag

    # build123d's `Rot(a, b, c)` composes as Rx(a) · Ry(b) · Rz(c)
    # when applied to a point. With Rz(0) = I:
    #   Rot(a, b, 0) · (0, 0, 1) = Rx(a) · Ry(b) · (0, 0, 1)
    #                            = (sin b, -sin a · cos b, cos a · cos b)
    # Matching component-wise to (nx, ny, nz):
    #   sin b = nx           →  b = asin(nx)
    #   tan a = -ny / nz     →  a = atan2(-ny, nz)   (for cos b > 0)
    beta = math.degrees(math.asin(nx))
    cb = math.cos(math.radians(beta))
    if abs(cb) < 1e-9:
        # Degenerate (local +Z maps onto ±X): any alpha works.
        alpha = 0.0
    else:
        alpha = math.degrees(math.atan2(-ny / cb, nz / cb))
    return Rot(alpha, beta, roll_deg)


def horizontal_x_roll_deg(normal: tuple[float, float, float]) -> float:
    """Roll (`Rz` component) that sends mount-local +X onto a
    world-horizontal direction after `rotation_for_normal(normal)`
    placement. The resulting mount-local +X lies in the plane
    perpendicular to the normal and has zero world-Y component — it
    aligns with the horizontal "tilt axis" of a bracket whose frame
    is `Rot(0, heading, 0)` for this normal's heading.

    Derivation: full rotation matrix is Rx(a)·Ry(b)·Rz(c) per
    `Rot(a,b,c)` semantics. Mount-local +X lands at world
    `(cos b · cos c, cos a · sin c + sin a · sin b · cos c,
      sin a · sin c − cos a · sin b · cos c)`.
    Setting the Y component to zero:
        cos a · sin c + sin a · sin b · cos c = 0
      ⇒ tan c = -sin a · sin b / cos a
    Pick the root with `cos c > 0` (so `R_11 = cos b · cos c` keeps
    the sign of `cos b`, i.e. mount +X stays on the +X side of the
    horizontal plane for normals with +Z component, matching the
    bracket's +X by convention).
    """
    nx, ny, nz = normal
    n_mag = math.sqrt(nx * nx + ny * ny + nz * nz)
    nx, ny, nz = nx / n_mag, ny / n_mag, nz / n_mag

    beta = math.asin(nx)
    cb = math.cos(beta)
    sb = math.sin(beta)
    if abs(cb) < 1e-9:
        return 0.0
    alpha = math.atan2(-ny / cb, nz / cb)
    sa = math.sin(alpha)
    ca = math.cos(alpha)

    # Root with cos c > 0. If ca > 0, setting sc = -sa·sb, cc = ca
    # (unscaled) gives the desired ratio; normalise signs so cc > 0.
    num = -sa * sb
    den = ca
    if den < 0:
        num, den = -num, -den
    return math.degrees(math.atan2(num, den))


# --- Enclosure envelope -------------------------------------------------

# Interior bounds for the rev 1 enclosure, sized to clear the plate
# corners of every placed sub-assembly with ≥10 mm margin. Worst case
# is a below-mirror rigid plate (110×110×5, ~78 mm half-diagonal)
# centred at (±201.4, -140.1, 25.4) — a corner can reach x ≈ ±280 and
# y ≈ -218.
ENCLOSURE_INNER_X_MIN = -295.0
ENCLOSURE_INNER_X_MAX = 295.0
ENCLOSURE_INNER_Y_MIN = -240.0
ENCLOSURE_INNER_Y_MAX = 290.0
ENCLOSURE_INNER_Z_MIN = -290.0
ENCLOSURE_INNER_Z_MAX = 260.0

# Printed panel thickness (uniform across all six panels at rev 1).
ENCLOSURE_WALL_T = 5.0

# FDM printability floor — matches the sub-assembly convention.
WALL_MIN = 1.5


# --- Below-mirror floor bracket placement (rev 1) -----------------------
#
# Each below-mirror sits tilted up-and-inward on a printed wedge whose
# base rests on the enclosure floor. Both brackets share one part
# design (`below_mirror_floor_bracket/`); they're placed at two world
# locations related by reflection across the YZ plane.
#
# Bracket local frame (see below_mirror_floor_bracket/assembly.py):
#     origin : on floor (y = ENCLOSURE_INNER_Y_MIN), directly under
#              the mount back face centre.
#     +Y     : world +Y (up).
#     +Z     : horizontal projection of back-face outward-normal
#              (= horizontal projection of -mirror_normal); the top
#              face rises along +Z.
#     +X     : +Y × +Z (horizontal, orthogonal to +Z).
#
# The bracket is reflection-symmetric in its local X, so below-left and
# below-right instances differ only in the Ry(heading) placement
# rotation, not in the part geometry.


def _back_face_centre_world(pos, normal, offset=MIRROR_MOUNT_BACK_OFFSET):
    """Mount plate back face centre in world: mirror_pos - offset·normal."""
    return (pos[0] - offset * normal[0],
            pos[1] - offset * normal[1],
            pos[2] - offset * normal[2])


def _bracket_heading_deg(normal):
    """Heading angle that Rot(0, heading, 0) needs so bracket-local +Z
    lands on the horizontal projection of -normal (= back-face normal)
    in world. Derived from (sin θ, 0, cos θ) = (-nx, 0, -nz)/|…|."""
    return math.degrees(math.atan2(-normal[0], -normal[2]))


# Both below-mirrors share elevation above horizontal (by the tetrahedral
# optimiser's threefold symmetry); the bracket top-face tilt and height
# derive from this common value.
_BL_NY = MIRROR_BELOW_LEFT_NORMAL[1]
_BR_NY = MIRROR_BELOW_RIGHT_NORMAL[1]
assert abs(_BL_NY - _BR_NY) < 1e-6, (
    "below-mirrors have drifted apart in elevation; floor bracket "
    "geometry assumes they share a single tilt angle"
)
BELOW_MIRROR_ELEV_DEG = math.degrees(math.asin(_BL_NY))

_BL_BACK = _back_face_centre_world(
    MIRROR_BELOW_LEFT_POS, MIRROR_BELOW_LEFT_NORMAL
)
_BR_BACK = _back_face_centre_world(
    MIRROR_BELOW_RIGHT_POS, MIRROR_BELOW_RIGHT_NORMAL
)

# Bracket local origin in world = (back_face_x, floor_y, back_face_z).
BELOW_MIRROR_BRACKET_LEFT_POS = (
    _BL_BACK[0], ENCLOSURE_INNER_Y_MIN, _BL_BACK[2]
)
BELOW_MIRROR_BRACKET_RIGHT_POS = (
    _BR_BACK[0], ENCLOSURE_INNER_Y_MIN, _BR_BACK[2]
)

# Mount back face centre height above floor. Bolt hole axes pass
# through this plane.
BELOW_MIRROR_BRACKET_MOUNT_PLANE_HEIGHT = _BL_BACK[1] - ENCLOSURE_INNER_Y_MIN
assert abs(
    (_BR_BACK[1] - ENCLOSURE_INNER_Y_MIN)
    - BELOW_MIRROR_BRACKET_MOUNT_PLANE_HEIGHT
) < 1e-6, (
    "below-mirrors have drifted apart in height; floor bracket "
    "geometry assumes a single mount-plane height"
)

# Assembly clearance between the bracket's top face and the mount
# plate's back face, measured along bracket-local +Y. Two purposes:
#   * separates coplanar faces so the cross-boundary boolean
#     intersection computes a clean zero volume (coplanar tangent
#     contact otherwise picks up ~4 mm³ of numerical noise that
#     trips `assert_no_interference`'s 0.001 mm³ threshold).
#   * absorbs print-dimension drift + surface-finish stack on both
#     the bracket top and the mount plate back face.
# Bolt-hole centres in the bracket remain on the mount bolts' world
# axes (modelled at MOUNT_PLANE_HEIGHT, not top-face height); the
# resulting lateral offset from bracket hole edge to bolt axis is
# well inside the M4 clearance (≤ 0.25 mm for GAP ≤ 0.3).
BELOW_MIRROR_BRACKET_MOUNT_GAP = 0.3

# Bracket's physical top-face height — the cut plane sits this much
# below the mount plate's back face.
BELOW_MIRROR_BRACKET_TOP_HEIGHT = (
    BELOW_MIRROR_BRACKET_MOUNT_PLANE_HEIGHT - BELOW_MIRROR_BRACKET_MOUNT_GAP
)

# Heading angles for the two placements.
BELOW_MIRROR_BRACKET_LEFT_HEADING_DEG = _bracket_heading_deg(
    MIRROR_BELOW_LEFT_NORMAL
)
BELOW_MIRROR_BRACKET_RIGHT_HEADING_DEG = _bracket_heading_deg(
    MIRROR_BELOW_RIGHT_NORMAL
)


# --- Back-upper wall bracket placement (rev 1) --------------------------
#
# The back-upper mirror's back-face outward-normal points up-and-back
# (world +Y +Z), with the +Z component (0.885) dominant over the +Y
# component (0.466). The natural support wall is therefore the back
# wall at z = ENCLOSURE_INNER_Z_MAX; the mount plate's back face sits
# ~80 mm inboard of the back wall along world +Z.
#
# Bracket local frame (see back_upper_wall_bracket/assembly.py):
#     origin : on back-wall inner face (z = ENCLOSURE_INNER_Z_MAX),
#              at the world-X and world-Y of the mount back face
#              centre.
#     +Y     : world -Z (into the enclosure from the back wall;
#              analogous to "up from floor" in the below-mirror
#              bracket's frame).
#     +Z     : horizontal projection of back-face outward-normal onto
#              the back-wall plane, normalised. For back-upper whose
#              nx = 0 this is world +Y (straight up in world terms).
#     +X     : +Y × +Z = world +X (horizontal tilt axis along the wall
#              surface).
#
# Placement rotation in the top-level assembly is Rot(-90, 0, 0): the
# pure Rx(-90) carries bracket-local +Y onto world -Z and bracket-
# local +Z onto world +Y. No heading rotation is needed because the
# back-upper is x-symmetric (nx = 0); the bracket itself is also
# symmetric in its local X, so both halves of the back-upper's mount
# plate land correctly under one placement.

# Back-upper mirror back face centre in world (mirror face centre
# offset by MIRROR_MOUNT_BACK_OFFSET along -mirror_normal).
_BU_BACK_CENTRE = _back_face_centre_world(
    MIRROR_BACK_UPPER_POS, MIRROR_BACK_UPPER_NORMAL
)

# The back-upper must sit on the YZ plane for the symmetric, heading-
# free placement to be valid. Fail loudly if the viewpoint optimiser
# drifts it off-axis.
assert abs(MIRROR_BACK_UPPER_NORMAL[0]) < 1e-6, (
    "back-upper mirror has drifted off the YZ plane (nx != 0); the "
    "back-upper wall bracket assumes the x-symmetric (heading = 0) "
    "pose"
)

# Elevation of the mirror normal above the back-wall inward-normal
# (world -Z). Mirror-normal component along -Z = -nz; for the back-
# upper this is 0.885, giving ELEV ≈ 62.2° and a complementary top-
# face tilt ≈ 27.8° — much gentler than the below-mirror's 52°.
BACK_UPPER_ELEV_DEG = math.degrees(math.asin(-MIRROR_BACK_UPPER_NORMAL[2]))

# Bracket local origin in world. x/y match the mount back face centre;
# z sits on the back-wall inner face.
BACK_UPPER_BRACKET_POS = (
    _BU_BACK_CENTRE[0], _BU_BACK_CENTRE[1], ENCLOSURE_INNER_Z_MAX
)

# Distance from back-wall inner face to the mount plate back face
# centre along world +Z (= bracket-local +Y).
BACK_UPPER_BRACKET_MOUNT_PLANE_HEIGHT = (
    ENCLOSURE_INNER_Z_MAX - _BU_BACK_CENTRE[2]
)

# MOUNT_GAP rationale matches the below-mirror floor bracket — see the
# comment on BELOW_MIRROR_BRACKET_MOUNT_GAP above. The gap is also
# large enough to absorb the lateral offset between the bracket hole
# centre (on the top-face plane) and the mount bolt axis (through the
# mount plate at mount_plane_height); widened bracket hole diameter
# covers the residual.
BACK_UPPER_BRACKET_MOUNT_GAP = 0.3

BACK_UPPER_BRACKET_TOP_HEIGHT = (
    BACK_UPPER_BRACKET_MOUNT_PLANE_HEIGHT - BACK_UPPER_BRACKET_MOUNT_GAP
)


# --- Camera bracket adapter (rev 1) -------------------------------------
#
# Wedge spacer between the flat front wall (world +X / +Y) and the
# 25°-tilted bracket back face. Placed in world without rotation; its
# local origin sits on the inner face of `wall_front`, vertically
# aligned with the bracket back face centre so the adapter's tilted
# front face is coplanar with the bracket's back face.

_TILT_RAD = math.radians(CAMERA_ROT_DEG[0])
_SIN_TILT = math.sin(_TILT_RAD)
_COS_TILT = math.cos(_TILT_RAD)

# Bracket back face centre in world: bracket-local (0, 0, -PLATE_T)
# rotated by Rot(TILT, 0, 0) and translated by CAMERA_POS.
CAMERA_BRACKET_BACK_CENTRE = (
    CAMERA_POS[0],
    CAMERA_POS[1] + CAMERA_BRACKET_PLATE_T * _SIN_TILT,
    CAMERA_POS[2] - CAMERA_BRACKET_PLATE_T * _COS_TILT,
)

# Adapter local origin in world. X = 0 (centred on the scanner spine);
# Y matches the bracket back face centre Y so the wedge is symmetric
# around the bracket's vertical axis; Z sits on the front-wall inner
# face.
ADAPTER_POS = (
    0.0,
    CAMERA_BRACKET_BACK_CENTRE[1],
    ENCLOSURE_INNER_Z_MIN,
)


# --- Platform ⇄ load-cell standoff interface ----------------------------
#
# External interface of `platform/` that the adjacent `load_cell_standoff/`
# must agree on. Each of these duplicates an internal constant in
# platform/assembly.py; change either side and the top-level cross-
# boundary interference assertion will catch the drift at the 0.3 mm
# MOUNT_GAP scale.

# 500 g beam load-cell geometry (duplicates platform/assembly.py).
PLATFORM_COLLAR_T = 8.0
PLATFORM_LOAD_CELL_L = 80.0
PLATFORM_LOAD_CELL_H = 13.0
PLATFORM_LOAD_CELL_R = 150.0
PLATFORM_LOAD_CELL_AZ_DEG = (90.0, 210.0, 330.0)

# Beam bottom face world-Y (collar bottom = -COLLAR_T, beam hanging
# one LOAD_CELL_H below that).
LOAD_CELL_BOTTOM_Y = -(PLATFORM_COLLAR_T + PLATFORM_LOAD_CELL_H)


# --- Load-cell fixed-end bolt pattern (beam ⇄ standoff interface) -------
#
# The outer (fixed) end of each 500 g beam load cell carries a two-bolt
# mounting pattern along the beam's radial axis. The standoff's top
# face matches this pattern so each beam bolts down to its own column.
# Values are conservative for a generic 80 × 13 × 13 beam (real
# TAL220-class cells are typically 15 mm spacing with the outer bolt
# 6 mm from the outer edge); re-spec when a specific cell is picked.

# Distance between the two bolts along the beam's radial axis (beam-
# local +X = standoff-local +X when heading = azimuth).
LOAD_CELL_FIXED_END_BOLT_SPACING = 15.0

# Radial position of the bolt-pattern centre, measured from the platform
# origin along the beam's radial outward direction. Beam extends
# r ∈ [PLATFORM_LOAD_CELL_R, PLATFORM_LOAD_CELL_R + PLATFORM_LOAD_CELL_L]
# = [150, 230]; bolt-pattern centre 10 mm inboard of the outer end.
LOAD_CELL_FIXED_END_BOLT_CENTER_R = (
    PLATFORM_LOAD_CELL_R + PLATFORM_LOAD_CELL_L - 10.0
)


# --- Load-cell standoff placement (rev 1) -------------------------------
#
# Each load-cell fixed end is carried from the enclosure floor by one
# printed column. The three instances share one part design
# (`load_cell_standoff/`) and differ only in the Ry(heading) placement
# rotation, mirroring the two-instance below-mirror floor bracket
# pattern one directory up.
#
# Standoff local frame (see load_cell_standoff/assembly.py):
#     origin : centre of column bottom face, on the enclosure floor
#              (y = ENCLOSURE_INNER_Y_MIN), directly under the load-
#              cell fixed-end bolt pattern centre
#     +Y     : world +Y (up)
#     +X     : beam radial outward direction (horizontal)
#     +Z     : beam tangential (horizontal; completes right-handed
#              frame with +X, +Y)
#
# Heading: under the platform's Rot(-90, 0, 0) world map, a beam at
# platform-local azimuth A (measured about platform-local +Z from
# platform-local +X) has its radial outward vector land on world
#     Rot(-90, 0, 0) · (cos A, sin A, 0) = (cos A, 0, -sin A)
# which is exactly Ry(A) · (1, 0, 0). So the standoff's world heading
# (Ry component) equals the load-cell's platform-local azimuth.

# Clearance between the standoff top face and the load-cell beam
# bottom face along world +Y. Same rationale as the mirror-bracket
# MOUNT_GAPs — separates coplanar faces so the cross-boundary boolean
# reads a clean zero volume at the 0.001 mm³ threshold, and absorbs
# sub-millimetre print-dimension drift on both parts.
LOAD_CELL_STANDOFF_MOUNT_GAP = 0.3

LOAD_CELL_STANDOFF_TOP_Y = LOAD_CELL_BOTTOM_Y - LOAD_CELL_STANDOFF_MOUNT_GAP

# Column height, floor → top face.
LOAD_CELL_STANDOFF_HEIGHT = (
    LOAD_CELL_STANDOFF_TOP_Y - ENCLOSURE_INNER_Y_MIN
)


def _load_cell_standoff_pos(az_deg: float) -> tuple[float, float, float]:
    """World position of a standoff's local origin — bottom face centre
    on the enclosure floor, radially under the fixed-end bolt pattern
    centre. Derived from heading = az convention above."""
    az_rad = math.radians(az_deg)
    r = LOAD_CELL_FIXED_END_BOLT_CENTER_R
    return (
        r * math.cos(az_rad),
        ENCLOSURE_INNER_Y_MIN,
        -r * math.sin(az_rad),
    )


LOAD_CELL_STANDOFF_POSITIONS = tuple(
    _load_cell_standoff_pos(_az) for _az in PLATFORM_LOAD_CELL_AZ_DEG
)
LOAD_CELL_STANDOFF_HEADING_DEG = PLATFORM_LOAD_CELL_AZ_DEG


# --- Load-cell standoff ⇄ floor panel interface -------------------------
#
# Four M4 floor bolts secure each standoff to the enclosure floor. The
# standoff's bottom face carries the through-hole pattern today; the
# mating through-hole pattern in `enclosure.wall_floor` is a rev 2
# follow-up bundled with the below-mirror bracket floor-bolt pattern
# (see `../NEXT_STEPS.md` "Suggested next CAD moves" item 2).

# Square XZ spacing of the four-bolt pattern on the standoff's bottom
# face (and the matching floor-panel through-holes once they land).
# 24 mm chosen so the pattern fits comfortably inside the column
# cross-section (see load_cell_standoff/assembly.py for the width
# constraint driven by the below-mirror bracket clearance).
LOAD_CELL_STANDOFF_FLOOR_BOLT_SPACING = 24.0

# M4 clearance hole diameter. Pure axial bolts (no tilt-driven lateral
# offset), so 4.5 mm matches the camera-adapter wall-bolt convention
# rather than the mirror-bracket widened 5.0 mm.
LOAD_CELL_STANDOFF_BOLT_HOLE_D = 4.5


# --- Skeleton rail interface --------------------------------------------
#
# 2020 T-slot extrusion geometry for the rev-2 frame skeleton (see
# skeleton/assembly.py). Duplicated here so mount clamps that grip a
# rail can derive their placement without sibling-to-sibling imports;
# the skeleton's own `assert` block pairs with these values.

SKELETON_RAIL_MEMBER = 20.0                         # 2020 cross-section
SKELETON_RAIL_CORNER_X = 290.0                      # |x| of corner posts' centerlines
SKELETON_RAIL_CORNER_Z = 290.0                      # |z| of corner posts' centerlines
SKELETON_RAIL_FLOOR_CENTERLINE_Y = -230.0           # floor rail centerlines
SKELETON_RAIL_FLOOR_TOP_Y = (
    SKELETON_RAIL_FLOOR_CENTERLINE_Y + SKELETON_RAIL_MEMBER / 2
)                                                   # -220 (rail +Y face)
SKELETON_RAIL_CEIL_CENTERLINE_Y = +330.0            # ceiling rail centerlines
SKELETON_RAIL_CEIL_BOTTOM_Y = (
    SKELETON_RAIL_CEIL_CENTERLINE_Y - SKELETON_RAIL_MEMBER / 2
)                                                   # +320 (rail -Y face)

# Rev-2 mid-front transverse rail. Spans world +X between post_fl and
# post_fr at the camera bracket back face height, coplanar with the
# other front rails in Z (same centerline z = -SKELETON_RAIL_CORNER_Z).
# Its +Z (inboard) face carries the camera-bracket rail clamp; choosing
# the rail centerline Y equal to the bracket back face Y makes the tab
# beam run nearly horizontal (pure +Z in world, ~0° tilt in clamp-local).
SKELETON_RAIL_FRONT_MID_Y = CAMERA_BRACKET_BACK_CENTRE[1]        # ≈ 107.81
SKELETON_RAIL_FRONT_MID_FRONT_FACE_Z = (
    -SKELETON_RAIL_CORNER_Z + SKELETON_RAIL_MEMBER / 2
)                                                                # -280


# --- Mirror-mount rail clamp placement (rev 2) --------------------------
#
# Rev 2 replacement for the rev-1 printed wedges that supported each
# mirror from an enclosure panel. A rail clamp grips a 2020 extrusion
# member of the rev-2 skeleton and presents the mirror_mount's 94 × 94
# four-corner bolt pattern at the mount's canonical pose.
#
# All three mirrors (below-left, below-right, back-upper) share one
# parameterised clamp body (see `mirror_mount_rail_clamp/assembly.py`);
# each instance supplies its own placement + mount-in-clamp Location.
# Clamp-local geometry is identical by symmetry for the two below-
# mirrors; the back-upper has a different mount-back distance so the
# beam tilt/length differ (the `clamp()` function derives those from
# the mount Location passed in).
#
# Clamp-local frame (see mirror_mount_rail_clamp/assembly.py):
#     origin : centre of clamp bottom face, on rail face being gripped,
#              at the along-rail position of the mount plate back face
#     +X     : along rail long axis
#     +Y     : away from rail (clamp body rises in +Y)
#     +Z     : from rail toward mount (horizontal for the below-mirror
#              floor-rail instances; tilted for the back-upper ceiling-
#              rail instance)
#
# Each instance carries a full `Rot` clamp rotation rather than a
# heading float because the back-upper's placement is not a pure Ry.

# Below-left mount back face centre in world (same derivation as the
# floor-bracket placement above; uses _back_face_centre_world).
_BL_CLAMP_BACK = _back_face_centre_world(
    MIRROR_BELOW_LEFT_POS, MIRROR_BELOW_LEFT_NORMAL
)

# Clamp origin on rail top, directly under the mount back face centre.
RAIL_CLAMP_BELOW_LEFT_POS = (
    -SKELETON_RAIL_CORNER_X,              # rail_floor_left centerline X
    SKELETON_RAIL_FLOOR_TOP_Y,            # rail top face Y
    _BL_CLAMP_BACK[2],                    # same Z as mount back face
)

# Ry(+90°): clamp-local +X → world -Z (along rail_floor_left long axis),
# +Y → world +Y (away from rail top), +Z → world +X (toward mount at
# x ≈ -207 from rail at x = -290).
RAIL_CLAMP_BELOW_LEFT_CLAMP_ROT = Rot(0, 90, 0)


# Below-right mirror. X-mirror of the below-left placement: same rail-
# top Y and same Z under the mount back face, opposite X post and
# opposite Ry heading. Clamp-local geometry (mount-back position, beam
# tilt, pad pose) is identical to below-left by mirror symmetry — the
# pad is square, the four-corner mount-bolt pattern is reflection-
# symmetric, and the sanity asserts attached to the below-left
# computation cover both instances.
_BR_CLAMP_BACK = _back_face_centre_world(
    MIRROR_BELOW_RIGHT_POS, MIRROR_BELOW_RIGHT_NORMAL
)

RAIL_CLAMP_BELOW_RIGHT_POS = (
    +SKELETON_RAIL_CORNER_X,              # rail_floor_right centerline X
    SKELETON_RAIL_FLOOR_TOP_Y,            # rail top face Y
    _BR_CLAMP_BACK[2],                    # same Z as mount back face
)

# Ry(-90°): clamp-local +X → world +Z, +Y → world +Y, +Z → world -X
# (toward mount at x ≈ +207 from rail at x = +290).
RAIL_CLAMP_BELOW_RIGHT_CLAMP_ROT = Rot(0, -90, 0)


# Back-upper mirror. Clamps onto `rail_ceil_back` (ceiling rail running
# along world +X at y = +330, z = +290), gripping the rail's -Y
# (bottom-in-world) face so the clamp hangs below the rail with the
# mount pad reaching down-and-inboard to the back-upper mount back
# face centre at world (0, 179.7, 179.8). Reach in clamp-local is
# ≈ (0, 140, 110) — longer than the below-mirror clamps' (0, 75, 83);
# rev-2 follow-up is to add a mid-height back cross-member and shorten
# the tab (see `../NEXT_STEPS.md`).
#
# Placement rotation is Rx(180°): clamp-local +X → world +X (along
# rail), +Y → world -Y (down, away from rail bottom face), +Z → world
# -Z (inboard toward mount). Unlike the below-mirror clamps' pure
# Ry(heading) placement, this combines two world axes, so the
# constant is stored as a full `Rot`.

_BU_CLAMP_BACK = _back_face_centre_world(
    MIRROR_BACK_UPPER_POS, MIRROR_BACK_UPPER_NORMAL
)

assert abs(_BU_CLAMP_BACK[0]) < 1e-6, (
    "back-upper mirror has drifted off the YZ plane; the back-upper "
    "rail clamp assumes the x-symmetric (centered on the rail) pose"
)

# Clamp origin on rail bottom face, centred along rail under the mount
# back face centre.
RAIL_CLAMP_BACK_UPPER_POS = (
    _BU_CLAMP_BACK[0],                    # x = 0 (centred on rail_ceil_back)
    SKELETON_RAIL_CEIL_BOTTOM_Y,          # rail -Y face Y
    SKELETON_RAIL_CORNER_Z,               # rail_ceil_back centerline Z
)

RAIL_CLAMP_BACK_UPPER_CLAMP_ROT = Rot(180, 0, 0)


# --- Camera-bracket rail clamp placement (rev 2) ------------------------
#
# Rev-2 replacement for the rev-1 `camera_bracket_adapter/` wedge. A
# rail clamp grips the new `rail_front_mid` transverse and presents the
# camera bracket's 52 × 52 four-corner bolt pattern at the bracket's
# canonical (25°-tilted) pose.
#
# Clamp-local frame (see camera_bracket_rail_clamp/assembly.py):
#     origin : centre of clamp bottom face, on the rail's +Z (inboard)
#              face, at the along-rail position of the bracket back
#              face centre (x = 0; rail runs along +X)
#     +X     : along rail long axis (world +X)
#     +Y     : away from rail (= world +Z, from the front-wall plane
#              toward the brick)
#     +Z     : from rail toward bracket along the horizontal axis
#              perpendicular to both (= world -Y, per the right-handed
#              frame convention)
#
# Placement rotation is Rx(+90°): clamp-local +X → world +X; +Y →
# world +Z; +Z → world -Y. Like the back-upper placement, this is not
# a pure Ry heading, so we store a full `Rot` rather than a float.
#
# The rail centerline Y equals the bracket back face Y
# (SKELETON_RAIL_FRONT_MID_Y), so the bracket back face in clamp-local
# lands at ≈ (0, 49, 0) — pure +Y reach, ~0° beam tilt. The bracket
# is tilted 25° about world +X, so the pad itself is tilted in clamp-
# local: its mount-facing face normal lands on clamp-local
# (0, -cos25°, -sin25°) ≈ (0, -0.906, -0.423). Beam and pad geometry
# are derived inside `camera_bracket_rail_clamp.clamp()` from the
# composed mount_in_clamp Location.

RAIL_CLAMP_CAMERA_POS = (
    0.0,
    SKELETON_RAIL_FRONT_MID_Y,
    SKELETON_RAIL_FRONT_MID_FRONT_FACE_Z,
)

RAIL_CLAMP_CAMERA_CLAMP_ROT = Rot(90, 0, 0)
