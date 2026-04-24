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
