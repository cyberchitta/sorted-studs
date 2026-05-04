"""
M03 camera rig — unit-local constants and placement helpers.

The camera rig images a brick at the brick centroid using four cameras
placed at the vertices of a regular tetrahedron centred on the origin.
The rig's local frame matches world (the top-level scanner places it
at world identity); all coordinates here are world-equivalent.

    origin = brick centroid at the scan position, on the platform top face
    +X     = operator's right
    +Y     = away from operator (toward the back of the machine)
    +Z     = up

Each rig sub-assembly (`skeleton_t_frame/`, `camera_mount_esp32cam/`,
`frame_gusset_t/`, `frame_gusset_l/`) carries its own local frame and
imports the constants below via `from camera_rig.params import …`.

Tetrahedral verification (2026-05-04): all four vertices sit
~248 mm from origin (within ±2 mm), and every pairwise dot product of
unit-vectors-from-origin is −1/3 — the regular-tetrahedron signature
(109.47° between every pair). The vertex set is unchanged from rev-4
(mirrors-and-direct-camera): the previous direct camera position
became one vertex and the three mirror centres became the other three.
See `cameras_design_notes.md` for the pivot rationale.

Vertex coordinates were originally derived in
`klods-syn/nbs/200_viewpoints.md` at tilt 65°, phase 90°,
camera_dist 250 mm. The upstream source is Y-up; values here are
converted to Z-up at the boundary (X invariant, Y/Z swapped). Update
this file and `docs/m03-multi-view-scanner.md` together whenever any
value changes.
"""

import math

from build123d import Rot


# --- Print floor (matches project-wide value in ../params.py) ----------

WALL_MIN = 1.5


# --- Tetrahedral camera vertices ----------------------------------------
#
# World-frame positions where each camera's PCB rear face lands. Names
# are pose-descriptive (independent of any prior mirror/camera role) so
# a future role swap doesn't churn the names.

VERTEX_FRONT_LOW_POS = (0.0, -226.6, 105.7)
VERTEX_BACK_UPPER_POS = (0.0, 172.7, 176.0)
VERTEX_BELOW_LEFT_POS = (-201.4, 25.4, -140.1)
VERTEX_BELOW_RIGHT_POS = (201.4, 25.4, -140.1)


def lens_axis_unit(
    vertex_pos: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Unit vector from a camera vertex toward the world origin
    (= the lens optical axis direction for a centroid-pointing camera).
    """
    mag = math.sqrt(sum(c * c for c in vertex_pos))
    return tuple(-c / mag for c in vertex_pos)


# --- Camera mount external interface ------------------------------------
#
# Plate thickness exposed here because the per-vertex clamp pose
# constants below land the clamp directly under the plate-back centre,
# and the back centre depends on the plate offset along the lens axis.
# PCB outline + corner-bolt pattern stay private inside
# `camera_mount_esp32cam/`.

CAMERA_MOUNT_PLATE_T = 5.0


def plate_back_centre_world(
    vertex_pos: tuple[float, float, float],
    plate_t: float = CAMERA_MOUNT_PLATE_T,
) -> tuple[float, float, float]:
    """Mount plate back-face centre in world: vertex_pos shifted away
    from origin along the vertex direction by plate_t.

    Equivalently: vertex_pos − plate_t · lens_axis_unit(vertex_pos),
    since lens_axis is the unit vector from vertex toward origin.
    """
    axis = lens_axis_unit(vertex_pos)
    return tuple(vertex_pos[i] - plate_t * axis[i] for i in range(3))


def rotation_for_normal(
    normal: tuple[float, float, float],
    roll_deg: float = 0.0,
) -> Rot:
    """Rotation that carries a sub-assembly's local +Z onto `normal`.

    Used to orient each camera mount: the mount's local +Z is the lens
    axis (from vertex toward origin), and this returns the `Rot(...)`
    that places that direction correctly in world space.

    Roll about the target normal is unconstrained for the symmetric
    PCB + 4-corner bolt pattern at this rev. `roll_deg` sets the
    `Rz` component of the Euler triple; callers that need mount-local
    +X to fall on a specific world direction (e.g. horizontal for
    cable-routing consistency) supply the value computed by
    `horizontal_x_roll_deg(normal)`.
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
    perpendicular to the normal and has zero world-Z component — it
    aligns with the horizontal "tilt axis" of a bracket whose frame
    is `Rot(0, heading, 0)` for this normal's heading.

    Derivation: full rotation matrix is Rx(a)·Ry(b)·Rz(c) per
    `Rot(a,b,c)` semantics. Mount-local +X lands at world
    `(cos b · cos c, cos a · sin c + sin a · sin b · cos c,
      sin a · sin c − cos a · sin b · cos c)`.
    Setting the Z component to zero (under Z-up "horizontal" means
    zero vertical / Z component):
        sin a · sin c − cos a · sin b · cos c = 0
      ⇒ tan c = cos a · sin b / sin a
    Pick the root with `cos c > 0` so `R_11 = cos b · cos c` keeps
    the sign of `cos b`, i.e. mount +X stays on the +X side of the
    horizontal plane for normals with +Z component, matching the
    bracket's +X by convention.
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

    # Root with cos c > 0. tan c = ca · sb / sa.
    num = ca * sb
    den = sa
    if den < 0:
        num, den = -num, -den
    return math.degrees(math.atan2(num, den))


# --- T-frame rail positions ---------------------------------------------
#
# Held stable from rev-4 (the mirrored design): vertex coordinates are
# unchanged in the rev-5 pivot, and the rail offsets that were sized
# to clear 110 × 110 mm mirror plates now have generous extra margin
# against the smaller ESP32-CAM mounts. A later size-optimization
# sweep will tighten these against the new mount footprint.

# Stock 2020 T-slot extrusion cross-section.
SKELETON_RAIL_MEMBER = 20.0

T_FRAME_RAIL_A_Z = -220.0                                # rail A centerline Z (height)
T_FRAME_RAIL_A_TOP_Z = T_FRAME_RAIL_A_Z + SKELETON_RAIL_MEMBER / 2   # = -210
# Rev-4 derivation: back-face Y of the below-left mirror plate (8 mm
# offset from the mirror centre along the mirror normal). Held as a
# literal here so the rail position is independent of the new
# ESP32-CAM mount's plate offset (5 mm) — the size-optimization sweep
# will re-derive this against the new mount.
T_FRAME_RAIL_A_Y = 28.44
T_FRAME_RAIL_A_FRONT_Y = T_FRAME_RAIL_A_Y - SKELETON_RAIL_MEMBER / 2 # ≈ 18.44
T_FRAME_RAIL_C_Y = 240.0                                 # rail C centerline Y (depth)
T_FRAME_RAIL_C_FRONT_Y = T_FRAME_RAIL_C_Y - SKELETON_RAIL_MEMBER / 2 # = 230
T_FRAME_RAIL_D_Z = 250.3                                 # rail D centerline Z (height)
T_FRAME_RAIL_D_BOTTOM_Z = T_FRAME_RAIL_D_Z - SKELETON_RAIL_MEMBER / 2   # = 240.3


# --- Per-vertex camera-mount placement ---------------------------------
#
# Four world placements of the single `camera_mount_esp32cam/` body —
# print one body, parameterise it by `mount_in_clamp` for each vertex.
# The clamp is X-Y centred under the plate-back centre so its beam
# runs straight up (or down, for the front-low cantilever-underside
# mount) to the supported plate.
#
# Clamp-local frame (see `camera_mount_esp32cam/assembly.py`):
#     origin : centre of clamp bottom face, on the rail face the clamp
#              grips, directly under the mount plate back-face centre
#              along the rail axis
#     +X     : along rail long axis
#     +Y     : away from rail (clamp body rises in +Y)
#     +Z     : right-handed third axis
#
# Rail assignments (unchanged from rev-4):
#     front-low  → rail D's -Z (underside) face
#     back-upper → rail C's -Y face
#     below-*    → rail A's +Z (top) face

_FL_BACK = plate_back_centre_world(VERTEX_FRONT_LOW_POS)
_BU_BACK = plate_back_centre_world(VERTEX_BACK_UPPER_POS)
_BL_BACK = plate_back_centre_world(VERTEX_BELOW_LEFT_POS)
_BR_BACK = plate_back_centre_world(VERTEX_BELOW_RIGHT_POS)

assert abs(_FL_BACK[0]) < 1e-6, (
    "front-low vertex has drifted off the YZ plane; the front-low rail "
    "clamp assumes the x-symmetric (centered on rail D) pose"
)
assert abs(_BU_BACK[0]) < 1e-6, (
    "back-upper vertex has drifted off the YZ plane; the back-upper "
    "rail clamp assumes the x-symmetric (centered on rail C) pose"
)

# Below-left: clamp on rail A's +Z face (top of rail), X-centred under
# the mount-back centre. `Rot(90, 0, 0)` maps clamp-local +X→world +X
# (along rail), +Y→world +Z (clamp body rises), +Z→world -Y.
RAIL_PAD_CLAMP_BELOW_LEFT_POS = (
    _BL_BACK[0],
    _BL_BACK[1],
    T_FRAME_RAIL_A_TOP_Z,
)
RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT = Rot(90, 0, 0)

# Below-right: X-mirror of below-left. Same rotation; the mount's own
# X-flipped lens axis supplies the chirality, so the printed body is
# the X-mirror of below-left's (different drilled M2 corner positions,
# same overall shape).
RAIL_PAD_CLAMP_BELOW_RIGHT_POS = (
    _BR_BACK[0],
    _BR_BACK[1],
    T_FRAME_RAIL_A_TOP_Z,
)
RAIL_PAD_CLAMP_BELOW_RIGHT_CLAMP_ROT = Rot(90, 0, 0)

# Back-upper: clamp on rail C's -Y face (rail C runs along world +Z).
# Rot(180, 90, 0) maps clamp-local +X→world +Z (along rail C),
# +Y→world -Y (away from rail toward mount), +Z→world +X.
RAIL_PAD_CLAMP_BACK_UPPER_POS = (
    _BU_BACK[0],                      # x = 0 (rail C is on the YZ plane)
    T_FRAME_RAIL_C_FRONT_Y,           # rail C -Y face
    _BU_BACK[2],                      # mount-back Z
)
RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT = Rot(180, 90, 0)

# Front-low: clamp on rail D's -Z (underside) face. Rail D is the
# Y-cantilever projecting forward from the rail C ↔ D junction; the
# camera hangs below rail D with its lens axis tilting back-and-up at
# the brick centroid.
#
# Rot(-90, -90, 0): clamp-local +X→world +Y (along rail D toward back),
# +Y→world -Z (down toward mount), +Z→world -X.
RAIL_PAD_CLAMP_FRONT_LOW_POS = (
    _FL_BACK[0],                      # x = 0 (rail D centred on YZ plane)
    _FL_BACK[1],                      # under plate back-face centre
    T_FRAME_RAIL_D_BOTTOM_Z,          # rail D -Z face
)
RAIL_PAD_CLAMP_FRONT_LOW_CLAMP_ROT = Rot(-90, -90, 0)
