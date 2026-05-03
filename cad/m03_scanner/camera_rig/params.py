"""
M03 camera rig — unit-local constants and placement helpers.

The camera rig groups everything the scanner uses to image a brick at
the brick centroid: the sparse T-frame, the merged camera clamp-bracket,
and the three merged mirror clamp-mounts. The rig's local frame happens
to match world (the top-level scanner places the rig at world identity);
all coordinates here are world-equivalent.

    origin = brick centroid at the scan position, on the platform top face
    +X     = operator's right
    +Y     = away from operator (toward the back of the machine)
    +Z     = up

Each rig sub-assembly (`skeleton_t_frame/`, `mirror_clamp_mount/`,
`camera_clamp_bracket/`, `frame_gusset_t/`, `frame_gusset_l/`) carries
its own local frame and imports the constants below via
`from camera_rig.params import …`.

Position and normal values are copied from
`docs/m03-multi-view-scanner.md` (derived in
`klods-syn/nbs/200_viewpoints.md` at tilt 65°, phase 90°,
camera_dist 250 mm). The upstream source is Y-up; the values here
are converted to Z-up at the boundary by swapping the second and
third tuple components (X invariant). Update this file and the
reference doc together whenever any value changes.
"""

import math

from build123d import Rot


# --- Print floor (matches project-wide value in ../params.py) ----------

WALL_MIN = 1.5


# --- Camera (direct viewpoint) ------------------------------------------

# Camera PCB rear face in world. This is the plane where the
# camera_clamp_bracket's bracket-local origin lands.
CAMERA_POS = (0.0, -226.6, 105.7)

# Pure X rotation placing the camera_clamp_bracket: local +Z (lens axis) maps
# to the unit vector from camera toward brick origin
# ≈ (0, +cos(25°), -sin(25°)) = (0, 0.9063, -0.4226). Solving
# `Rot(α, 0, 0) · (0, 0, 1) = (0, -sin α, cos α) = (0, cos 25°,
# -sin 25°)` gives α = -115° (equivalently 245°). The 25° below-
# horizontal tilt of the lens axis is preserved; under Z-up the same
# tilt is expressed by a rotation in the third quadrant of Rx.
CAMERA_ROT_DEG = (-115.0, 0.0, 0.0)


# --- Camera clamp-bracket external interface ----------------------------
#
# Plate thickness of `camera_clamp_bracket/` exposed here because
# `CAMERA_BRACKET_BACK_CENTRE` below derives from it to position the
# rail-D clamp directly under the plate back-face centre. Plate size +
# PCB-mount hole spacing are no longer cross-boundary (the merged body
# has no rev-3 corner-bolt pattern) and live private inside
# `camera_clamp_bracket/assembly.py`.

# Plate thickness along the bracket's local Z.
CAMERA_BRACKET_PLATE_T = 5.0

# Camera bracket plate back face centre in world. The bracket's local
# origin sits on the PCB rear plane (CAMERA_POS); this is the same
# point offset by PLATE_T along the bracket's local -Z (the back-face
# normal), then mapped to world by Rot(CAMERA_ROT_DEG[0], 0, 0). Used
# by RAIL_PAD_CLAMP_CAMERA_POS below to land the clamp directly under
# the plate-back centre so the beam runs straight up. The structure of
# the formula is unchanged from the Y-up version; only CAMERA_ROT_DEG[0]'s
# value changes underneath.
_CAMERA_TILT_RAD = math.radians(CAMERA_ROT_DEG[0])
_CAMERA_SIN_TILT = math.sin(_CAMERA_TILT_RAD)
_CAMERA_COS_TILT = math.cos(_CAMERA_TILT_RAD)
CAMERA_BRACKET_BACK_CENTRE = (
    CAMERA_POS[0],
    CAMERA_POS[1] + CAMERA_BRACKET_PLATE_T * _CAMERA_SIN_TILT,
    CAMERA_POS[2] - CAMERA_BRACKET_PLATE_T * _CAMERA_COS_TILT,
)


# --- Mirrors ------------------------------------------------------------

# Physical mirror centres in world (centre of reflective face).
MIRROR_BACK_UPPER_POS = (0.0, 172.7, 176.0)
MIRROR_BELOW_LEFT_POS = (-201.4, 25.4, -140.1)
MIRROR_BELOW_RIGHT_POS = (201.4, 25.4, -140.1)

# Mirror outward normals (unit). The reflective face normal points
# along +normal toward the camera/brick; each mirror_clamp_mount's
# mount-local +Z maps onto this direction under `rotation_for_normal(...)`.
MIRROR_BACK_UPPER_NORMAL = (0.0, -0.885, -0.466)
MIRROR_BELOW_LEFT_NORMAL = (0.690, -0.380, 0.616)
MIRROR_BELOW_RIGHT_NORMAL = (-0.690, -0.380, 0.616)


# --- Mirror mount external interface ------------------------------------
#
# Plate-level constants of `mirror_clamp_mount/` exposed here because
# `skeleton_t_frame/` and the `_back_face_centre_world(...)` helper
# below need them to project the mount plate's worst-case corner
# against the rail clearance envelope and to derive rail Y positions
# from the mount-back centre. Plate size + bolt spacing are no longer
# cross-boundary (the merged body has no mount bolts) and live private
# inside `mirror_clamp_mount/assembly.py`.

# Mirror glass thickness.
MIRROR_MOUNT_MIRROR_T = 3.0

# Printed plate thickness (rev-4 merged body).
MIRROR_MOUNT_PLATE_T = 5.0

# Offset from mirror reflective face to plate back face along mount-
# local -Z (= outward-normal direction reversed). The merged body's
# beam meets the plate at this depth.
MIRROR_MOUNT_BACK_OFFSET = MIRROR_MOUNT_MIRROR_T + MIRROR_MOUNT_PLATE_T


def _back_face_centre_world(pos, normal, offset=MIRROR_MOUNT_BACK_OFFSET):
    """Mount plate back face centre in world: mirror_pos - offset·normal."""
    return (pos[0] - offset * normal[0],
            pos[1] - offset * normal[1],
            pos[2] - offset * normal[2])


def rotation_for_normal(
    normal: tuple[float, float, float],
    roll_deg: float = 0.0,
) -> Rot:
    """Rotation that carries a sub-assembly's local +Z onto `normal`.

    Used to orient each mirror_clamp_mount: the mount's local +Z is
    the mirror outward normal, and this returns the `Rot(...)` that
    places that direction correctly in world space.

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
    #
    # The math is frame-agnostic — feed it any unit vector and it
    # returns the (a, b) pair that lands local +Z onto it. The fact
    # that under Z-up world the mirror normals' Z component encodes
    # the up-down split (vs Y under the upstream Y-up source) does
    # not change the algebra here.
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


# --- Rev-3 T-frame rail positions ---------------------------------------
#
# The rev-3 sparse T-frame (see `skeleton_t_frame/assembly.py`)
# carries only four rails plus two corner braces. Rails A (X-span)
# and C (vertical post) are placed just outside the mount plate
# volumes — close enough that the printed clamp is a short stub but
# far enough that the clamp base clears the plate by a few mm.
# Magic numbers below derive from the worst-case plate-projection
# analysis in skeleton_t_frame/assembly.py.

# Stock 2020 T-slot extrusion cross-section. Used here and by the
# mirror_clamp_mount + camera_clamp_bracket to match rail face Y/Z
# coordinates.
SKELETON_RAIL_MEMBER = 20.0

T_FRAME_RAIL_A_Z = -220.0                                # rail A centerline Z (height)
T_FRAME_RAIL_A_TOP_Z = T_FRAME_RAIL_A_Z + SKELETON_RAIL_MEMBER / 2   # = -210
T_FRAME_RAIL_A_Y = _back_face_centre_world(
    MIRROR_BELOW_LEFT_POS, MIRROR_BELOW_LEFT_NORMAL
)[1]                                                                 # ≈ 28.44 (depth)
T_FRAME_RAIL_A_FRONT_Y = T_FRAME_RAIL_A_Y - SKELETON_RAIL_MEMBER / 2 # ≈ 18.44
T_FRAME_RAIL_C_Y = 240.0                                 # rail C centerline Y (depth)
T_FRAME_RAIL_C_FRONT_Y = T_FRAME_RAIL_C_Y - SKELETON_RAIL_MEMBER / 2 # = 230
T_FRAME_RAIL_D_Z = 250.3                                 # rail D centerline Z (height)
T_FRAME_RAIL_D_BOTTOM_Z = T_FRAME_RAIL_D_Z - SKELETON_RAIL_MEMBER / 2   # = 240.3


# --- Mirror clamp-mount placement --------------------------------------
#
# Three world placements of the single `mirror_clamp_mount/` body —
# print one body, parameterise it by `mount_in_clamp` for each mirror.
# The rev-3 rail positions put the mount-back at ≈ (0, 65, 0) in
# clamp-local for the below-mirrors and ≈ (0, 50, 0) for the back-
# upper, so the beam runs straight up.
#
# Clamp-local frame (see `mirror_clamp_mount/assembly.py`):
#     origin : centre of clamp bottom face, on the rail face the clamp
#              grips, directly under the mount plate back-face centre
#              along the rail axis
#     +X     : along rail long axis
#     +Y     : away from rail (clamp body rises in +Y)
#     +Z     : right-handed third axis; for the below-mirrors this is
#              the world horizontal perpendicular to the rail; for the
#              back-upper it is world +X
#
# Each instance pairs a world placement (`*_POS`) with a `Rot` that
# carries clamp-local axes onto world axes. Under Z-up the below-
# mirror clamps gain a non-identity rotation (was `Rot(0,0,0)`):
# clamp-local +Y must map to world +Z (vertical), which `Rot(90,0,0)`
# delivers.

# Mount plate back face centres in world — used as the world Y of each
# clamp instance so the beam runs straight up to the supported mount.
_BL_CLAMP_BACK = _back_face_centre_world(
    MIRROR_BELOW_LEFT_POS, MIRROR_BELOW_LEFT_NORMAL
)
_BR_CLAMP_BACK = _back_face_centre_world(
    MIRROR_BELOW_RIGHT_POS, MIRROR_BELOW_RIGHT_NORMAL
)
_BU_CLAMP_BACK = _back_face_centre_world(
    MIRROR_BACK_UPPER_POS, MIRROR_BACK_UPPER_NORMAL
)
assert abs(_BU_CLAMP_BACK[0]) < 1e-6, (
    "back-upper mirror has drifted off the YZ plane; the back-upper "
    "rail clamp assumes the x-symmetric (centered on the rail) pose"
)

# Below-left: clamp on rail A's +Z face (top of rail), X-centred under
# the mount-back centre. `Rot(90, 0, 0)` maps clamp-local +X→world +X
# (along rail), +Y→world +Z (clamp body rises), +Z→world -Y.
RAIL_PAD_CLAMP_BELOW_LEFT_POS = (
    _BL_CLAMP_BACK[0],                # mount-back X (= mirror_pos.x - 8·mirror_normal.x)
    _BL_CLAMP_BACK[1],                # mount-back Y (= rail A centerline Y, beam straight up)
    T_FRAME_RAIL_A_TOP_Z,             # rail A +Z face (top)
)
RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT = Rot(90, 0, 0)

# Below-right: X-mirror of below-left. Same rotation; the mount's own
# X-flipped normal supplies the chirality, so the printed body is the
# X-mirror of below-left's (different drilled holes, same overall
# shape).
RAIL_PAD_CLAMP_BELOW_RIGHT_POS = (
    _BR_CLAMP_BACK[0],
    _BR_CLAMP_BACK[1],
    T_FRAME_RAIL_A_TOP_Z,
)
RAIL_PAD_CLAMP_BELOW_RIGHT_CLAMP_ROT = Rot(90, 0, 0)

# Back-upper: clamp on rail C's -Y face (rail C runs along world +Z).
# Rot(180, 90, 0) maps clamp-local +X→world +Z (along rail C),
# +Y→world -Y (away from rail toward mount), +Z→world +X.
RAIL_PAD_CLAMP_BACK_UPPER_POS = (
    _BU_CLAMP_BACK[0],                # x = 0 (rail C is on the YZ plane)
    T_FRAME_RAIL_C_FRONT_Y,           # rail C -Y face
    _BU_CLAMP_BACK[2],                # mount-back Z (= rail C clamp Z)
)
RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT = Rot(180, 90, 0)


# --- Camera clamp-bracket placement -------------------------------------
#
# The camera hangs below rail D — the Y-cantilever projecting forward
# from the rail C ↔ D junction. The merged body's clamp grips rail D's
# -Z (underside) face directly above the bracket plate back-face centre,
# so its beam runs straight down to the plate with no Y offset.
#
# Clamp-local frame (see `camera_clamp_bracket/assembly.py`):
#     origin : centre of clamp bottom face, on rail D's -Z face,
#              at the along-rail (Y) position of the plate back face
#     +X     : along rail D long axis (= world +Y)
#     +Y     : away from rail (downward in world; merged body extends
#              in clamp +Y to reach the plate below the rail)
#     +Z     : right-handed third (= world -X)
#
# Beam reach in clamp-local +Y is fixed by the rail D underside Z minus
# the plate back face Z: 240.3 - 107.81 ≈ 132.5 mm. Longer than the
# merged mirror clamp-mounts' 50–65 mm reach; rail D's height is set by
# clearance over the back-upper mount plate corner, so it can't be
# pushed any closer to the camera. The strut + propped-beam architecture
# (see `skeleton_t_frame/assembly.py`) is what keeps the cantilever
# stiff under this load.

RAIL_PAD_CLAMP_CAMERA_POS = (
    CAMERA_BRACKET_BACK_CENTRE[0],    # x = 0 (rail D centred on YZ plane)
    CAMERA_BRACKET_BACK_CENTRE[1],    # under plate back face centre
    T_FRAME_RAIL_D_BOTTOM_Z,          # rail D -Z face
)

# Rot(-90, -90, 0): clamp-local +X→world +Y (along rail D toward back),
# +Y→world -Z (down toward bracket), +Z→world -X.
RAIL_PAD_CLAMP_CAMERA_CLAMP_ROT = Rot(-90, -90, 0)
