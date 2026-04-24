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


def rotation_for_normal(normal: tuple[float, float, float]) -> Rot:
    """Rotation that carries a sub-assembly's local +Z onto `normal`.

    Used to orient each mirror_mount: the mount's local +Z is the
    mirror outward normal, and this returns the `Rot(...)` that places
    that direction correctly in world space.

    Roll about the target normal is unconstrained for the square mirror
    + threefold adjuster triangle at this rev; we pick the solution
    with Rz = 0. If a future rev needs a specific corner orientation,
    post-multiply by `Rot(0, 0, roll_deg)` on the caller side.
    """
    nx, ny, nz = normal
    n_mag = math.sqrt(nx * nx + ny * ny + nz * nz)
    nx, ny, nz = nx / n_mag, ny / n_mag, nz / n_mag

    # `Rot(a, b, 0)` applied to (0, 0, 1) yields
    # (cos(a)·sin(b), -sin(a), cos(a)·cos(b)). Solve for a, b.
    alpha = math.degrees(math.asin(-ny))
    ca = math.cos(math.radians(alpha))
    if abs(ca) < 1e-9:
        # Degenerate (local +Z maps onto ±Y): any beta works.
        beta = 0.0
    else:
        beta = math.degrees(math.atan2(nx / ca, nz / ca))
    return Rot(alpha, beta, 0.0)


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
