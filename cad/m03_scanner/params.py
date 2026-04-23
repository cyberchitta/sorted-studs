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
