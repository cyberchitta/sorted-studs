"""
M03 scanner — shared world-frame constants and placement helpers.

The world frame is the frame of the top-level assembly (`assembly.py`):

    origin = brick centroid at the scan position, on the platform top face
    +X     = operator's right
    +Y     = up
    +Z     = away from operator (toward the back of the machine)

Each rev-3 sub-assembly (`skeleton_t_frame/`, `mirror_mount/`,
`mirror_pad_clamp/`, `camera_bracket/`, `camera_pad_clamp/`,
`platform_frame/`, `platform_hinge_bearing/`, `platform_acrylic/`)
carries its own local frame; the top-level assembly composes each
into world space using `Pos(...)` and a rotation — either `Rot(...)`
directly or `rotation_for_normal(...)` for the mirrors where the
world normal drives the rotation.

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


# --- Camera bracket external interface ----------------------------------
#
# Plate-level constants of `camera_bracket/` that the camera_pad_clamp
# (rev-3) needs to size its mount pad and bolt pattern. Each duplicates
# an internal constant in camera_bracket/assembly.py; change either
# side and the top-level cross-boundary assertion catches the drift.

# Square plate dimension (PLATE_W == PLATE_H in camera_bracket).
CAMERA_BRACKET_PLATE_SIZE = 68.0

# Plate thickness along the bracket's local Z.
CAMERA_BRACKET_PLATE_T = 5.0

# Corner-bolt spacing (PLATE_W - 2·BOLT_INSET in camera_bracket, with
# BOLT_INSET = 8.0). Same 52 mm square pattern is reused on the rev-3
# camera pad-clamp's pad face.
CAMERA_BRACKET_BOLT_SPACING = 52.0

# Camera bracket back face centre in world. The bracket's local origin
# sits on the PCB rear plane (CAMERA_POS); this is the same point
# offset by PLATE_T along the bracket's local -Z (the back-face
# normal), then mapped to world by Rot(CAMERA_ROT_DEG[0], 0, 0). Used
# by RAIL_PAD_CLAMP_CAMERA_POS below.
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
MIRROR_BACK_UPPER_POS = (0.0, 176.0, 172.7)
MIRROR_BELOW_LEFT_POS = (-201.4, -140.1, 25.4)
MIRROR_BELOW_RIGHT_POS = (201.4, -140.1, 25.4)

# Mirror outward normals (unit). The reflective face normal points
# along +normal toward the camera/brick; each mirror_mount's local +Z
# maps onto this direction under `rotation_for_normal(...)`.
MIRROR_BACK_UPPER_NORMAL = (0.0, -0.466, -0.885)
MIRROR_BELOW_LEFT_NORMAL = (0.690, 0.616, -0.380)
MIRROR_BELOW_RIGHT_NORMAL = (-0.690, 0.616, -0.380)


# --- Mirror mount external interface ------------------------------------
#
# Plate-level constants of `mirror_mount/` that the rev-3
# mirror_pad_clamp + skeleton_t_frame need to size their pad and to
# project the plate's worst-case corner against the rail clearance
# envelope. Each duplicates an internal constant in
# mirror_mount/assembly.py; the top-level cross-boundary tangent-
# contact assertion catches drift.

# Mirror glass thickness.
MIRROR_MOUNT_MIRROR_T = 3.0

# Printed plate thickness (rigid rev 1).
MIRROR_MOUNT_PLATE_T = 5.0

# Offset from mirror reflective face to plate back face along mount-
# local -Z (= outward-normal direction reversed). Rev-3 mirror_pad_clamp
# pads land coplanar with this plane minus MOUNT_GAP.
MIRROR_MOUNT_BACK_OFFSET = MIRROR_MOUNT_MIRROR_T + MIRROR_MOUNT_PLATE_T

# Square plate size (PLATE_W == PLATE_H in mirror_mount).
MIRROR_MOUNT_PLATE_SIZE = 110.0

# Corner-bolt spacing (PLATE_SIZE - 2·BOLT_INSET in mirror_mount, with
# BOLT_INSET = 8.0). Same 94 mm square pattern is reused on the rev-3
# mirror pad-clamp's pad face.
MIRROR_MOUNT_BOLT_SPACING = 94.0


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


# --- Print parameters ---------------------------------------------------

# FDM printability floor — minimum wall thickness any printed
# sub-assembly may carry. Matches each sub-assembly's per-part
# `inspect()` threshold so a single change here propagates everywhere.
WALL_MIN = 1.5


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
# rev-3 mirror_pad_clamp + camera_pad_clamp to match rail face Y/Z
# coordinates.
SKELETON_RAIL_MEMBER = 20.0

T_FRAME_RAIL_A_Y = -220.0                                # rail A centerline Y
T_FRAME_RAIL_A_TOP_Y = T_FRAME_RAIL_A_Y + SKELETON_RAIL_MEMBER / 2   # = -210
T_FRAME_RAIL_A_Z = _back_face_centre_world(
    MIRROR_BELOW_LEFT_POS, MIRROR_BELOW_LEFT_NORMAL
)[2]                                                                 # ≈ 28.44
T_FRAME_RAIL_A_FRONT_Z = T_FRAME_RAIL_A_Z - SKELETON_RAIL_MEMBER / 2 # ≈ 18.44
T_FRAME_RAIL_C_Z = 240.0                                 # rail C centerline Z
T_FRAME_RAIL_C_FRONT_Z = T_FRAME_RAIL_C_Z - SKELETON_RAIL_MEMBER / 2 # = 230
T_FRAME_RAIL_D_Y = 250.3                                 # rail D centerline Y
T_FRAME_RAIL_D_BOTTOM_Y = T_FRAME_RAIL_D_Y - SKELETON_RAIL_MEMBER / 2   # = 240.3


# --- Rev-3 mirror pad-clamp placement -----------------------------------
#
# Three world placements of the single `mirror_pad_clamp/` body. Same
# conceptual role as the rev-2 RAIL_CLAMP_* (now removed) — print one
# body, parameterise it by `mount_in_clamp` for each mirror — but the
# rev-3 rail positions put the mount-back at ≈ (0, 65, 0) in clamp-
# local for the below-mirrors and ≈ (0, 50, 0) for the back-upper, so
# the beam runs straight up rather than rev-2's 38–48° tilt.
#
# Clamp-local frame (see `mirror_pad_clamp/assembly.py`):
#     origin : centre of clamp bottom face, on the rail face the clamp
#              grips, directly under the mount plate back-face centre
#              along the rail axis
#     +X     : along rail long axis
#     +Y     : away from rail (clamp body rises in +Y)
#     +Z     : right-handed third axis; for the below-mirrors this is
#              the world horizontal perpendicular to the rail; for the
#              back-upper it is world -X
#
# Each instance pairs a world placement (`*_POS`) with a `Rot` that
# carries clamp-local axes onto world axes.

# Mount plate back face centres in world — used as the world Z of each
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

# Below-left: clamp on rail A's +Y face, X-centred under the mount-
# back centre. Identity rotation — clamp-local matches world.
RAIL_PAD_CLAMP_BELOW_LEFT_POS = (
    _BL_CLAMP_BACK[0],                # mount-back X (= mirror_pos.x - 8·mirror_normal.x)
    T_FRAME_RAIL_A_TOP_Y,             # rail A +Y face
    _BL_CLAMP_BACK[2],                # mount-back Z (= rail A centerline Z, beam straight up)
)
RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT = Rot(0, 0, 0)

# Below-right: X-mirror of below-left. Same identity rotation; the
# mount's own X-flipped normal supplies the chirality, so the printed
# body is the X-mirror of below-left's (different drilled holes, same
# overall shape).
RAIL_PAD_CLAMP_BELOW_RIGHT_POS = (
    _BR_CLAMP_BACK[0],
    T_FRAME_RAIL_A_TOP_Y,
    _BR_CLAMP_BACK[2],
)
RAIL_PAD_CLAMP_BELOW_RIGHT_CLAMP_ROT = Rot(0, 0, 0)

# Back-upper: clamp on rail C's -Z face (rail C runs along world +Y).
# Rot(-90, -90, 0) maps clamp-local +X → world +Y (along rail), +Y →
# world -Z (away from rail toward mount), +Z → world -X.
RAIL_PAD_CLAMP_BACK_UPPER_POS = (
    _BU_CLAMP_BACK[0],                # x = 0 (rail C is on the YZ plane)
    _BU_CLAMP_BACK[1],                # mount-back Y (= rail C clamp Y)
    T_FRAME_RAIL_C_FRONT_Z,           # rail C -Z face
)
RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT = Rot(-90, -90, 0)


# --- Rev-3 camera pad-clamp placement -----------------------------------
#
# The rev-3 camera hangs below rail D — the Z-cantilever projecting
# forward from the rail C ↔ D junction. The clamp grips rail D's -Y
# (underside) face directly above the camera bracket back face centre,
# so its beam runs straight down to the bracket pad with no Z offset.
#
# Clamp-local frame (see `camera_pad_clamp/assembly.py`):
#     origin : centre of clamp bottom face, on rail D's -Y face,
#              at the along-rail (Z) position of the bracket back face
#     +X     : along rail D long axis (= world +Z)
#     +Y     : away from rail (downward in world; clamp body hangs in
#              clamp +Y while gravity pulls in clamp +Y)
#     +Z     : right-handed third (= world +X)
#
# Beam reach in clamp-local +Y is fixed by the rail D underside Y minus
# the bracket back face Y: 240.3 - 107.81 ≈ 132.5 mm. Longer than the
# rev-3 mirror clamps' 50–65 mm reach; rail D's height is set by
# clearance over the back-upper mount plate corner, so it can't be
# pushed any closer to the camera. The strut + propped-beam architecture
# (see `skeleton_t_frame/assembly.py`) is what keeps the cantilever
# stiff under this load.

RAIL_PAD_CLAMP_CAMERA_POS = (
    CAMERA_BRACKET_BACK_CENTRE[0],    # x = 0 (rail D centred on YZ plane)
    T_FRAME_RAIL_D_BOTTOM_Y,          # rail D -Y face
    CAMERA_BRACKET_BACK_CENTRE[2],    # under bracket back face centre
)

# Rot(180, 90, 0): clamp-local +X → world +Z (along rail D toward back),
# +Y → world -Y (down toward bracket), +Z → world +X (right-handed).
RAIL_PAD_CLAMP_CAMERA_CLAMP_ROT = Rot(180, 90, 0)


# --- Brick reference (from docs/parameters.md) -------------------------

# 90% of LEGO parts fit within an 80 mm sphere. Drives the minimum
# platform aperture and the camera-rig clearance envelope. Optical
# sightline cones at the platform plane (≤ Ø172 per rev-1's design)
# dominate this for acrylic sizing.
BRICK_SPHERE_DIA = 80.0


# --- Rev-3 platform frame (slice a) ------------------------------------
#
# Standalone freestanding 2020 frame that hosts the rev-3 hinged
# platform. Sits in front of and slightly under the camera rig's
# rail A / rail B bottom faces. Slides into the open camera-rig front
# by construction; no precision indexing needed (the brick sits on the
# platform, so only platform-to-brick geometry matters optically).
#
# Floor uses a "T-tongue" topology — front X-rail + back Z-tongue —
# rather than a closed rectangle. Three contact regions (two front-
# corner posts + one tongue tip) give a stable footprint without the
# 4-point rocking risk of a closed rectangle on an uneven bench.

# Acrylic platform plate: smallest rectangle that simultaneously
# clears the below-mirror sightline cones (≤ Ø172 at the platform
# plane per rev-1) and accommodates the largest brick footprint
# (BRICK_SPHERE_DIA = 80). 180 mm gives ~4 mm of clear acrylic margin
# outside the Ø172 sightline circle for edge clamping.
PLATFORM_ACRYLIC_SIZE = 180.0

# Acrylic plate thickness. Common cast PMMA stock thickness; gives
# enough stiffness to span 180 mm under a brick load without flex,
# and enough edge thickness to host the bonded hinge brackets'
# top face without the bond joint dominating the plate's geometry.
PLATFORM_ACRYLIC_T = 4.0

# Floor rails are coplanar with the camera rig's rail A / rail B
# (centerline y = -220, top face y = -210, bottom face y = -230).
# Both the rig and the platform frame share a single bench at
# y = -230. The Z-tongue therefore can't pass under rail B; it butts
# up against rail B's front face (CORNER_GAP separator), and rail B's
# front outrigger has been removed correspondingly.
PLATFORM_FRAME_FLOOR_Y = -220.0

# Front-post centerlines in X. ±100 sits 10 mm outboard of the
# 180 mm acrylic edge (acrylic at x ∈ [-90, +90]) so the cross-rail
# tops carry hinge bearings just outside the platform footprint, and
# clears rail A's outrigger end at x = ±270 by ~170 mm.
PLATFORM_FRAME_POST_X = 100.0

# Front-post centerline Z. -100 sits 10 mm forward of the acrylic
# front edge (z = -90), so the cross-rail centerline passes just in
# front of the platform's hinge edge.
PLATFORM_FRAME_POST_Z = -100.0

# Cross-rail centerline Y. Top face at -30 puts the hinge-bearing
# upper face just below the platform top at y = 0; bearing height
# ~25–30 mm brings the hinge bore into the acrylic edge thickness.
PLATFORM_FRAME_CROSS_RAIL_Y = -40.0

# Back-tongue tip Z. Tongue butts against rail B's new front face
# (z ≈ 18.44, flush with rail A's front face) with a CORNER_GAP
# (0.3 mm) separator — they share the same y = -220 plane, so they
# can't overlap in z. Significantly shorter than the acrylic's
# z-extent (z ∈ [-90, +90]); accepted because the platform is light
# and its CG sits forward of the new back foot.
PLATFORM_FRAME_TONGUE_Z_BACK = T_FRAME_RAIL_A_FRONT_Z - 0.3


# --- Rev-3 platform hinge bearing (slice b) ----------------------------
#
# Two printed bushing blocks bolted to the platform-frame cross-rail's
# top face. Each carries a single horizontal pin bore (along world ±X)
# for the hinge pin running across the platform's front edge. Two M5
# bolts per block drop into the cross-rail's top T-slot (T-nuts not
# modelled — stock hardware). Load cells are deferred — the rev-3
# variant trades this body for a load-cell-pocket variant on the same
# bolt pattern, with no change to platform_frame.

# Bearing block centerlines in X. ±90 lines up with the slice (c)
# acrylic platform edges (acrylic at x ∈ [-90, +90]); 10 mm inboard
# of the front-post centerlines at x = ±100 and 20 mm inboard of the
# cross-rail X-end face at x = ±110.
PLATFORM_HINGE_BEARING_X = 90.0

# Hinge pin: M5 round stock. 0.4 mm diametric clearance on the bore.
PLATFORM_HINGE_PIN_D = 5.0

# Hinge pin axis Y. Cross-rail top face at y = -30; bearing block is
# 25 mm tall (top face y = -5, just below the acrylic top at y = 0)
# with the bore at the block centerline. The slice (c) platform-side
# hinge bracket spans from the acrylic underside down to the pin.
# Numerical: PLATFORM_FRAME_CROSS_RAIL_Y + MEMBER/2 + 12.5 = -17.5.
PLATFORM_HINGE_PIN_Y = -17.5

# Hinge pin axis Z. Bearing centered on the cross-rail in Z so the
# rail-T-slot bolts enter on the rail centerline (only line where
# T-nuts can sit in the top slot). Same Z as the cross-rail / front
# posts.
PLATFORM_HINGE_PIN_Z = PLATFORM_FRAME_POST_Z


# --- Rev-3 platform acrylic + hinge bracket (slice c) ------------------
#
# The "platform" in operational terms: the moving piece that hinges
# forward to drop bricks. 180 × 180 × 4 mm PMMA plate (top face at
# y = 0) with two small printed hinge brackets bonded to its underside
# near the front edge. The brackets share their pin bore with the
# slice (b) bearing pillow-blocks; one continuous Ø5 round-stock pin
# threads through bearings + brackets together.

# Hinge-bracket centerlines in X. With a 30 mm bracket width, ±55
# puts the bracket's outboard X face at ±70 — 5 mm inboard of the
# bearing's inboard X face at ±75 (bearing is 30 mm wide centered at
# ±90). The 5 mm air gap along the pin axis lets the platform rotate
# without bearing-bracket rub. Bracket is entirely under the acrylic
# (acrylic span x ∈ [-90, +90]).
PLATFORM_HINGE_BRACKET_X = 55.0


# --- Rev-3 turret (replaces single tilting platform) -------------------
#
# 4-station rotary indexed turret described in
# `turret_design_brief.md`. Hub axis vertical at world (0, *, -130) —
# under the camera (camera bracket back face at z ≈ -231.6 sits 100 mm
# behind the hub) and on the operator side of the brick. Arm radius
# r = 130 mm puts the IMAGE platform centroid at world origin and the
# other three stations all at z ≤ 0 (operator side); none reach
# rail C at z = +240. Cam ring at platform-edge height (y ≈ 0); the
# camera rig's rails sit at y = -220 (A/B) and y = +250 (D), so
# Y-separation alone keeps the cam ring clear of the rig.
#
# Station centroids (all at y = 0):
#     IMAGE   = (0,    0, 0)        — brick centred on world origin
#     LOAD    = (+130, 0, -130)     — operator-right-and-forward
#     DUMP    = (-130, 0, -130)     — operator-left-and-forward
#     RETURN  = (0,    0, -260)     — directly forward of origin
#
# IMAGE platform's outer edge (away from hub) sits at z = +90 (closest
# point to rail C at z = +230 — 140 mm gap). RETURN platform's outer
# edge sits at z = -350 (90 mm forward of the camera-rig front face,
# which the rig's "open front" decision already accommodates).

# Hub axis Z. Centred under the camera, slightly forward of the camera
# bracket back face (z ≈ -231.6). 130 mm forward of brick → 100 mm
# forward of camera bracket — clears camera structure without
# extending the turret further into operator space than necessary.
TURRET_HUB_Z = -130.0

# Hub axis X. Centred on the world Y/Z plane (4-fold symmetry).
TURRET_HUB_X = 0.0

# Arm radius (hub axis to platform centroid). 130 mm sits in the
# middle of the design brief's 100–150 mm range. Hub-to-hinge-edge
# arm reach = TURRET_ARM_R - PLATFORM_ACRYLIC_SIZE/2 = 40 mm.
TURRET_ARM_R = 130.0

# Station centroid XZ positions (Y = 0 for all four).
TURRET_STATION_IMAGE_XZ = (0.0, 0.0)
TURRET_STATION_LOAD_XZ = (+TURRET_ARM_R, TURRET_HUB_Z)
TURRET_STATION_DUMP_XZ = (-TURRET_ARM_R, TURRET_HUB_Z)
TURRET_STATION_RETURN_XZ = (0.0, TURRET_HUB_Z - TURRET_ARM_R)


# --- Rev-3 turret_frame (slice a) --------------------------------------
#
# Freestanding 2020 floor frame on the shared bench (y = -220) hosting
# the hub support and providing the M2 → M3 → M5 mechanical interface
# (singulator at LOAD, diverter at DUMP). Replaces the rev-3
# platform_frame's interface role; the platform-frame stack stays in
# tree as historical reference until cleanup. The frame's back X-rail
# butts the camera rig's rail A front face (CORNER_GAP separator) on
# the same y = -220 bench.
#
# Topology: closed rectangle (4 perimeter rails) plus a center X-rail
# at z = TURRET_HUB_Z. The center rail is the mounting line shared by
# the hub bearing (slice b, x = 0), the singulator base (x = +130) and
# the diverter base (x = -130) — putting all three M2/M3/M5 anchors on
# one rail keeps their mounting holes coplanar with the cross-rail's
# T-slot.

TURRET_FRAME_FLOOR_Y = PLATFORM_FRAME_FLOOR_Y                    # = -220 (shared bench)

# Center X-rail Z (= hub axis Z). Hosts hub bearing + neighbour-module
# anchors on a single line.
TURRET_FRAME_CROSS_Z = TURRET_HUB_Z                              # = -130


# --- Rev-3 turret_hub (slice b) ----------------------------------------
#
# Printed pillow-block hub housing carrying a vertical Ø8 shaft on two
# 608ZZ ball bearings (8 × 22 × 7). Bolts to the slice (a) turret_frame
# cross-rail's top T-slot at the hub axis (x = TURRET_HUB_X = 0,
# z = TURRET_HUB_Z = -130). Drive coupling (motor + belt or worm)
# deferred until shaft layout is fixed.

# Stock 8 mm hardened steel shaft. Selected to match 608ZZ bearing ID.
TURRET_SHAFT_D = 8.0

# Shaft top Y in world. 100 mm above the housing top face — leaves
# room for the slice (c) arm spider to clamp around the shaft above
# the top bearing. The camera sightline at z = -130 covers world
# y > -29 (camera-to-brick line passes through y ≈ +60.7 at this z,
# with a ~90 mm cone half-radius), so any structure above y ≈ -30
# here appears in the camera frame. The slice (c) spider top is
# flush with the shaft top, so the spider sits in the camera frame
# corner; the classifier learns to ignore it.
TURRET_SHAFT_TOP_Y = -10.0


# --- Rev-3 turret_arm (slice c) ----------------------------------------
#
# Single printed "spider" body that clamps to the shaft top and
# carries the platform-side hinge for all 4 stations. NEXT_STEPS.md
# originally read "4 instances ... single printed body parameterised
# by station index" — that turned out to be the wrong topology
# (stacking 4 arms vertically on the shaft would force each platform
# to a different Y, breaking the "all 4 platforms optically
# equivalent" requirement). Replaced with a single 4-arm spider
# (90° symmetry around the hub axis) printed once.
#
# Geometry (centred on the hub axis):
#   * hub disc — central cylindrical clamp, OD chosen for set-screw
#     wall (M3 + heat-set insert), shaft bore Ø8.4 (Ø8 + 0.4 dia
#     clearance over the slice (b) shaft).
#   * 4 radial arms — rectangular beams in +X / -X / +Z / -Z from
#     the hub OD out to the tip bushing-block inner face.
#   * 4 tip bushing blocks — rectangular blocks at radial 40 mm,
#     each with one horizontal Ø5.4 pin bore aligned tangentially
#     (perpendicular to its arm's radial direction).
#   * 4 stock Ø5 pins captured in the bushing blocks; each
#     protrudes tangentially to host the slice (d) platform's
#     bonded hinge bracket.
#
# All 4 arms share a single Y span (the spider top face is flush
# with the shaft top), so all 4 platform pivot axes land at the same
# world Y → all 4 platforms can use identical bonded hinge brackets
# and reach the same y = 0 acrylic top face.

# Hinge pin axis Y (world). Chosen so the spider's top face sits
# flush with the shaft top (y = -10) and the pin lies on the spider
# centerline: top - SPIDER_HEIGHT/2 = -10 - 25/2 = -22.5. The
# slice (d) platform's bonded hinge brackets reach from acrylic
# underside (y = -4) down to this pin Y, ~18.5 mm tall.
TURRET_HINGE_PIN_Y = -22.5

# Hinge pin radial offset from hub axis. The platform centroid sits
# at TURRET_ARM_R = 130 from the hub; its hub-side edge is
# PLATFORM_ACRYLIC_SIZE/2 = 90 closer in. So the hinge pin axis at
# the inner edge sits at radial 130 - 90 = 40 mm from hub axis.
TURRET_HINGE_PIN_RADIAL = TURRET_ARM_R - PLATFORM_ACRYLIC_SIZE / 2  # = 40

# Hinge pin diameter — M5 stock round (matches the rev-3
# PLATFORM_HINGE_PIN_D so existing M5 round stock applies).
TURRET_HINGE_PIN_D = 5.0
