"""
M03 scanner — top-level inter-unit interface and rev-3 platform-stack
constants.

After the Phase-2 frame refactor, the camera rig and the turret each
own a unit-local frame and a unit-local `params.py`. This top-level
file now holds only:

  * Project-wide print floor / reference values (`WALL_MIN`,
    `BRICK_SPHERE_DIA`).
  * Stock specs shared between the two units (`PLATFORM_ACRYLIC_*`).
    Each unit's `params.py` redefines these for standalone use; the
    top-level `assembly.py` asserts agreement.
  * Inter-unit interface: the world-frame placement of the turret and
    its butt clearance against the camera rig's rail A.
  * A re-export of `T_FRAME_RAIL_A_FRONT_Z` from `camera_rig.params/`,
    so the rev-3 single-platform stack (still living at this level
    until the cleanup commit) keeps its existing import working.
  * Constants for the rev-3 platform stack (`platform_frame/`,
    `platform_hinge_bearing/`, `platform_acrylic/`). Scheduled for
    deletion when the rev-3 stack itself is deleted.
"""

# --- Project-wide policy -----------------------------------------------

# FDM printability floor — minimum wall thickness any printed
# sub-assembly may carry. Each unit's `params.py` defines its own copy
# (matched against this value by `assembly.py`).
WALL_MIN = 1.5

# 90% of LEGO parts fit within an 80 mm sphere. Drives the minimum
# platform aperture and the camera-rig clearance envelope. Optical
# sightline cones at the platform plane (≤ Ø172 per rev-1's design)
# dominate this for acrylic sizing.
BRICK_SPHERE_DIA = 80.0


# --- Acrylic platform stock spec ---------------------------------------
#
# Smallest rectangle that simultaneously clears the below-mirror
# sightline cones (≤ Ø172 at the platform plane per rev-1) and
# accommodates the largest brick footprint (`BRICK_SPHERE_DIA = 80`).
# 180 mm gives ~4 mm of clear acrylic margin outside the Ø172 sightline
# circle for edge clamping. 4 mm thickness gives enough stiffness to
# span 180 mm under a brick load without flex, and enough edge thickness
# to host bonded brackets without the bond joint dominating geometry.
#
# Used by the rev-3 single-platform stack and the rev-4 turret platform
# cartridges. Each unit duplicates these in its own params.py for
# standalone use; assembly.py asserts the values match.

PLATFORM_ACRYLIC_SIZE = 180.0
PLATFORM_ACRYLIC_T = 4.0


# --- Inter-unit interface (camera rig ↔ turret) ------------------------

# Shared bench Y. Both the camera rig's rail A/B and the turret_frame
# floor sit on this single bench at y = -220.
SHARED_BENCH_Y = -220.0

# Butt clearance between the camera rig's rail A front face and the
# turret_frame's back rail back face when the turret is placed in
# world. The boundary assertion in `assembly.py` enforces this.
RIG_TURRET_BUTT_GAP = 0.3


# --- Camera-rig constant re-exported for the rev-3 platform stack ------
#
# The rev-3 `platform_frame/` sub-assembly (still at this level until
# the platform-stack cleanup) reads `T_FRAME_RAIL_A_FRONT_Z` to derive
# its tongue back face. The canonical owner is `camera_rig.params`;
# this re-export lets the rev-3 stack keep its `from params import …`
# line without sibling-importing. Goes away with the platform-stack
# cleanup.

from camera_rig.params import T_FRAME_RAIL_A_FRONT_Z  # noqa: E402, F401


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

# Floor rails are coplanar with the camera rig's rail A / rail B
# (centerline y = -220, top face y = -210, bottom face y = -230).
# Both the rig and the platform frame share a single bench at
# y = -230. The Z-tongue therefore can't pass under rail B; it butts
# up against rail B's front face (CORNER_GAP separator), and rail B's
# front outrigger has been removed correspondingly.
PLATFORM_FRAME_FLOOR_Y = SHARED_BENCH_Y                  # = -220

# Cross-rail centerline Y. Top face at -30 puts the hinge-bearing
# upper face just below the platform top at y = 0; bearing height
# ~25–30 mm brings the hinge bore into the acrylic edge thickness.
PLATFORM_FRAME_CROSS_RAIL_Y = -40.0


# --- Rev-3 platform hinge bearing (slice b) ----------------------------
#
# Two printed bushing blocks bolted to the platform-frame cross-rail's
# top face. Each carries a single horizontal pin bore (along world ±X)
# for the hinge pin running across the platform's front edge. Two M5
# bolts per block drop into the cross-rail's top T-slot (T-nuts not
# modelled — stock hardware). Load cells are deferred — the rev-3
# variant trades this body for a load-cell-pocket variant on the same
# bolt pattern, with no change to platform_frame.

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
PLATFORM_HINGE_PIN_Z = -100.0
