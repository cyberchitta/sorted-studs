"""
M03 scanner — top-level inter-unit interface constants.

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

# Shared bench Z. Both the camera rig's rail A/B and the turret_frame
# floor sit on this single bench at z = -220.
SHARED_BENCH_Z = -220.0

# Butt clearance between the camera rig's rail A front face and the
# turret_frame's back rail back face when the turret is placed in
# world. The boundary assertion in `assembly.py` enforces this.
RIG_TURRET_BUTT_GAP = 0.3
