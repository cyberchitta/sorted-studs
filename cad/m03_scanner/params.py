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
# Sized to the per-camera imaged region from the rev-5 4-cam direct
# topology (sizing.py at lens=3.6 / brick_max=80 / scatter=±15):
# imaged region = 110 × 110 mm + 5 mm PMMA edge grip per side =
# 120 × 120 mm. The prior 180 mm was sized against a Ø172 mirror-era
# sightline cone (rev-1) that no longer exists. Slice c3 (2026-05-05)
# shrunk this 180 → 120 alongside the turret arm shrink.
#
# 4 mm thickness gives enough stiffness to span 120 mm under a brick
# load without flex, and enough edge thickness to host bonded brackets
# without the bond joint dominating geometry.
#
# Used by the rev-3 single-platform stack and the rev-4 turret platform
# cartridges. Each unit duplicates these in its own params.py for
# standalone use; assembly.py asserts the values match.

PLATFORM_ACRYLIC_SIZE = 120.0
PLATFORM_ACRYLIC_T = 4.0


# --- Inter-unit interface (camera rig ↔ turret) ------------------------

# Shared bench Z. The turret_frame floor sits on this single bench at
# z = -220. Slice c2 (2026-05-05) raised the camera rig's rail A from
# z = -220 to z = -131 (rig shrink, r=160), so the rig now floats
# ~80 mm above the bench until a future floor-feet slice bridges the
# gap; SHARED_BENCH_Z stays as the project-wide reference for the
# eventual workbench surface.
SHARED_BENCH_Z = -220.0
