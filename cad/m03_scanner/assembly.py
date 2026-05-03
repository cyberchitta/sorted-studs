"""
M03 scanner — rev-4 top-level assembly.

Composes two relocatable units in world coordinates and runs only
cross-unit checks here:

  * camera_rig/ — placed at world identity (rig-local frame matches
    world). Internal composition + within-unit checks live in
    `camera_rig/assembly.py`.
  * turret/ — placed at world `Pos(0, -TURRET_ARM_R, 0)` (turret-local
    hub axis lands at world `(0, -185, *)`, IMAGE platform centroid at
    world origin). Internal composition + within-unit checks live in
    `turret/assembly.py`.

Cross-unit checks at this level:

  1. Sanity: each unit's `WALL_MIN` and `PLATFORM_ACRYLIC_*` mirror
     the top-level values. Drift is a hard error.
  2. Boundary: the turret's back rail back face sits exactly
     `RIG_TURRET_BUTT_GAP` forward of the camera rig's rail A front
     face after the turret is placed in world. Both share the
     `SHARED_BENCH_Z = -220` bench, so a Y-overlap would crash in 3D.
  3. Interference: every turret part vs every camera-rig part — a
     comprehensive regression guard. Catches drift in either subsystem
     that would push a turret part into a rig part.

The rev-3 single-platform stack (`platform_frame/`,
`platform_hinge_bearing/`, `platform_acrylic/`) was deleted in the
rev-3 platform-stack cleanup commit; see
`superseded_platform_designs.md` for the retrieval commit hash.
"""

import sys
from pathlib import Path

from build123d import Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

sys.path.insert(0, str(Path(__file__).resolve().parent))

from params import (  # noqa: E402
    PLATFORM_ACRYLIC_SIZE,
    PLATFORM_ACRYLIC_T,
    RIG_TURRET_BUTT_GAP,
    SHARED_BENCH_Z,
    WALL_MIN,
)
from camera_rig import assembly as _camera_rig  # noqa: E402
from camera_rig.params import T_FRAME_RAIL_A_FRONT_Y  # noqa: E402
from camera_rig.params import WALL_MIN as _RIG_WALL_MIN  # noqa: E402
from design_systems import DESIGN_SYSTEM  # noqa: E402
from turret import assembly as _turret  # noqa: E402
from turret.params import (  # noqa: E402
    PLATFORM_ACRYLIC_SIZE as _TURRET_ACRYLIC_SIZE,
    PLATFORM_ACRYLIC_T as _TURRET_ACRYLIC_T,
    TURRET_ARM_R,
    TURRET_FRAME_BACK_Y as _TURRET_BACK_Y_LOCAL,
    TURRET_FRAME_FLOOR_Z as _TURRET_FLOOR_Z,
)
from turret.params import WALL_MIN as _TURRET_WALL_MIN  # noqa: E402


# --- Cross-unit consistency asserts ------------------------------------
#
# Each unit defines its own copy of project-policy constants so it can
# stand alone without reaching upward. Drift between copies is caught
# here.

assert WALL_MIN == _RIG_WALL_MIN == _TURRET_WALL_MIN, (
    f"WALL_MIN mismatch across units: top={WALL_MIN}, "
    f"camera_rig={_RIG_WALL_MIN}, turret={_TURRET_WALL_MIN}"
)
assert PLATFORM_ACRYLIC_SIZE == _TURRET_ACRYLIC_SIZE, (
    f"PLATFORM_ACRYLIC_SIZE mismatch: top={PLATFORM_ACRYLIC_SIZE}, "
    f"turret={_TURRET_ACRYLIC_SIZE}"
)
assert PLATFORM_ACRYLIC_T == _TURRET_ACRYLIC_T, (
    f"PLATFORM_ACRYLIC_T mismatch: top={PLATFORM_ACRYLIC_T}, "
    f"turret={_TURRET_ACRYLIC_T}"
)
assert _TURRET_FLOOR_Z == SHARED_BENCH_Z, (
    f"turret floor Z ({_TURRET_FLOOR_Z}) ≠ SHARED_BENCH_Z "
    f"({SHARED_BENCH_Z}); both units must sit on the same bench"
)


# --- Boundary assert: turret back rail butts rig rail A ---------------
#
# Turret is placed in world at `Pos(0, -TURRET_ARM_R, 0)` (translation
# only, no rotation). The turret's back rail back face is at
# turret-local Y = TURRET_FRAME_BACK_Y + MEMBER/2; in world that lands
# at (TURRET_FRAME_BACK_Y + MEMBER/2) - TURRET_ARM_R. Must equal
# T_FRAME_RAIL_A_FRONT_Y - RIG_TURRET_BUTT_GAP within numerical noise.

_MEMBER = 20.0          # 2020 extrusion (matches turret_frame.MEMBER)
_TURRET_BACK_FACE_WORLD_Y = (
    _TURRET_BACK_Y_LOCAL + _MEMBER / 2 - TURRET_ARM_R
)
_EXPECTED_BACK_FACE_WORLD_Y = T_FRAME_RAIL_A_FRONT_Y - RIG_TURRET_BUTT_GAP
assert abs(_TURRET_BACK_FACE_WORLD_Y - _EXPECTED_BACK_FACE_WORLD_Y) < 1e-6, (
    f"turret back rail back face world Y ({_TURRET_BACK_FACE_WORLD_Y:.4f}) "
    f"≠ rig rail A front face minus butt gap "
    f"({_EXPECTED_BACK_FACE_WORLD_Y:.4f}); update TURRET_FRAME_BACK_Y "
    f"in turret/params.py"
)


# --- Design-system coverage --------------------------------------------
#
# Every named camera-rig and turret part must appear in the active
# design system's tagging dict (`None` allowed for hardware / optical
# glass / acrylic — those keep their material's intrinsic color). A
# part added without a tag fails here instead of silently rendering
# off-palette. Swapping `DESIGN_SYSTEM` to a palette with missing
# tags also fails here, so the swap surface is loud.

_DESIGN_SYSTEM_PARTS = (
    list(_camera_rig.CAMERA_RIG_PART_NAMES) + list(_turret.TURRET_PART_NAMES)
)
_untagged = [n for n in _DESIGN_SYSTEM_PARTS if n not in DESIGN_SYSTEM.tag_of]
assert not _untagged, (
    f"{len(_untagged)} m03 part(s) missing a tag for design system "
    f"'{DESIGN_SYSTEM.name}' in design_systems.py: {_untagged}"
)


# --- Compose the two units in world ------------------------------------

assembly = Assembly()

# Camera rig at world identity. Run the rig's internal composition
# (no checks added at this level — those live in camera_rig/assembly.py
# and are exercised by `khana check camera_rig/assembly.py`).
assembly = _camera_rig.add_camera_rig(assembly)

# Turret placed at world Pos(0, -ARM_R, 0) — translation only.
_TURRET_WORLD_POSE = Pos(0, -TURRET_ARM_R, 0)
assembly = _turret.add_turret(assembly, base=_TURRET_WORLD_POSE)


# --- Cross-unit interference: every turret part vs every rig part -----

for turret_part in _turret.TURRET_PART_NAMES:
    for rig_part in _camera_rig.CAMERA_RIG_PART_NAMES:
        assembly = assembly.assert_no_interference(turret_part, rig_part)


if __name__ == "__main__":
    check(assembly, out="outputs")
