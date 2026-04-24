"""
M03 scanner — top-level assembly.

Composes every rev-1 sub-assembly in world coordinates and runs the
cross-boundary interference checks. Per-sub-system mechanism + per-part
printability live in each sub-assembly's standalone `assembly.py`; this
file only covers pairs that span two sub-systems.

Sub-assemblies placed here:

  * enclosure              — six-panel rectangular box (world frame)
  * platform               — Ø360 collar + Ø180 optical disc + load cells
  * camera_bracket         — Pi HQ Camera + 2.8 mm lens + printed plate
  * camera_bracket_adapter — tilted wedge between front wall and bracket
  * mirror_mount (×3)      — rigid rev 1 mount at each mirror pose
                             (back-upper, below-left, below-right)

Placement references come from `params.py`; those values are copied
from `docs/m03-multi-view-scanner.md`. If the reference doc's
parameter table changes, update `params.py` to match — never inline
mirror/camera coordinates in this file.

Cross-boundary checks (rev 1):

  * Every internal part vs every enclosure panel. Guards against a
    rev bump in any sub-assembly pushing a part through a wall.
  * Every mirror mount vs the camera bracket (mirrors sit at ≥ 250 mm
    from the camera; assertion is a regression guard against future
    mount growth into the direct-viewpoint cone).
  * Every mirror mount vs the platform (below-mirrors sit at
    y = -140 mm, ≥ 120 mm below the platform collar; assertion guards
    against future mirror-support brackets climbing up into the
    platform / load-cell envelope).

Mirror-to-mirror interference is not asserted: the three mirrors are
fixed by the tetrahedral optimiser in `klods-syn/nbs/200_viewpoints.md`
and their plate extents don't approach each other (closest centre pair
is ≈ 460 mm apart). A check would pass trivially and add noise.

What the rev-1 enclosure does NOT yet provide — noted in `NEXT_STEPS.md`:
mount-interface bolt patterns for any sub-assembly, cut-outs for the
chute / eject slot / access door, or ring-light and ToF mount features.
The camera bracket and mirror mounts sit in space at their target
poses for this rev; rev 2 adds the walls' mount interfaces (likely
tilted standoffs or tilted sub-panels, since none of the mounts is
flush against an axis-aligned world wall).
"""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

from build123d import Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

# `params.py` sits next to this file, so a plain module-level import
# works once this directory is on sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from params import (  # noqa: E402
    ADAPTER_POS,
    CAMERA_POS,
    CAMERA_ROT_DEG,
    MIRROR_BACK_UPPER_NORMAL,
    MIRROR_BACK_UPPER_POS,
    MIRROR_BELOW_LEFT_NORMAL,
    MIRROR_BELOW_LEFT_POS,
    MIRROR_BELOW_RIGHT_NORMAL,
    MIRROR_BELOW_RIGHT_POS,
    rotation_for_normal,
)


# --- Sub-assembly loading -----------------------------------------------
#
# Each sub-directory holds a bare `assembly.py` with no `__init__.py`.
# Three of the four names (`platform` in particular) collide with stdlib
# modules, so the namespace-package trick used in
# `cad/m05_diverter/assembly.py` doesn't survive. Load each by file
# path via importlib.util — same pattern for all four keeps the
# top-level uniform.

def _load(subdir: str) -> ModuleType:
    path = Path(__file__).resolve().parent / subdir / "assembly.py"
    spec = importlib.util.spec_from_file_location(
        f"m03_scanner_{subdir}", path
    )
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


_enc = _load("enclosure")
_plat = _load("platform")
_cam = _load("camera_bracket")
_adapter = _load("camera_bracket_adapter")
_mirror = _load("mirror_mount")


# --- Placement constants ------------------------------------------------

# The platform's local +Z points out of the top face; the scanner's
# world "up" is +Y. Rot(-90, 0, 0) maps local +Z onto world +Y and
# keeps the local origin at the world origin. See platform/assembly.py.
_PLATFORM_LOC = Rot(-90, 0, 0)

# Camera placement: local +Z (lens axis) rotated to 25° below horizontal
# toward +Z; origin translated to the PCB rear plane in world.
_CAMERA_LOC = Pos(*CAMERA_POS) * Rot(*CAMERA_ROT_DEG)

# Adapter placement: world-axis-aligned (no rotation). Local origin
# at ADAPTER_POS — the back face sits flush against the front-wall
# inner face, and the tilted front face is coplanar with the bracket
# back face.
_ADAPTER_LOC = Pos(*ADAPTER_POS)

# Mirror placements: local +Z = mirror outward normal; translation
# places the reflective face centre at the mirror's world coordinates.
_MIRROR_POSES = {
    "mirror_back_upper": (
        Pos(*MIRROR_BACK_UPPER_POS)
        * rotation_for_normal(MIRROR_BACK_UPPER_NORMAL)
    ),
    "mirror_below_left": (
        Pos(*MIRROR_BELOW_LEFT_POS)
        * rotation_for_normal(MIRROR_BELOW_LEFT_NORMAL)
    ),
    "mirror_below_right": (
        Pos(*MIRROR_BELOW_RIGHT_POS)
        * rotation_for_normal(MIRROR_BELOW_RIGHT_NORMAL)
    ),
}


# --- Assembly composition -----------------------------------------------

assembly = Assembly()

# Enclosure panels. Flat in world frame; no additional location needed.
_PANEL_NAMES = (
    "wall_floor", "wall_ceiling",
    "wall_back", "wall_front",
    "wall_left", "wall_right",
)
for _name in _PANEL_NAMES:
    assembly = assembly.add(_name, getattr(_enc, _name)())

# Platform (collar + disc + three load cells). Sibling sub-assembly
# names aren't prefixed since these are the only collar / disc / load
# cells in the whole rig.
assembly = (
    assembly
    .add("collar", _plat.collar(), location=_PLATFORM_LOC)
    .add("disc", _plat.disc_stub(), location=_PLATFORM_LOC)
)
_LOAD_CELL_NAMES = tuple(
    f"load_cell_{i}" for i in range(len(_plat.LOAD_CELL_AZ_DEG))
)
for _idx, _az in enumerate(_plat.LOAD_CELL_AZ_DEG):
    # _load_cell_location returns the beam pose in the platform's local
    # frame; compose _PLATFORM_LOC on the outside to lift into world.
    assembly = assembly.add(
        _LOAD_CELL_NAMES[_idx],
        _plat.load_cell_stub(),
        location=_PLATFORM_LOC * _plat._load_cell_location(_az),
    )

# Camera bracket + PCB + lens stand-in.
assembly = (
    assembly
    .add("camera_plate", _cam.bracket(), location=_CAMERA_LOC)
    .add("camera_pcb", _cam.camera_pcb_stub(), location=_CAMERA_LOC)
    .add("camera_lens", _cam.lens_stub(), location=_CAMERA_LOC)
)

# Tilted spacer between front wall and bracket back face.
assembly = assembly.add(
    "camera_adapter", _adapter.adapter(), location=_ADAPTER_LOC
)

# Three rigid mirror mounts. Each contributes a bracket + mirror stand-in
# under a unique prefix.
_MIRROR_PART_NAMES = []
for _prefix, _loc in _MIRROR_POSES.items():
    _bracket_name = f"{_prefix}_bracket"
    _mirror_name = f"{_prefix}_glass"
    _MIRROR_PART_NAMES.extend([_bracket_name, _mirror_name])
    assembly = (
        assembly
        .add(_bracket_name, _mirror.bracket(), location=_loc)
        .add(_mirror_name, _mirror.mirror_stub(), location=_loc)
    )


# --- Cross-boundary assertions ------------------------------------------

# Internal parts that must clear every enclosure panel.
_INTERNAL_NAMES = (
    ["collar", "disc", *_LOAD_CELL_NAMES,
     "camera_plate", "camera_pcb", "camera_lens", "camera_adapter"]
    + _MIRROR_PART_NAMES
)

for _internal in _INTERNAL_NAMES:
    for _panel in _PANEL_NAMES:
        assembly = assembly.assert_no_interference(_internal, _panel)

# Mirror mounts vs camera-bracket triplet. Regression guard — the
# mirrors are well clear today, but any future growth in the kinematic
# mount stack or in the lens envelope could close the gap.
for _mirror_part in _MIRROR_PART_NAMES:
    for _cam_part in ("camera_plate", "camera_pcb", "camera_lens"):
        assembly = assembly.assert_no_interference(_mirror_part, _cam_part)

# Mirror mounts vs platform. Same logic — below-mirrors currently sit
# ≥ 120 mm below the platform collar, but a future floor-bracket rev
# could extend the mount up toward the platform aperture.
for _mirror_part in _MIRROR_PART_NAMES:
    for _plat_part in ["collar", "disc", *_LOAD_CELL_NAMES]:
        assembly = assembly.assert_no_interference(_mirror_part, _plat_part)

# Adapter vs camera-bracket triplet. Tangent contact at the tilted
# face is intended (bracket back face = adapter front face); the
# assertion enforces tangency-without-overlap. Also catches any drift
# in CAMERA_POS, CAMERA_ROT_DEG, CAMERA_BRACKET_PLATE_T, or
# ADAPTER_POS that would push the bracket into or away from the
# adapter face.
for _cam_part in ("camera_plate", "camera_pcb", "camera_lens"):
    assembly = assembly.assert_no_interference("camera_adapter", _cam_part)

# Adapter vs every mirror mount. Adapter sits at world y ∈ [73, 142]
# and z ∈ [-290, -215]; mirrors are well clear today, but the
# below-mirror floor-bracket rev planned in NEXT_STEPS.md could grow
# inward toward the front wall.
for _mirror_part in _MIRROR_PART_NAMES:
    assembly = assembly.assert_no_interference("camera_adapter", _mirror_part)

# Adapter vs platform. Adapter sits forward of the platform; this is
# a regression guard for any future shuttle / disc-retainer feature
# growth.
for _plat_part in ["collar", "disc", *_LOAD_CELL_NAMES]:
    assembly = assembly.assert_no_interference("camera_adapter", _plat_part)


if __name__ == "__main__":
    check(assembly, out="outputs")
