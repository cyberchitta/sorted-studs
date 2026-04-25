"""
M03 scanner — rev-3 top-level assembly (in progress).

Composes the rev-3 sub-assemblies that exist so far in world
coordinates and runs cross-boundary interference checks. Per-sub-
system mechanism + per-part printability live in each sub-assembly's
standalone `assembly.py`; this file only covers pairs that span two
sub-systems.

Slices placed here today:

  * skeleton_t_frame              — rev-3 sparse 4-rail T-frame +
                                    two corner braces. Replaces the
                                    rev-2 12-edge cube `../skeleton/`.
  * mirror_pad_clamp (×3)         — rev-3 short-stub mirror clamp.
                                    Below-left + below-right grip
                                    rail A's +Y face; back-upper
                                    grips rail C's -Z face. Replaces
                                    the rev-2 mirror_mount_rail_clamp.
  * mirror_mount (×3)             — rev-1 rigid mount, carried
                                    forward unchanged (its 94 × 94
                                    bolt pattern is the rev-3 pad
                                    clamp's pad-bolt interface).
  * camera_bracket                — rev-1 bracket, carried forward
                                    unchanged. Floats in world space
                                    until slice (c) `camera_pad_clamp/`
                                    lands; cross-boundary clamp
                                    assertions are stubbed until then.
  * platform + load_cell_standoff (×3)
                                    — rev-1 components carried forward
                                    until the rev-3 platform slices
                                    (`platform_frame/`,
                                    `platform_pillow_block/`,
                                    `platform_hinged/`) land. The
                                    standoffs were rev-1-aligned to
                                    enclosure-floor anchors that no
                                    longer exist; they remain in the
                                    tree at their original world
                                    poses but are physically
                                    "floating".

Slices NOT yet placed (see `NEXT_STEPS.md` Suggested next CAD moves):
  * camera_pad_clamp/             — slice (c)
  * frame_gusset_t/, _l/          — slice (d)
  * platform_frame/, _pillow_block/, _hinged/  — slice (2)

The rev-1/rev-2 reference top-level lives at `assembly_rev2.py`.

Cross-boundary checks:
  * Every internal part vs every skeleton_t_frame member (regression
    guard against any sub-assembly drift pushing a part into a rail).
  * Mirror pad clamps vs the supported mirror mounts — tangent
    contact at the pad face (`assert_no_interference` catches drift
    past MOUNT_GAP), and regression-only against the other two.
  * Mirror mounts vs camera bracket and vs platform — same regression
    guards as the rev-2 top-level.

Mirror-to-mirror interference is not asserted: the three mirrors are
fixed by the tetrahedral optimiser in
`klods-syn/nbs/200_viewpoints.md` and their plate extents don't
approach each other (closest centre pair is ≈ 460 mm apart).
"""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

from build123d import Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

sys.path.insert(0, str(Path(__file__).resolve().parent))

from params import (  # noqa: E402
    CAMERA_POS,
    CAMERA_ROT_DEG,
    LOAD_CELL_STANDOFF_HEADING_DEG,
    LOAD_CELL_STANDOFF_POSITIONS,
    MIRROR_BACK_UPPER_NORMAL,
    MIRROR_BACK_UPPER_POS,
    MIRROR_BELOW_LEFT_NORMAL,
    MIRROR_BELOW_LEFT_POS,
    MIRROR_BELOW_RIGHT_NORMAL,
    MIRROR_BELOW_RIGHT_POS,
    RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
    RAIL_PAD_CLAMP_BACK_UPPER_POS,
    RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
    RAIL_PAD_CLAMP_BELOW_LEFT_POS,
    RAIL_PAD_CLAMP_BELOW_RIGHT_CLAMP_ROT,
    RAIL_PAD_CLAMP_BELOW_RIGHT_POS,
    horizontal_x_roll_deg,
    rotation_for_normal,
)


# --- Sub-assembly loading -----------------------------------------------
#
# Same importlib trick as the rev-2 top-level: each sub-directory
# holds a bare `assembly.py` with no `__init__.py`, and `platform`
# in particular collides with stdlib so the namespace-package
# approach doesn't survive. Load each by file path.

def _load(subdir: str) -> ModuleType:
    path = Path(__file__).resolve().parent / subdir / "assembly.py"
    spec = importlib.util.spec_from_file_location(
        f"m03_scanner_{subdir}", path
    )
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


_skel = _load("skeleton_t_frame")
_plat = _load("platform")
_cam = _load("camera_bracket")
_mirror = _load("mirror_mount")
_standoff = _load("load_cell_standoff")
_pad_clamp = _load("mirror_pad_clamp")


# --- Placement constants ------------------------------------------------

# The platform's local +Z points out of the top face; the scanner's
# world "up" is +Y. Rot(-90, 0, 0) maps local +Z onto world +Y.
_PLATFORM_LOC = Rot(-90, 0, 0)

# Camera placement: local +Z (lens axis) rotated to 25° below
# horizontal toward +Z; origin translated to the PCB rear plane.
_CAMERA_LOC = Pos(*CAMERA_POS) * Rot(*CAMERA_ROT_DEG)

# Mirror placements — local +Z = mirror outward normal. Roll picked
# by `horizontal_x_roll_deg` so mount-local +X lands on a world-
# horizontal direction.
_MIRROR_POSES = {
    "mirror_back_upper": (
        Pos(*MIRROR_BACK_UPPER_POS)
        * rotation_for_normal(
            MIRROR_BACK_UPPER_NORMAL,
            roll_deg=horizontal_x_roll_deg(MIRROR_BACK_UPPER_NORMAL),
        )
    ),
    "mirror_below_left": (
        Pos(*MIRROR_BELOW_LEFT_POS)
        * rotation_for_normal(
            MIRROR_BELOW_LEFT_NORMAL,
            roll_deg=horizontal_x_roll_deg(MIRROR_BELOW_LEFT_NORMAL),
        )
    ),
    "mirror_below_right": (
        Pos(*MIRROR_BELOW_RIGHT_POS)
        * rotation_for_normal(
            MIRROR_BELOW_RIGHT_NORMAL,
            roll_deg=horizontal_x_roll_deg(MIRROR_BELOW_RIGHT_NORMAL),
        )
    ),
}

# Rev-3 mirror pad-clamp placements (one per mirror). Each spec
# pairs the world placement with the data needed to derive the
# clamp's local mount-bolt pattern.
_PAD_CLAMP_SPECS = {
    "pad_clamp_below_left": {
        "clamp_pos": RAIL_PAD_CLAMP_BELOW_LEFT_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
        "mirror_pos": MIRROR_BELOW_LEFT_POS,
        "mirror_normal": MIRROR_BELOW_LEFT_NORMAL,
    },
    "pad_clamp_below_right": {
        "clamp_pos": RAIL_PAD_CLAMP_BELOW_RIGHT_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BELOW_RIGHT_CLAMP_ROT,
        "mirror_pos": MIRROR_BELOW_RIGHT_POS,
        "mirror_normal": MIRROR_BELOW_RIGHT_NORMAL,
    },
    "pad_clamp_back_upper": {
        "clamp_pos": RAIL_PAD_CLAMP_BACK_UPPER_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
        "mirror_pos": MIRROR_BACK_UPPER_POS,
        "mirror_normal": MIRROR_BACK_UPPER_NORMAL,
    },
}

# Mirror name → corresponding pad-clamp name. Used to label which
# clamp / mount pair is the supported tangent contact (vs the other
# two, which are regression guards).
_MIRROR_TO_CLAMP = {
    "mirror_back_upper": "pad_clamp_back_upper",
    "mirror_below_left": "pad_clamp_below_left",
    "mirror_below_right": "pad_clamp_below_right",
}

# Load-cell standoff placements (rev-1, carried forward). Local +Y =
# world +Y; local +X aligns with the supported beam's radial outward
# direction under Ry(heading), where heading = the load-cell's
# platform-local azimuth (derivation in params.py).
_STANDOFF_LOCS = [
    Pos(*pos) * Rot(0, heading, 0)
    for pos, heading in zip(
        LOAD_CELL_STANDOFF_POSITIONS,
        LOAD_CELL_STANDOFF_HEADING_DEG,
    )
]


# --- Assembly composition -----------------------------------------------

assembly = Assembly()

# Rev-3 sparse T-frame skeleton. Each member in `_skel._MEMBERS` is
# already positioned at its world location (the sub-assembly bakes
# the `Pos(...)` into each part).
_SKELETON_NAMES = tuple(_skel._MEMBERS.keys())
for _name, _part in _skel._MEMBERS.items():
    assembly = assembly.add(_name, _part)

# Platform — rev-1 components carried forward.
assembly = (
    assembly
    .add("collar", _plat.collar(), location=_PLATFORM_LOC)
    .add("disc", _plat.disc_stub(), location=_PLATFORM_LOC)
)
_LOAD_CELL_NAMES = tuple(
    f"load_cell_{i}" for i in range(len(_plat.LOAD_CELL_AZ_DEG))
)
for _idx, _az in enumerate(_plat.LOAD_CELL_AZ_DEG):
    assembly = assembly.add(
        _LOAD_CELL_NAMES[_idx],
        _plat.load_cell_stub(),
        location=_PLATFORM_LOC * _plat._load_cell_location(_az),
    )

# Camera bracket + PCB + lens stand-in (rev-1, floating until slice (c)).
assembly = (
    assembly
    .add("camera_plate", _cam.bracket(), location=_CAMERA_LOC)
    .add("camera_pcb", _cam.camera_pcb_stub(), location=_CAMERA_LOC)
    .add("camera_lens", _cam.lens_stub(), location=_CAMERA_LOC)
)

# Three rigid mirror mounts — bracket + mirror-glass stand-in per pose.
_MIRROR_PART_NAMES = []
_MIRROR_BRACKET_NAMES = {}      # mirror prefix → bracket part name
for _prefix, _loc in _MIRROR_POSES.items():
    _bracket_name = f"{_prefix}_bracket"
    _mirror_name = f"{_prefix}_glass"
    _MIRROR_PART_NAMES.extend([_bracket_name, _mirror_name])
    _MIRROR_BRACKET_NAMES[_prefix] = _bracket_name
    assembly = (
        assembly
        .add(_bracket_name, _mirror.bracket(), location=_loc)
        .add(_mirror_name, _mirror.mirror_stub(), location=_loc)
    )

# Three rev-3 mirror pad clamps. Each instance gets its own clamp
# body with the bolt-hole pattern derived from `mount_in_clamp`.
_PAD_CLAMP_NAMES = tuple(_PAD_CLAMP_SPECS.keys())
for _name, _spec in _PAD_CLAMP_SPECS.items():
    _loc = Pos(*_spec["clamp_pos"]) * _spec["clamp_rot"]
    _mount_in_clamp = _pad_clamp.mount_in_clamp_for(**_spec)
    assembly = assembly.add(
        _name,
        _pad_clamp.clamp(mount_in_clamp=_mount_in_clamp),
        location=_loc,
    )

# Three load-cell standoffs (rev-1, carried forward). Names mirror
# the platform's `load_cell_{i}` indices so cross-boundary assertions
# pair cleanly.
_STANDOFF_NAMES = tuple(
    f"load_cell_standoff_{i}" for i in range(len(_STANDOFF_LOCS))
)
for _name, _loc in zip(_STANDOFF_NAMES, _STANDOFF_LOCS):
    assembly = assembly.add(_name, _standoff.bracket(), location=_loc)


# --- Cross-boundary assertions ------------------------------------------

_PLATFORM_NAMES = ["collar", "disc", *_LOAD_CELL_NAMES]
_CAMERA_BRACKET_NAMES = ("camera_plate", "camera_pcb", "camera_lens")
_INTERNAL_NAMES = (
    _PLATFORM_NAMES
    + list(_CAMERA_BRACKET_NAMES)
    + _MIRROR_PART_NAMES
    + list(_STANDOFF_NAMES)
    + list(_PAD_CLAMP_NAMES)
)

# Internal parts vs every skeleton_t_frame member.
for _internal in _INTERNAL_NAMES:
    for _member in _SKELETON_NAMES:
        assembly = assembly.assert_no_interference(_internal, _member)

# Mirror mounts vs camera bracket triplet — regression guard.
for _mirror_part in _MIRROR_PART_NAMES:
    for _cam_part in _CAMERA_BRACKET_NAMES:
        assembly = assembly.assert_no_interference(_mirror_part, _cam_part)

# Mirror mounts vs platform — regression guard.
for _mirror_part in _MIRROR_PART_NAMES:
    for _plat_part in _PLATFORM_NAMES:
        assembly = assembly.assert_no_interference(_mirror_part, _plat_part)

# Mirror pad-clamps vs every mirror-mount part. The clamp's pad face
# is coplanar with its supported mirror mount plate back face minus
# MOUNT_GAP (tangent treatment); the cross-boundary assertion catches
# drift past that 0.3 mm gap.
for _clamp_name in _PAD_CLAMP_NAMES:
    for _mirror_part in _MIRROR_PART_NAMES:
        assembly = assembly.assert_no_interference(_clamp_name, _mirror_part)

    # vs platform parts — below-mirror clamps at y ∈ [-220, ~-98],
    # back-upper clamp at y ∈ [...]; platform at y ≈ 0; all
    # regression guards.
    for _plat_part in _PLATFORM_NAMES:
        assembly = assembly.assert_no_interference(_clamp_name, _plat_part)

    # vs camera bracket triplet — below-mirror clamps at world
    # |x| ≥ 200, back-upper clamp at x = 0 z ≥ 215; camera at z ≈
    # -226. Well clear.
    for _cam_part in _CAMERA_BRACKET_NAMES:
        assembly = assembly.assert_no_interference(_clamp_name, _cam_part)

# Mirror pad-clamp vs mirror pad-clamp — below-left/right sit in
# opposite X hemispheres; back-upper sits on rail C with no X overlap
# with the below-mirror clamps. Regression guard.
for _i in range(len(_PAD_CLAMP_NAMES)):
    for _j in range(_i + 1, len(_PAD_CLAMP_NAMES)):
        assembly = assembly.assert_no_interference(
            _PAD_CLAMP_NAMES[_i], _PAD_CLAMP_NAMES[_j]
        )

# Load-cell standoffs vs everything else (carried-forward rev-1
# layout — kept as regression guards until rev-3 platform replaces
# them).
for _idx, _standoff_name in enumerate(_STANDOFF_NAMES):
    for _plat_part in _PLATFORM_NAMES:
        assembly = assembly.assert_no_interference(_standoff_name, _plat_part)

    for _mirror_part in _MIRROR_PART_NAMES:
        assembly = assembly.assert_no_interference(_standoff_name, _mirror_part)

    for _cam_part in _CAMERA_BRACKET_NAMES:
        assembly = assembly.assert_no_interference(_standoff_name, _cam_part)

    for _clamp_name in _PAD_CLAMP_NAMES:
        assembly = assembly.assert_no_interference(_standoff_name, _clamp_name)

# Standoff vs standoff — three columns at r = 220 with 120° angular
# separation (centre-to-centre 2·220·sin(60°) ≈ 381 mm). Regression.
for _i in range(len(_STANDOFF_NAMES)):
    for _j in range(_i + 1, len(_STANDOFF_NAMES)):
        assembly = assembly.assert_no_interference(
            _STANDOFF_NAMES[_i], _STANDOFF_NAMES[_j]
        )


if __name__ == "__main__":
    check(assembly, out="outputs")
