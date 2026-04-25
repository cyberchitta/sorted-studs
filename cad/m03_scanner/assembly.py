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
                                    unchanged. Now supported by the
                                    rev-3 camera_pad_clamp (slice c).
  * camera_pad_clamp              — rev-3 hanging pad-clamp on rail D's
                                    underside, carrying the camera
                                    bracket. Replaces the rev-2
                                    camera_bracket_rail_clamp.
  * platform_frame (×5 members)   — rev-3 standalone 2020 frame
                                    carrying the hinged platform.
                                    Sits in front of the camera rig
                                    and is mechanically decoupled
                                    from it; replaces rev-1
                                    `../platform/` + `load_cell_standoff/`.
  * platform_hinge_bearing (×2)   — rev-3 printed pillow-blocks
                                    bolted to the platform frame's
                                    top cross-rail.
  * platform_acrylic + brackets (×3 parts)
                                  — rev-3 PMMA plate (`acrylic_plate`)
                                    bonded to two printed hinge
                                    brackets that share the world
                                    hinge axis with the bearings.

Slices NOT yet placed (see `NEXT_STEPS.md` Suggested next CAD moves):
  * frame_gusset_t/, _l/          — camera-rig slice (d)

The rev-1/rev-2 sub-assembly directories were deleted from the tree
in the slice (3) cleanup commit (see `superseded_platform_designs.md`
for the retrieval commit hashes if you need to inspect the old
designs).

Cross-boundary checks:
  * Every internal part vs every skeleton_t_frame member (regression
    guard against any sub-assembly drift pushing a part into a rail).
  * Mirror pad clamps vs the supported mirror mounts — tangent
    contact at the pad face (`assert_no_interference` catches drift
    past MOUNT_GAP), and regression-only against the other two.
  * Camera pad clamp vs the camera bracket — same MOUNT_GAP tangent
    contact at the pad face.
  * Hinge bearings sit on the platform-frame cross-rail's top face
    (tangent contact); the brackets share the world hinge axis with
    the bearings but sit 5 mm inboard along ±X. Both are
    `assert_no_interference` — they catch drift past the modelled
    gap (tangent contact + the X air-gap respectively) without
    pretending the parts don't touch.
  * Acrylic plate sits 1 mm above the bearing top faces at the
    bearings' inboard X; the assertion guards against the gap
    closing.
  * Mirror mounts vs camera bracket and vs platform parts — same
    regression guards as the rev-2 top-level.

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
    RAIL_PAD_CLAMP_CAMERA_CLAMP_ROT,
    RAIL_PAD_CLAMP_CAMERA_POS,
    horizontal_x_roll_deg,
    rotation_for_normal,
)


# --- Sub-assembly loading -----------------------------------------------
#
# Same importlib trick as the rev-2 top-level: each sub-directory
# holds a bare `assembly.py` with no `__init__.py`. Load each by
# file path.

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
_plat_frame = _load("platform_frame")
_bearing = _load("platform_hinge_bearing")
_plat_acrylic = _load("platform_acrylic")
_cam = _load("camera_bracket")
_mirror = _load("mirror_mount")
_pad_clamp = _load("mirror_pad_clamp")
_cam_pad_clamp = _load("camera_pad_clamp")


# --- Placement constants ------------------------------------------------

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


# --- Assembly composition -----------------------------------------------

assembly = Assembly()

# Rev-3 sparse T-frame skeleton. Each member in `_skel._MEMBERS` is
# already positioned at its world location.
_SKELETON_NAMES = tuple(_skel._MEMBERS.keys())
for _name, _part in _skel._MEMBERS.items():
    assembly = assembly.add(_name, _part)

# Rev-3 platform frame (5 members: front X-rail + Z-tongue floor,
# 2 front posts, top X cross-rail). Same convention as skeleton: the
# sub-assembly bakes world positions into each part.
_PLATFORM_FRAME_NAMES = tuple(_plat_frame._MEMBERS.keys())
for _name, _part in _plat_frame._MEMBERS.items():
    assembly = assembly.add(_name, _part)

# Rev-3 hinge bearings (×2). Bearing-local origin is the bottom face
# of the block; world placement puts that face on the cross-rail's
# top face (tangent contact), centered along the rail in Z.
_BEARING_NAMES = ("bearing_left", "bearing_right")
for _name, _x_sign in zip(_BEARING_NAMES, (-1, +1)):
    assembly = assembly.add(
        _name,
        _bearing.bearing(),
        location=Pos(
            _x_sign * _bearing.PLATFORM_HINGE_BEARING_X,
            _bearing._RAIL_TOP_Y_WORLD,
            _bearing.PLATFORM_HINGE_PIN_Z,
        ),
    )

# Rev-3 platform: PMMA plate centered on world origin + two printed
# hinge brackets bonded to its underside.
_ACRYLIC_NAME = "acrylic_plate"
assembly = assembly.add(_ACRYLIC_NAME, _plat_acrylic.acrylic_plate())

_BRACKET_NAMES = ("bracket_left", "bracket_right")
for _name, _x_sign in zip(_BRACKET_NAMES, (-1, +1)):
    assembly = assembly.add(
        _name,
        _plat_acrylic.bracket(),
        location=Pos(
            _x_sign * _plat_acrylic.PLATFORM_HINGE_BRACKET_X,
            _plat_acrylic._BRACKET_Y_BOTTOM_WORLD,
            _plat_acrylic.BRACKET_Z_CENTRE_WORLD,
        ),
    )

# Camera bracket + PCB + lens stand-in (rev-1, supported by camera_pad_clamp).
assembly = (
    assembly
    .add("camera_plate", _cam.bracket(), location=_CAMERA_LOC)
    .add("camera_pcb", _cam.camera_pcb_stub(), location=_CAMERA_LOC)
    .add("camera_lens", _cam.lens_stub(), location=_CAMERA_LOC)
)

# Three rigid mirror mounts — bracket + mirror-glass stand-in per pose.
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

# Rev-3 camera pad clamp — single instance, hangs below rail D
# carrying the camera bracket.
_CAMERA_CLAMP_NAME = "pad_clamp_camera"
_camera_clamp_loc = (
    Pos(*RAIL_PAD_CLAMP_CAMERA_POS) * RAIL_PAD_CLAMP_CAMERA_CLAMP_ROT
)
_camera_mount_in_clamp = _cam_pad_clamp.mount_in_clamp_for(
    clamp_pos=RAIL_PAD_CLAMP_CAMERA_POS,
    clamp_rot=RAIL_PAD_CLAMP_CAMERA_CLAMP_ROT,
    camera_pos=CAMERA_POS,
    camera_rot_deg=CAMERA_ROT_DEG,
)
assembly = assembly.add(
    _CAMERA_CLAMP_NAME,
    _cam_pad_clamp.clamp(mount_in_clamp=_camera_mount_in_clamp),
    location=_camera_clamp_loc,
)


# --- Cross-boundary assertions ------------------------------------------

# Group names by sub-system for the pairwise nests below.
_PLATFORM_NAMES = (
    *_PLATFORM_FRAME_NAMES,
    *_BEARING_NAMES,
    _ACRYLIC_NAME,
    *_BRACKET_NAMES,
)
_CAMERA_BRACKET_NAMES = ("camera_plate", "camera_pcb", "camera_lens")
_INTERNAL_NAMES = (
    list(_PLATFORM_NAMES)
    + list(_CAMERA_BRACKET_NAMES)
    + _MIRROR_PART_NAMES
    + list(_PAD_CLAMP_NAMES)
    + [_CAMERA_CLAMP_NAME]
)

# Every internal part vs every skeleton_t_frame member.
for _internal in _INTERNAL_NAMES:
    for _member in _SKELETON_NAMES:
        assembly = assembly.assert_no_interference(_internal, _member)

# Mirror mounts vs camera bracket triplet — regression guard.
for _mirror_part in _MIRROR_PART_NAMES:
    for _cam_part in _CAMERA_BRACKET_NAMES:
        assembly = assembly.assert_no_interference(_mirror_part, _cam_part)

# Mirror mounts vs platform sub-system — regression guard.
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

    # vs platform sub-system — below-mirror clamps at y ∈ [-220, ~-98],
    # back-upper clamp on rail C's -Z face; platform frame at
    # y ∈ [-245, -30], acrylic at y ∈ [-4, 0]. Regression guards.
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

# Camera pad-clamp vs the camera bracket triplet — tangent contact at
# the pad face (`assert_no_interference` catches drift past MOUNT_GAP).
for _cam_part in _CAMERA_BRACKET_NAMES:
    assembly = assembly.assert_no_interference(_CAMERA_CLAMP_NAME, _cam_part)

# Camera pad-clamp vs every mirror mount + every mirror pad-clamp —
# regression guard. The camera clamp hangs from rail D over the camera
# (z ≈ -231); the mirrors live at z ∈ [25, 173]. No overlap expected.
for _mirror_part in _MIRROR_PART_NAMES:
    assembly = assembly.assert_no_interference(_CAMERA_CLAMP_NAME, _mirror_part)
for _mirror_clamp_name in _PAD_CLAMP_NAMES:
    assembly = assembly.assert_no_interference(
        _CAMERA_CLAMP_NAME, _mirror_clamp_name
    )

# Camera pad-clamp vs platform sub-system — regression guard.
for _plat_part in _PLATFORM_NAMES:
    assembly = assembly.assert_no_interference(_CAMERA_CLAMP_NAME, _plat_part)

# Internal platform sub-system cross-boundary pairs. Each platform
# piece (frame member, bearing, acrylic, bracket) crossed against
# every other piece — most are regression, the supported tangent /
# air-gap pairs are called out in the docstring above.
for _i in range(len(_PLATFORM_NAMES)):
    for _j in range(_i + 1, len(_PLATFORM_NAMES)):
        assembly = assembly.assert_no_interference(
            _PLATFORM_NAMES[_i], _PLATFORM_NAMES[_j]
        )


if __name__ == "__main__":
    check(assembly, out="outputs")
