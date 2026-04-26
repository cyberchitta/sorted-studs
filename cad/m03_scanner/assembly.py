"""
M03 scanner — rev-3 top-level assembly (in progress).

Composes the rev-3 sub-assemblies that exist so far in world
coordinates and runs cross-boundary interference checks. Per-sub-
system mechanism + per-part printability live in each sub-assembly's
standalone `assembly.py`; this file only covers pairs that span two
sub-systems.

Camera rig (unchanged from prior rev):
  * skeleton_t_frame              — sparse 4-rail T-frame + braces
  * mirror_pad_clamp (×3)         — short-stub mirror clamps
  * mirror_mount (×3)             — rev-1 rigid mounts
  * camera_bracket                — rev-1 bracket
  * camera_pad_clamp              — rev-3 hanging pad-clamp on rail D
  * frame_gusset_t + frame_gusset_l — printed gussets at rail
                                    A/B and rail C/D joints

Rev-4 turret stack (replaces the rev-3 single-tilt platform):
  * turret_frame                  — 5-member 2020 floor frame on the
                                    shared bench (y = -220), back
                                    rail butting rail A's front face
  * turret_hub                    — printed pillow-block + Ø8 shaft +
                                    two 608ZZ bearings, bolted to
                                    the cross-rail's top T-slot
  * turret_arm                    — single printed 4-arm spider
                                    clamped to the shaft top + 4
                                    stock M5 hinge pins
  * turret_platform (×4 stations) — 4 cartridges (PMMA plate +
                                    bonded hinge brackets +
                                    perimeter ring), one per station
                                    (IMAGE / LOAD / DUMP / RETURN)
  * turret_cam_ring               — fixed annular cam ring at radius
                                    220 (level all the way around;
                                    DUMP notch + banking deferred)

The rev-3 single-platform stack (`platform_frame/`,
`platform_hinge_bearing/`, `platform_acrylic/`) stays in tree as
historical reference — the directories are still importable
standalones, but their placements are no longer in this top-level.

Cross-boundary checks:
  * Camera-rig sub-assemblies: same nested loops as before
    (skeleton-vs-everything, mirror-vs-{camera, mirror clamps,
    other mirror clamps}, camera clamp vs camera bracket, etc.).
  * Turret-vs-camera-rig: every turret part vs every camera-rig
    part — comprehensive regression guard.
  * Turret-vs-turret: every turret part vs every other turret part
    (skipping self-pairs). Catches things like spider/shaft tangent
    fit, bracket/pin tangent fit at each station, cam ring vs swept
    platform footprint, etc.

Mirror-to-mirror interference is not asserted (closest pair ≈ 460 mm
apart by tetrahedral optics).
"""

import sys
from pathlib import Path

from build123d import Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

sys.path.insert(0, str(Path(__file__).resolve().parent))

from camera_bracket import assembly as _cam  # noqa: E402
from camera_pad_clamp import assembly as _cam_pad_clamp  # noqa: E402
from frame_gusset_l import assembly as _gusset_l  # noqa: E402
from frame_gusset_t import assembly as _gusset_t  # noqa: E402
from mirror_mount import assembly as _mirror  # noqa: E402
from mirror_pad_clamp import assembly as _pad_clamp  # noqa: E402
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
from skeleton_t_frame import assembly as _skel  # noqa: E402
from turret_arm import assembly as _turret_arm  # noqa: E402
from turret_cam_ring import assembly as _turret_cam_ring  # noqa: E402
from turret_frame import assembly as _turret_frame  # noqa: E402
from turret_hub import assembly as _turret_hub  # noqa: E402
from turret_platform import assembly as _turret_platform  # noqa: E402


# --- Camera-rig placement constants ------------------------------------

_CAMERA_LOC = Pos(*CAMERA_POS) * Rot(*CAMERA_ROT_DEG)

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


# --- Camera-rig assembly composition ----------------------------------

assembly = Assembly()

# Camera-rig skeleton.
_SKELETON_NAMES = _skel.SKELETON_T_FRAME_PART_NAMES
assembly = _skel.add_skeleton_t_frame(assembly)

# Camera-rig gussets.
_GUSSET_NAMES = ("gusset_t_ab", "gusset_l_cd")
assembly = _gusset_t.add_frame_gusset_t(assembly, name=_GUSSET_NAMES[0])
assembly = _gusset_l.add_frame_gusset_l(assembly, name=_GUSSET_NAMES[1])

# Camera bracket + PCB + lens stand-in.
assembly = (
    assembly
    .add("camera_plate", _cam.bracket(), location=_CAMERA_LOC)
    .add("camera_pcb", _cam.camera_pcb_stub(), location=_CAMERA_LOC)
    .add("camera_lens", _cam.lens_stub(), location=_CAMERA_LOC)
)

# Three mirror mounts.
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

# Three mirror pad clamps.
_PAD_CLAMP_NAMES = tuple(_PAD_CLAMP_SPECS.keys())
for _name, _spec in _PAD_CLAMP_SPECS.items():
    _loc = Pos(*_spec["clamp_pos"]) * _spec["clamp_rot"]
    _mount_in_clamp = _pad_clamp.mount_in_clamp_for(**_spec)
    assembly = assembly.add(
        _name,
        _pad_clamp.clamp(mount_in_clamp=_mount_in_clamp),
        location=_loc,
    )

# Camera pad clamp.
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


# --- Rev-4 turret assembly composition ---------------------------------

# Turret floor frame (5 × 2020 members on the shared bench).
_TURRET_FRAME_NAMES = _turret_frame.TURRET_FRAME_PART_NAMES
assembly = _turret_frame.add_turret_frame(assembly)

# Turret hub (housing + shaft + 2 bearings).
_TURRET_HUB_NAMES = _turret_hub.HUB_PART_NAMES
assembly = _turret_hub.add_turret_hub(assembly)

# Turret arm spider (hub clamp + 4 radial arms + 4 tip bushing
# blocks) and 4 stock pins captured in the bushing block bores.
_TURRET_ARM_NAMES = _turret_arm.ARM_PART_NAMES
assembly = _turret_arm.add_turret_arm(assembly)

# Four turret platform cartridges, one per station.
_TURRET_STATIONS = ("image", "load", "dump", "return")
_TURRET_PLATFORM_NAMES = []
for _station in _TURRET_STATIONS:
    assembly = _turret_platform.add_turret_platform(assembly, _station)
    _TURRET_PLATFORM_NAMES.extend(
        f"platform_{_station}_{sub_name}"
        for sub_name, _ in _turret_platform._IMAGE_LOCS
    )

# Turret cam ring (single instance, level all the way around).
_TURRET_CAM_RING_NAMES = (_turret_cam_ring.CAM_RING_PART_NAME,)
assembly = _turret_cam_ring.add_turret_cam_ring(assembly)


# --- Cross-boundary assertions: camera-rig internal -------------------

_CAMERA_BRACKET_NAMES = ("camera_plate", "camera_pcb", "camera_lens")
_CAMERA_RIG_NAMES = (
    list(_SKELETON_NAMES)
    + list(_GUSSET_NAMES)
    + list(_CAMERA_BRACKET_NAMES)
    + _MIRROR_PART_NAMES
    + list(_PAD_CLAMP_NAMES)
    + [_CAMERA_CLAMP_NAME]
)

# Every non-skeleton camera-rig part vs every skeleton member.
_CAMERA_RIG_NON_SKELETON = [
    n for n in _CAMERA_RIG_NAMES if n not in _SKELETON_NAMES
]
for _internal in _CAMERA_RIG_NON_SKELETON:
    for _member in _SKELETON_NAMES:
        assembly = assembly.assert_no_interference(_internal, _member)

# Mirror mounts vs camera bracket triplet — regression.
for _mirror_part in _MIRROR_PART_NAMES:
    for _cam_part in _CAMERA_BRACKET_NAMES:
        assembly = assembly.assert_no_interference(_mirror_part, _cam_part)

# Mirror pad-clamps vs every mirror mount + camera bracket + each
# other (regression / tangent).
for _clamp_name in _PAD_CLAMP_NAMES:
    for _mirror_part in _MIRROR_PART_NAMES:
        assembly = assembly.assert_no_interference(_clamp_name, _mirror_part)
    for _cam_part in _CAMERA_BRACKET_NAMES:
        assembly = assembly.assert_no_interference(_clamp_name, _cam_part)
for _i in range(len(_PAD_CLAMP_NAMES)):
    for _j in range(_i + 1, len(_PAD_CLAMP_NAMES)):
        assembly = assembly.assert_no_interference(
            _PAD_CLAMP_NAMES[_i], _PAD_CLAMP_NAMES[_j]
        )

# Camera pad clamp vs everything camera-rig.
for _other in _MIRROR_PART_NAMES + list(_PAD_CLAMP_NAMES) + list(_CAMERA_BRACKET_NAMES):
    assembly = assembly.assert_no_interference(_CAMERA_CLAMP_NAME, _other)

# Frame gussets vs all non-skeleton internals (skeleton checked above).
_NON_GUSSET_CAMERA_RIG = [
    n for n in _CAMERA_RIG_NON_SKELETON if n not in _GUSSET_NAMES
]
for _gusset in _GUSSET_NAMES:
    for _other in _NON_GUSSET_CAMERA_RIG:
        assembly = assembly.assert_no_interference(_gusset, _other)
assembly = assembly.assert_no_interference(_GUSSET_NAMES[0], _GUSSET_NAMES[1])


# --- Cross-boundary assertions: turret vs camera-rig ------------------

_TURRET_NAMES = (
    list(_TURRET_FRAME_NAMES)
    + list(_TURRET_HUB_NAMES)
    + list(_TURRET_ARM_NAMES)
    + _TURRET_PLATFORM_NAMES
    + list(_TURRET_CAM_RING_NAMES)
)

# Every turret part vs every camera-rig part. Comprehensive
# regression guard: catches any drift in either subsystem that
# would push a turret part into a camera-rig part.
for _turret_part in _TURRET_NAMES:
    for _rig_part in _CAMERA_RIG_NAMES:
        assembly = assembly.assert_no_interference(_turret_part, _rig_part)


# --- Cross-boundary assertions: turret internal -----------------------

# Adjacent platforms overlap geometrically because TURRET_ARM_R = 130
# is too small for the 180 × 180 mm platform footprint. Two squares
# 90° apart at radius R have non-overlapping inner corners only when
# R ≥ 180 mm; the brief's "r = 100–150 mm" range was wrong for the
# Ø172-driven platform size. Each adjacent-station pair overlaps in
# its acrylic (~10 000 mm³) and its perimeter ring (~160 mm³). The
# 4 adjacent station pairs (image↔load, image↔dump, load↔return,
# dump↔return) account for 8 expected interferences. Diagonal pairs
# (image↔return, load↔dump, both 180° apart with 260 mm centroid
# spacing) clear by 80 mm and don't overlap.
#
# These overlaps are documented via `assert_interference(reason=)`
# rather than skipped — the assertion FAILS if the overlap disappears
# (e.g., the user fixes TURRET_ARM_R to ≥ 180 in a follow-up session),
# which forces the assertion to be removed at the same time as the
# design fix. NEXT_STEPS.md tracks this as the next-session refactor.
_ADJACENT_STATION_PAIRS = (
    ("image", "load"),
    ("image", "dump"),
    ("load", "return"),
    ("dump", "return"),
)
_OVERLAPPING_PLATFORM_PARTS = ("acrylic", "ring")
_EXPECTED_PLATFORM_OVERLAPS = {
    frozenset((f"platform_{s1}_{sub}", f"platform_{s2}_{sub}"))
    for s1, s2 in _ADJACENT_STATION_PAIRS
    for sub in _OVERLAPPING_PLATFORM_PARTS
}
_PLATFORM_OVERLAP_REASON = (
    "TURRET_ARM_R = 130 is too small for the 180 × 180 platform; "
    "adjacent platforms overlap at their inner corners. Fix in a "
    "follow-up: raise TURRET_ARM_R to ≥ 180 (and update TURRET_HUB_Z, "
    "TURRET_FRAME_*, TURRET_HINGE_PIN_RADIAL, TURRET_CAM_RING_R + "
    "their derived constants). This assertion fires when the overlap "
    "is fixed — remove it then."
)

# Every turret part vs every other turret part. Catches:
#   * spider hub bore vs shaft (tangent contact, must not interfere)
#   * spider bushing blocks vs platform brackets (AIR_GAP separation)
#   * pin in bracket bore (tangent, captures the platform)
#   * cam ring vs swept platform footprint (Y-separated)
#   * frame members vs hub housing (housing bolts to cross-rail)
#   * etc.
# The within-sub-assembly pairs are also re-checked here (cheap;
# they re-verify after world placement). The 8 expected
# platform-vs-platform overlaps use assert_interference instead.
for _i in range(len(_TURRET_NAMES)):
    for _j in range(_i + 1, len(_TURRET_NAMES)):
        _a, _b = _TURRET_NAMES[_i], _TURRET_NAMES[_j]
        if frozenset((_a, _b)) in _EXPECTED_PLATFORM_OVERLAPS:
            assembly = assembly.assert_interference(
                _a, _b, reason=_PLATFORM_OVERLAP_REASON
            )
        else:
            assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
