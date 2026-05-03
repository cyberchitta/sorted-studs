"""
M03 camera rig — unit-internal composition + within-unit checks.

The camera rig groups the sparse 4-rail T-frame, the 3 mirror mounts +
their pad-clamps, the camera bracket + its hanging pad-clamp, and the
two printed gussets at the rail A/B and rail C/D joints. The rig's
local frame matches world (Z-up; the top-level scanner places the rig
at world identity), so all coordinates here are world-equivalent.

Standalone composition: every rig sub-assembly placed in rig-local
coordinates, plus all the within-unit cross-boundary
`assert_no_interference` pairs. The cross-unit checks
(turret-vs-rig) live in `../assembly.py`.

Sub-assemblies:
  * skeleton_t_frame              — sparse 4-rail T-frame + braces
  * mirror_mount (×3)             — rev-1 rigid mounts
  * mirror_pad_clamp (×3)         — short-stub mirror clamps
  * camera_bracket                — rev-1 bracket + PCB + lens stubs
  * camera_pad_clamp              — rev-3 hanging pad-clamp on rail D
  * frame_gusset_t + frame_gusset_l — printed gussets at the rail
                                    A/B and rail C/D joints

Mirror-to-mirror interference is not asserted (closest pair ≈ 460 mm
apart by tetrahedral optics).
"""

import sys
from pathlib import Path

from build123d import Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from camera_rig.params import (  # noqa: E402
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
from camera_rig.camera_bracket import assembly as _cam  # noqa: E402
from camera_rig.camera_pad_clamp import assembly as _cam_pad_clamp  # noqa: E402
from camera_rig.frame_gusset_l import assembly as _gusset_l  # noqa: E402
from camera_rig.frame_gusset_t import assembly as _gusset_t  # noqa: E402
from camera_rig.mirror_mount import assembly as _mirror  # noqa: E402
from camera_rig.mirror_pad_clamp import assembly as _pad_clamp  # noqa: E402
from camera_rig.skeleton_t_frame import assembly as _skel  # noqa: E402

from build123d import Rot  # noqa: E402


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


# --- Public composition builder ----------------------------------------

# Names of every rig part this builder adds, exposed so the top-level
# can iterate them for cross-unit assertions.
SKELETON_NAMES = _skel.SKELETON_T_FRAME_PART_NAMES
GUSSET_NAMES = ("gusset_t_ab", "gusset_l_cd")
CAMERA_BRACKET_NAMES = ("camera_plate", "camera_pcb", "camera_lens")
MIRROR_PART_NAMES = tuple(
    name
    for prefix in _MIRROR_POSES
    for name in (f"{prefix}_bracket", f"{prefix}_glass")
)
PAD_CLAMP_NAMES = tuple(_PAD_CLAMP_SPECS.keys())
CAMERA_CLAMP_NAME = "pad_clamp_camera"

CAMERA_RIG_PART_NAMES = (
    list(SKELETON_NAMES)
    + list(GUSSET_NAMES)
    + list(CAMERA_BRACKET_NAMES)
    + list(MIRROR_PART_NAMES)
    + list(PAD_CLAMP_NAMES)
    + [CAMERA_CLAMP_NAME]
)


def add_camera_rig(a: Assembly) -> Assembly:
    """Add every camera-rig part in rig-local coordinates."""
    # Skeleton.
    a = _skel.add_skeleton_t_frame(a)
    # Gussets.
    a = _gusset_t.add_frame_gusset_t(a, name=GUSSET_NAMES[0])
    a = _gusset_l.add_frame_gusset_l(a, name=GUSSET_NAMES[1])
    # Camera bracket + PCB + lens stand-in.
    a = (
        a
        .add("camera_plate", _cam.bracket(), location=_CAMERA_LOC, material="pla_matte")
        # camera_pcb / camera_lens material left unset — see render.py overrides.
        .add("camera_pcb", _cam.camera_pcb_stub(), location=_CAMERA_LOC)
        .add("camera_lens", _cam.lens_stub(), location=_CAMERA_LOC)
    )
    # Three mirror mounts. Glass material left unset — see render.py overrides.
    for prefix, loc in _MIRROR_POSES.items():
        a = (
            a
            .add(f"{prefix}_bracket", _mirror.bracket(), location=loc, material="pla_matte")
            .add(f"{prefix}_glass", _mirror.mirror_stub(), location=loc)
        )
    # Three mirror pad clamps.
    for name, spec in _PAD_CLAMP_SPECS.items():
        loc = Pos(*spec["clamp_pos"]) * spec["clamp_rot"]
        mount_in_clamp = _pad_clamp.mount_in_clamp_for(**spec)
        a = a.add(
            name,
            _pad_clamp.clamp(mount_in_clamp=mount_in_clamp),
            location=loc,
            material="pla_matte",
        )
    # Camera pad clamp.
    camera_clamp_loc = (
        Pos(*RAIL_PAD_CLAMP_CAMERA_POS) * RAIL_PAD_CLAMP_CAMERA_CLAMP_ROT
    )
    camera_mount_in_clamp = _cam_pad_clamp.mount_in_clamp_for(
        clamp_pos=RAIL_PAD_CLAMP_CAMERA_POS,
        clamp_rot=RAIL_PAD_CLAMP_CAMERA_CLAMP_ROT,
        camera_pos=CAMERA_POS,
        camera_rot_deg=CAMERA_ROT_DEG,
    )
    a = a.add(
        CAMERA_CLAMP_NAME,
        _cam_pad_clamp.clamp(mount_in_clamp=camera_mount_in_clamp),
        location=camera_clamp_loc,
        material="pla_matte",
    )
    return a


def add_camera_rig_internal_checks(a: Assembly) -> Assembly:
    """Add every within-unit `assert_no_interference` pair. Exposed so
    the top-level can run them after placing the rig in world (since
    the rig is at world identity, the same checks hold)."""
    # Every non-skeleton camera-rig part vs every skeleton member.
    non_skeleton = [n for n in CAMERA_RIG_PART_NAMES if n not in SKELETON_NAMES]
    for internal in non_skeleton:
        for member in SKELETON_NAMES:
            a = a.assert_no_interference(internal, member)

    # Mirror mounts vs camera bracket triplet — regression.
    for mp in MIRROR_PART_NAMES:
        for cp in CAMERA_BRACKET_NAMES:
            a = a.assert_no_interference(mp, cp)

    # Mirror pad-clamps vs every mirror mount + camera bracket + each
    # other (regression / tangent).
    for clamp_name in PAD_CLAMP_NAMES:
        for mp in MIRROR_PART_NAMES:
            a = a.assert_no_interference(clamp_name, mp)
        for cp in CAMERA_BRACKET_NAMES:
            a = a.assert_no_interference(clamp_name, cp)
    for i in range(len(PAD_CLAMP_NAMES)):
        for j in range(i + 1, len(PAD_CLAMP_NAMES)):
            a = a.assert_no_interference(PAD_CLAMP_NAMES[i], PAD_CLAMP_NAMES[j])

    # Camera pad clamp vs everything else camera-rig.
    for other in MIRROR_PART_NAMES + PAD_CLAMP_NAMES + CAMERA_BRACKET_NAMES:
        a = a.assert_no_interference(CAMERA_CLAMP_NAME, other)

    # Frame gussets vs all non-skeleton internals (skeleton checked above).
    non_gusset = [n for n in non_skeleton if n not in GUSSET_NAMES]
    for gusset in GUSSET_NAMES:
        for other in non_gusset:
            a = a.assert_no_interference(gusset, other)
    a = a.assert_no_interference(GUSSET_NAMES[0], GUSSET_NAMES[1])

    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_camera_rig_internal_checks(add_camera_rig(Assembly()))


if __name__ == "__main__":
    check(assembly, out="outputs")
