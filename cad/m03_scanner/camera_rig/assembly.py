"""
M03 camera rig — unit-internal composition + within-unit checks.

The camera rig groups the sparse 4-rail T-frame, the 3 merged mirror
clamp-mounts, the merged camera clamp-bracket, and the two printed
gussets at the rail A/B and rail C/D joints. The rig's local frame
matches world (Z-up; the top-level scanner places the rig at world
identity), so all coordinates here are world-equivalent.

Standalone composition: every rig sub-assembly placed in rig-local
coordinates, plus all the within-unit cross-boundary
`assert_no_interference` pairs. The cross-unit checks
(turret-vs-rig) live in `../assembly.py`.

Sub-assemblies:
  * skeleton_t_frame              — sparse 4-rail T-frame + braces
  * mirror_clamp_mount (×3)       — merged clamp + plate + mirror bond
  * camera_clamp_bracket          — merged clamp + plate + PCB/lens stubs
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
from camera_rig.camera_clamp_bracket import assembly as _cam_cb  # noqa: E402
from camera_rig.frame_gusset_l import assembly as _gusset_l  # noqa: E402
from camera_rig.frame_gusset_t import assembly as _gusset_t  # noqa: E402
from camera_rig.mirror_clamp_mount import assembly as _mirror_cm  # noqa: E402
from camera_rig.skeleton_t_frame import assembly as _skel  # noqa: E402

from build123d import Rot  # noqa: E402

from design_systems import color_of  # noqa: E402
from materials import material_of  # noqa: E402


# --- Camera-rig placement constants ------------------------------------

_CAMERA_LOC = Pos(*CAMERA_POS) * Rot(*CAMERA_ROT_DEG)

# Per-instance specs for the three merged mirror clamp-mounts. Each
# instance contributes one printed body (`{prefix}_clamp_mount`) placed
# at the clamp pose and one mirror stub (`{prefix}_glass`) placed at
# the mirror pose. The printed body's plate orientation is parameterised
# by `mount_in_clamp` so all three derive from one `clamp_mount(...)`
# function.
_MIRROR_INSTANCES = {
    "mirror_below_left": {
        "clamp_pos": RAIL_PAD_CLAMP_BELOW_LEFT_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
        "mirror_pos": MIRROR_BELOW_LEFT_POS,
        "mirror_normal": MIRROR_BELOW_LEFT_NORMAL,
    },
    "mirror_below_right": {
        "clamp_pos": RAIL_PAD_CLAMP_BELOW_RIGHT_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BELOW_RIGHT_CLAMP_ROT,
        "mirror_pos": MIRROR_BELOW_RIGHT_POS,
        "mirror_normal": MIRROR_BELOW_RIGHT_NORMAL,
    },
    "mirror_back_upper": {
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
CAMERA_BRACKET_NAMES = ("camera_clamp_bracket", "camera_pcb", "camera_lens")
MIRROR_PART_NAMES = tuple(
    name
    for prefix in _MIRROR_INSTANCES
    for name in (f"{prefix}_clamp_mount", f"{prefix}_glass")
)

CAMERA_RIG_PART_NAMES = (
    list(SKELETON_NAMES)
    + list(GUSSET_NAMES)
    + list(CAMERA_BRACKET_NAMES)
    + list(MIRROR_PART_NAMES)
)


def add_camera_rig(a: Assembly) -> Assembly:
    """Add every camera-rig part in rig-local coordinates."""
    # Skeleton.
    a = _skel.add_skeleton_t_frame(a)
    # Gussets.
    a = _gusset_t.add_frame_gusset_t(a, name=GUSSET_NAMES[0])
    a = _gusset_l.add_frame_gusset_l(a, name=GUSSET_NAMES[1])
    # Merged camera clamp-bracket. Single printed body at the clamp pose
    # with the bracket plate transformed by `mount_in_clamp`; the PCB
    # and lens stubs sit at the camera pose for diagnostics + render
    # material override.
    camera_clamp_loc = (
        Pos(*RAIL_PAD_CLAMP_CAMERA_POS) * RAIL_PAD_CLAMP_CAMERA_CLAMP_ROT
    )
    camera_mount_in_clamp = _cam_cb.mount_in_clamp_for(
        clamp_pos=RAIL_PAD_CLAMP_CAMERA_POS,
        clamp_rot=RAIL_PAD_CLAMP_CAMERA_CLAMP_ROT,
        camera_pos=CAMERA_POS,
        camera_rot_deg=CAMERA_ROT_DEG,
    )
    a = (
        a
        .add(
            "camera_clamp_bracket",
            _cam_cb.clamp_bracket(mount_in_clamp=camera_mount_in_clamp),
            location=camera_clamp_loc,
            material=material_of("camera_clamp_bracket"),
            color=color_of("camera_clamp_bracket"),
        )
        # camera_pcb / camera_lens material left unset — see render.py overrides.
        .add("camera_pcb", _cam_cb.camera_pcb_stub(), location=_CAMERA_LOC)
        .add("camera_lens", _cam_cb.lens_stub(), location=_CAMERA_LOC)
    )
    # Three merged mirror clamp-mounts. Each is one printed body at the
    # clamp pose; the mirror stub sits at the mirror pose for diagnostics
    # + render material override.
    for prefix, spec in _MIRROR_INSTANCES.items():
        clamp_loc = Pos(*spec["clamp_pos"]) * spec["clamp_rot"]
        mirror_loc = Pos(*spec["mirror_pos"]) * rotation_for_normal(
            spec["mirror_normal"],
            roll_deg=horizontal_x_roll_deg(spec["mirror_normal"]),
        )
        mount_in_clamp = _mirror_cm.mount_in_clamp_for(**spec)
        a = (
            a
            .add(
                f"{prefix}_clamp_mount",
                _mirror_cm.clamp_mount(mount_in_clamp=mount_in_clamp),
                location=clamp_loc,
                material=material_of(f"{prefix}_clamp_mount"),
                color=color_of(f"{prefix}_clamp_mount"),
            )
            .add(
                f"{prefix}_glass", _mirror_cm.mirror_stub(),
                location=mirror_loc,
                material=material_of(f"{prefix}_glass"),
            )
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

    # Mirror clamp-mounts + glass vs merged camera clamp-bracket triplet
    # — regression guard for the rig-internal mirror/camera spacing.
    for mp in MIRROR_PART_NAMES:
        for cp in CAMERA_BRACKET_NAMES:
            a = a.assert_no_interference(mp, cp)

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
