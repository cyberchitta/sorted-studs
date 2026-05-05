"""
M03 camera rig — unit-internal composition + within-unit checks.

The camera rig groups the sparse 4-rail T-frame, four merged ESP32-CAM
clamp+mount bodies (one per tetrahedral vertex), and the two printed
gussets at the rail A/B and rail C/D joints. The rig's local frame
matches world (Z-up; the top-level scanner places the rig at world
identity), so all coordinates here are world-equivalent.

Standalone composition: every rig sub-assembly placed in rig-local
coordinates, plus all the within-unit cross-boundary
`assert_no_interference` pairs. The cross-unit checks
(turret-vs-rig) live in `../assembly.py`.

Sub-assemblies:
  * skeleton_t_frame              — sparse 4-rail T-frame + braces
  * camera_mount_esp32cam (×4)    — merged clamp + plate + PCB/lens stubs
  * frame_gusset_t + frame_gusset_l — printed gussets at the rail
                                    A/B and rail C/D joints

Camera-to-camera interference is not asserted (closest pair ≈ 400 mm
apart by tetrahedral optics).
"""

import sys
from pathlib import Path

from build123d import Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from camera_rig.params import (  # noqa: E402
    RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
    RAIL_PAD_CLAMP_BACK_UPPER_POS,
    RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
    RAIL_PAD_CLAMP_BELOW_LEFT_POS,
    RAIL_PAD_CLAMP_BELOW_RIGHT_CLAMP_ROT,
    RAIL_PAD_CLAMP_BELOW_RIGHT_POS,
    RAIL_PAD_CLAMP_FRONT_LOW_CLAMP_ROT,
    RAIL_PAD_CLAMP_FRONT_LOW_POS,
    VERTEX_BACK_UPPER_POS,
    VERTEX_BELOW_LEFT_POS,
    VERTEX_BELOW_RIGHT_POS,
    VERTEX_FRONT_LOW_POS,
    horizontal_x_roll_deg,
    lens_axis_unit,
    rotation_for_normal,
)
from camera_rig.camera_mount_esp32cam import assembly as _camera_mount  # noqa: E402
from camera_rig.frame_gusset_l import assembly as _gusset_l  # noqa: E402
from camera_rig.frame_gusset_t import assembly as _gusset_t  # noqa: E402
from camera_rig.skeleton_t_frame import assembly as _skel  # noqa: E402

from design_systems import color_of  # noqa: E402
from materials import material_of  # noqa: E402


# --- Per-vertex specs --------------------------------------------------
#
# Each instance contributes one printed body (`{prefix}_clamp_mount`)
# placed at the clamp pose plus one PCB stub (`{prefix}_pcb`) and one
# M12 lens stub (`{prefix}_lens`) placed at the vertex pose. The
# printed body's plate orientation is parameterised by `mount_in_clamp`
# so all four instances derive from one `clamp_mount(...)` function.

_CAMERA_INSTANCES = {
    "cam_front_low": {
        "clamp_pos": RAIL_PAD_CLAMP_FRONT_LOW_POS,
        "clamp_rot": RAIL_PAD_CLAMP_FRONT_LOW_CLAMP_ROT,
        "vertex_pos": VERTEX_FRONT_LOW_POS,
    },
    "cam_back_upper": {
        "clamp_pos": RAIL_PAD_CLAMP_BACK_UPPER_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
        "vertex_pos": VERTEX_BACK_UPPER_POS,
    },
    "cam_below_left": {
        "clamp_pos": RAIL_PAD_CLAMP_BELOW_LEFT_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
        "vertex_pos": VERTEX_BELOW_LEFT_POS,
    },
    "cam_below_right": {
        "clamp_pos": RAIL_PAD_CLAMP_BELOW_RIGHT_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BELOW_RIGHT_CLAMP_ROT,
        "vertex_pos": VERTEX_BELOW_RIGHT_POS,
    },
}


# --- Public composition builder ----------------------------------------

# Names of every rig part this builder adds, exposed so the top-level
# can iterate them for cross-unit assertions.
SKELETON_NAMES = _skel.SKELETON_T_FRAME_PART_NAMES
GUSSET_NAMES = ("gusset_t_ab", "gusset_l_cd_pos_x", "gusset_l_cd_neg_x")
CAMERA_PART_NAMES = tuple(
    name
    for prefix in _CAMERA_INSTANCES
    for name in (f"{prefix}_clamp_mount", f"{prefix}_pcb", f"{prefix}_lens")
)

CAMERA_RIG_PART_NAMES = (
    list(SKELETON_NAMES)
    + list(GUSSET_NAMES)
    + list(CAMERA_PART_NAMES)
)


def add_camera_rig(a: Assembly) -> Assembly:
    """Add every camera-rig part in rig-local coordinates."""
    # Skeleton.
    a = _skel.add_skeleton_t_frame(a)
    # Gussets.
    a = _gusset_t.add_frame_gusset_t(a, name=GUSSET_NAMES[0])
    a = _gusset_l.add_frame_gusset_l(a, name=GUSSET_NAMES[1], side=+1)
    a = _gusset_l.add_frame_gusset_l(a, name=GUSSET_NAMES[2], side=-1)
    # Four merged camera clamp+mount bodies. Each is one printed body
    # at the clamp pose; the PCB and M12 lens stubs sit at the vertex
    # pose for diagnostics + render material override.
    for prefix, spec in _CAMERA_INSTANCES.items():
        clamp_loc = Pos(*spec["clamp_pos"]) * spec["clamp_rot"]
        axis = lens_axis_unit(spec["vertex_pos"])
        camera_loc = Pos(*spec["vertex_pos"]) * rotation_for_normal(
            axis, roll_deg=horizontal_x_roll_deg(axis)
        )
        mount_in_clamp = _camera_mount.mount_in_clamp_for(**spec)
        a = (
            a
            .add(
                f"{prefix}_clamp_mount",
                _camera_mount.clamp_mount(mount_in_clamp=mount_in_clamp),
                location=clamp_loc,
                material=material_of(f"{prefix}_clamp_mount"),
                color=color_of(f"{prefix}_clamp_mount"),
            )
            # PCB / lens material left unset — render-time overrides
            # in material_variations.py supply the FR4 / glass tags.
            .add(
                f"{prefix}_pcb", _camera_mount.camera_pcb_stub(),
                location=camera_loc,
            )
            .add(
                f"{prefix}_lens", _camera_mount.m12_lens_stub(),
                location=camera_loc,
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

    # Frame gussets vs all non-skeleton internals (skeleton checked above).
    non_gusset = [n for n in non_skeleton if n not in GUSSET_NAMES]
    for gusset in GUSSET_NAMES:
        for other in non_gusset:
            a = a.assert_no_interference(gusset, other)
    # Every gusset-vs-gusset pair (the C/D pair lives on opposite X
    # sides; the T at A/B lives well clear of them).
    for _i, _g1 in enumerate(GUSSET_NAMES):
        for _g2 in GUSSET_NAMES[_i + 1:]:
            a = a.assert_no_interference(_g1, _g2)

    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_camera_rig_internal_checks(add_camera_rig(Assembly()))


if __name__ == "__main__":
    check(assembly, out="outputs")
