"""M03 scanner — 4-camera POV render via chitra-cad.

Sister to ``render_camera_pov.py`` (single camera + 3 first-surface
mirrors). Replaces the mirror topology with 4 direct cameras at the
optimised tetrahedral viewpoints — the same world positions the
camera and 3 mirrors currently occupy. Each camera looks at the
world origin (the brick stand-in) and renders one PNG.

The mirror clamp-mounts + mirror glass and the camera clamp-bracket +
PCB + lens stubs are stripped from the rendered scene because the
4-camera arrangement physically replaces them — there's no point
rendering the mounting hardware that wouldn't exist in this variant.

Outputs land in ``outputs/renders_pov_4cam/`` as
``cam0_direct.png``, ``cam1_back_upper.png``,
``cam2_below_left.png``, ``cam3_below_right.png``.

Coordinate frame: world Z-up. Camera + mirror positions come from
``camera_rig/params.py``.
"""

import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from assembly import assembly  # noqa: E402
from camera_rig.params import (  # noqa: E402
    CAMERA_POS,
    MIRROR_BACK_UPPER_POS,
    MIRROR_BELOW_LEFT_POS,
    MIRROR_BELOW_RIGHT_POS,
)
from detail_variations import apply_default_detail  # noqa: E402
from material_variations import DEFAULT_OVERRIDES  # noqa: E402

from build123d import Box, Color  # noqa: E402
from chitra_cad.framing import FixedPose  # noqa: E402
from chitra_cad.render import render  # noqa: E402
from chitra_cad.scene import Scene  # noqa: E402


BRICK_2X4 = (31.8, 15.8, 9.6)

# Four cameras at the tetrahedral viewpoints. The camera + 3 mirrors
# in the existing rig sit at these world positions (all ≈ 250 mm
# from origin); replacing each mirror with a real camera at the same
# point preserves the optimised viewpoint geometry without any mirror
# bounce. The direct camera takes CAMERA_POS (PCB rear, the optical
# reference point on the existing bracket) rather than the lens-nose
# offset used in render_camera_pov.py — stripping the bracket means
# the camera no longer renders from inside opaque geometry, so the
# extra 56 mm forward-shift isn't needed.
CAMERA_POSES = [
    ("cam0_direct", CAMERA_POS),
    ("cam1_back_upper", MIRROR_BACK_UPPER_POS),
    ("cam2_below_left", MIRROR_BELOW_LEFT_POS),
    ("cam3_below_right", MIRROR_BELOW_RIGHT_POS),
]

# Single uniform VFOV. All 4 cameras sit ≈ 250 mm from origin, so
# the same lens fills each frame consistently:
#   HFOV = 2·atan((90 + 5) / 250) ≈ 41.6°
#   VFOV at 4:3 ≈ 2·atan(0.75 · tan(HFOV/2)) ≈ 32.0°
# This is a ~5–6 mm M12 lens equivalent — far narrower than the
# single-camera 90° wide-angle that mirror coverage forced.
CAMERA_VFOV_DEG = 32.0

# Parts the 4-camera variant doesn't have: 3 mirror clamp-mounts +
# 3 mirror glass + camera clamp-bracket + camera PCB + camera lens.
# All stripped from the scene before rendering.
EXCLUDED_PARTS = frozenset({
    "back_upper_clamp_mount",
    "below_left_clamp_mount",
    "below_right_clamp_mount",
    "back_upper_glass",
    "below_left_glass",
    "below_right_glass",
    "camera_clamp_bracket",
    "camera_pcb",
    "camera_lens",
})


detailed = apply_default_detail(assembly).add(
    "brick_2x4_stand_in",
    Box(*BRICK_2X4),
    material="pla_matte",
    color=Color("red"),
)
scene = Scene.from_assembly(detailed).with_materials(DEFAULT_OVERRIDES)
scene = replace(
    scene,
    parts=tuple(p for p in scene.parts if p.name not in EXCLUDED_PARTS),
)


if __name__ == "__main__":
    out_dir = Path("outputs/renders_pov_4cam")
    for name, location in CAMERA_POSES:
        [written] = render(
            scene,
            out=str(out_dir),
            resolution=(1600, 1200),
            world_up=(0.0, 0.0, 1.0),
            framing=FixedPose(
                location=location,
                target=(0.0, 0.0, 0.0),
                vertical_fov_deg=CAMERA_VFOV_DEG,
            ),
        )
        target = written.parent / f"{name}.png"
        written.replace(target)
        print(f"wrote {target}")
