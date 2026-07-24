"""M03 scanner — 4-camera POV render via chitra-cad.

Sister to ``render.py``. Where ``render.py`` shoots the rig from the
default 3/4 hero pose, this script pins the camera to each of the four
tetrahedral vertices in turn (``VERTEX_*_POS`` in the module-level
``params.py``) and renders one PNG per viewpoint. Each
camera looks at the world origin (the brick stand-in).

A solid red 2×4-footprint ``Box`` (no studs) sits at the world origin
as the focal target; geometric occlusion + framing is the question,
not stud detail.

Outputs land in ``outputs/renders_pov/`` as
``cam_front_low.png``, ``cam_back_upper.png``,
``cam_below_left.png``, ``cam_below_right.png``.

Coordinate frame: world Z-up. Vertex positions come from the
module-level ``params.py`` (the manual `sizing.py` mirror).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from assembly import assembly  # noqa: E402
from params import (  # noqa: E402
    VERTEX_BACK_UPPER_POS,
    VERTEX_BELOW_LEFT_POS,
    VERTEX_BELOW_RIGHT_POS,
    VERTEX_FRONT_LOW_POS,
)
from detail_variations import apply_default_detail  # noqa: E402
from material_variations import DEFAULT_OVERRIDES  # noqa: E402

from build123d import Box, Color  # noqa: E402
from chitra_cad.framing import FixedPose  # noqa: E402
from chitra_cad.render import render  # noqa: E402
from chitra_cad.scene import Scene  # noqa: E402


BRICK_2X4 = (31.8, 15.8, 9.6)

# Four cameras at the tetrahedral vertices. Each renders from the PCB
# rear plane (the optical reference point on the mount); the M12 lens
# stack extends ~25 mm forward but for this sanity-render we shoot
# from the PCB plane directly — the mount-front is excluded from the
# scene below so we don't render from inside opaque geometry.
CAMERA_POSES = [
    ("cam_front_low", VERTEX_FRONT_LOW_POS),
    ("cam_back_upper", VERTEX_BACK_UPPER_POS),
    ("cam_below_left", VERTEX_BELOW_LEFT_POS),
    ("cam_below_right", VERTEX_BELOW_RIGHT_POS),
]

# Single uniform VFOV. All 4 cameras sit ≈ 250 mm from origin, so the
# same M12 board lens fills each frame consistently:
#   HFOV = 2·atan((90 + 5) / 250) ≈ 41.6°
#   VFOV at 4:3 ≈ 2·atan(0.75 · tan(HFOV/2)) ≈ 32.0°
# This is a ~5–6 mm M12 lens equivalent — the design target for the
# AI-Thinker ESP32-CAM M12 variant.
CAMERA_VFOV_DEG = 32.0

# Each camera shoots through its own mount; exclude the rendering
# camera's own PCB + lens + clamp body from the scene before
# rendering, so we don't see the inside of the housing in front of
# the lens. Per-camera in the loop below.


detailed = apply_default_detail(assembly).with_part(
    "brick_2x4_stand_in",
    Box(*BRICK_2X4),
    material="pla_matte",
    color=Color("red"),
)
base_scene = Scene.from_assembly(detailed).with_materials(DEFAULT_OVERRIDES)


def _scene_excluding(prefix: str) -> Scene:
    """Scene with the named camera's own clamp_mount + pcb + lens
    parts stripped, so the camera doesn't render from inside its own
    housing."""
    from dataclasses import replace
    excluded = frozenset(
        {f"{prefix}_clamp_mount", f"{prefix}_pcb", f"{prefix}_lens"}
    )
    return replace(
        base_scene,
        parts=tuple(p for p in base_scene.parts if p.name not in excluded),
    )


if __name__ == "__main__":
    out_dir = Path("outputs/renders_pov")
    for prefix, location in CAMERA_POSES:
        scene = _scene_excluding(prefix)
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
        target = written.parent / f"{prefix}.png"
        written.replace(target)
        print(f"wrote {target}")
