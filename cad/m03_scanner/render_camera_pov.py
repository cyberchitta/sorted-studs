"""M03 scanner — camera-POV render via chitra-cad.

Sister script to ``render.py``. Where ``render.py`` shoots the rig from
the default 3/4 hero pose, this one pins the camera to the actual Pi HQ
sensor pose (``CAMERA_POS`` in ``camera_rig/params.py``) with the lens's
real field of view, so we can verify all four viewpoints — the direct
top-front view plus the three first-surface-mirror reflections — fall
in-frame and unoccluded at the rig's current geometry.

A solid red 2×4-footprint ``Box`` (no studs) sits at the world origin as
the focal target; geometric occlusion + framing is the question, not
stud detail.

Coordinate frame: world Z-up, matching the rest of the m03 module. The
camera pose and field of view come from the rig parameters and the lens
spec in ``docs/m03-multi-view-scanner.md`` (Pi HQ sensor 7.4 × 5.6 mm,
2.8 mm C-mount → VFOV ≈ 90°, aspect 4:3).
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from assembly import assembly  # noqa: E402
from camera_rig.params import CAMERA_POS, CAMERA_ROT_DEG  # noqa: E402
from detail_variations import apply_default_detail  # noqa: E402
from material_variations import DEFAULT_OVERRIDES  # noqa: E402

from build123d import Box, Color  # noqa: E402
from chitra_cad.framing import FixedPose  # noqa: E402
from chitra_cad.render import render  # noqa: E402
from chitra_cad.scene import Scene  # noqa: E402


BRICK_2X4 = (31.8, 15.8, 9.6)
CAMERA_VFOV_DEG = 90.0

# CAMERA_POS is the PCB *rear* face (mounting reference). The lens body
# extends forward of that along the lens axis by PCB_T + LENS_L = 1.6 + 55
# = 56.6 mm (values mirror the private constants in
# camera_clamp_bracket/assembly.py). Synth camera sits at the lens nose so
# we don't render from inside the camera body.
_LENS_STACK_L = 56.6
_TILT_RAD = math.radians(CAMERA_ROT_DEG[0])
_LENS_AXIS = (0.0, -math.sin(_TILT_RAD), math.cos(_TILT_RAD))
LENS_NOSE_POS = tuple(c + _LENS_STACK_L * a for c, a in zip(CAMERA_POS, _LENS_AXIS))

detailed = apply_default_detail(assembly).add(
    "brick_2x4_stand_in",
    Box(*BRICK_2X4),
    material="pla_matte",
    color=Color("red"),
)
scene = Scene.from_assembly(detailed).with_materials(DEFAULT_OVERRIDES)


if __name__ == "__main__":
    render(
        scene,
        out="outputs/renders_pov",
        resolution=(2048, 1536),
        world_up=(0.0, 0.0, 1.0),
        framing=FixedPose(
            location=LENS_NOSE_POS,
            target=(0.0, 0.0, 0.0),
            vertical_fov_deg=CAMERA_VFOV_DEG,
        ),
    )
