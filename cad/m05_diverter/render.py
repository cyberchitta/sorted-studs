"""M05 diverter — photo-real render via chitra-cad.

Imports the composed top-level assembly (already module-level in
``assembly.py``) and applies the detail-fidelity layer before
handing it to chitra-cad:

  * ``detail_variations.apply_default_detail`` — swap each cheap
    20×20×L Box stub for a real V-slot profile from ``bd_warehouse``.
    Same pose, same material/color; only the underlying shape
    changes. Fasteners / joint hardware are deferred to a follow-up
    slice (see ``NEXT_STEPS.md``).

Lighting and camera framing come from chitra-cad's defaults; do not
author them here.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from assembly import assembly  # noqa: E402
from detail_variations import apply_default_detail  # noqa: E402

from chitra_cad.render import render  # noqa: E402
from chitra_cad.scene import Scene  # noqa: E402


detailed = apply_default_detail(assembly)
scene = Scene.from_assembly(detailed)


if __name__ == "__main__":
    render(
        scene,
        out="renders",
        resolution=(1920, 1080),
        world_up=(0.0, 0.0, 1.0),
        camera_direction=(1.0, -1.0, 0.7),
    )
