"""M03 scanner — photo-real render via chitra-cad.

Imports the composed top-level assembly (already module-level in
``assembly.py``) and applies two override layers before handing it to
chitra-cad:

  * ``detail_variations.default_detail`` — swap each cheap 20×20×L
    Box stub for a real V-slot profile from ``bd_warehouse``. Same
    pose, same material/color; only the underlying shape changes.
  * ``material_variations.DEFAULT_OVERRIDES`` — the render-only
    material overrides for parts whose real-world material isn't yet
    representable in chitra-cad's table. Settled materials live on
    the assembly itself; see ``material_variations.py`` for the
    promotion convention.

Lighting and camera framing come from chitra-cad's defaults; do not
author them here.
"""

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from assembly import assembly  # noqa: E402
from detail_variations import apply_default_detail  # noqa: E402
from material_variations import DEFAULT_OVERRIDES  # noqa: E402

from chitra_cad.framing import HERO_3Q  # noqa: E402
from chitra_cad.render import render  # noqa: E402
from chitra_cad.scene import Scene  # noqa: E402


detailed = apply_default_detail(assembly)
scene = Scene.from_assembly(detailed).with_materials(DEFAULT_OVERRIDES)


if __name__ == "__main__":
    out_dir = os.environ.get("SORTED_STUDS_RENDERS_OUT", "outputs/renders")
    render(
        scene,
        out=out_dir,
        resolution=(1920, 1080),
        world_up=(0.0, 0.0, 1.0),
        framing=HERO_3Q,
    )
