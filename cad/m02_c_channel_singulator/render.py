"""M02 c-channel singulator — photo-real render via chitra-cad.

Imports the composed top-level assembly (already module-level in
``assembly.py``) and hands it to chitra-cad. No detail-geometry or
material override layers yet — both are deferred to a later slice
(see ``_notes/state/m02-singulator.md`` "Detailed geometry for rendering"). When the
cad-khana ``Assembly.with_detailed_geometry`` API lands and m02
grows a ``detail_variations.py``, fold it in here mirroring the m03
``render.py`` pattern.

Lighting and camera framing come from chitra-cad's defaults; do not
author them here.
"""


from .assembly import assembly

from chitra_cad.framing import HERO_3Q
from chitra_cad.render import render
from chitra_cad.scene import Scene


scene = Scene.from_assembly(assembly)


if __name__ == "__main__":
    render(
        scene,
        out="outputs/renders",
        resolution=(1920, 1080),
        world_up=(0.0, 0.0, 1.0),
        framing=HERO_3Q,
        transparent_background=True,
    )
