"""M05 diverter — photo-real render via chitra-cad.

Imports the composed top-level assembly (already module-level in
``assembly.py``) and applies the detail-fidelity layer before
handing it to chitra-cad:

  * ``detail_variations.apply_default_detail`` — swap each cheap
    20×20×L Box stub for a real V-slot profile from ``bd_warehouse``.
    Same pose, same material/color; only the underlying shape
    changes. Fasteners / joint hardware are deferred to a follow-up
    slice (see ``_notes/state/m05-diverter.md``).

Lighting and camera framing come from chitra-cad's defaults; do not
author them here.
"""


from .assembly import assembly
from .detail_variations import apply_default_detail

from chitra_cad.framing import HERO_3Q
from chitra_cad.render import render
from chitra_cad.scene import Scene


detailed = apply_default_detail(assembly)
scene = Scene.from_assembly(detailed)


if __name__ == "__main__":
    render(
        scene,
        out="outputs/renders",
        resolution=(1920, 1080),
        world_up=(0.0, 0.0, 1.0),
        framing=HERO_3Q,
    )
