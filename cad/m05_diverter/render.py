"""M05 diverter — photo-real render via cad_khana.render.

Builds the tower via ``assembly.build_m05`` and applies the
detail-fidelity layer before handing it to cad_khana.render:

  * ``detail_variations.apply_default_detail`` — swap each cheap
    20×20×L Box stub for a real V-slot profile from ``bd_warehouse``.
    Same pose, same material/color; only the underlying shape
    changes. Fasteners / joint hardware are deferred to a follow-up
    slice (see ``_notes/state/m05-diverter.md``).

Lighting and camera framing come from cad_khana.render's defaults; do not
author them here.
"""


from ..design_systems import DEFAULT_SCHEMA, schema_from_env
from .assembly import build_m05
from .detail_variations import apply_default_detail

from cad_khana.render.framing import HERO_3Q
from cad_khana.render.render import render
from cad_khana.render.scene import Scene


def build_scene(schema: str = DEFAULT_SCHEMA) -> Scene:
    """The m05 scene under ``schema``, detail layer applied.

    A function rather than a module-level value: the schema is a
    parameter now, so building at import would pick one palette
    arbitrarily and pay for the geometry on any import of this module.
    """
    return Scene.from_assembly(apply_default_detail(build_m05(schema)))


if __name__ == "__main__":
    render(
        build_scene(schema_from_env("M05")),
        out="outputs/renders",
        resolution=(1920, 1080),
        world_up=(0.0, 0.0, 1.0),
        framing=HERO_3Q,
    )
