"""M02 c-channel singulator — photo-real render via cad_khana.render.

Builds the singulator via ``assembly.build_m02`` and hands it to
cad_khana.render. No detail-geometry or
material override layers yet — both are deferred to a later slice
(see ``_notes/state/m02-singulator.md`` "Detailed geometry for rendering").
cad-khana's ``Assembly.with_detailed_geometry`` is available; when m02
grows a ``detail_variations.py``, fold it in here mirroring the m03
``render.py`` pattern.

Lighting and camera framing come from cad_khana.render's defaults; do not
author them here.
"""


from ..design_systems import DEFAULT_SCHEMA, schema_from_env
from .assembly import build_m02

from cad_khana.render.framing import HERO_3Q
from cad_khana.render.render import render
from cad_khana.render.scene import Scene


def build_scene(schema: str = DEFAULT_SCHEMA) -> Scene:
    """The m02 scene under ``schema``.

    A function rather than a module-level value: the schema is a
    parameter now, so building at import would pick one palette
    arbitrarily and pay for the geometry on any import of this module.
    """
    return Scene.from_assembly(build_m02(schema))


if __name__ == "__main__":
    render(
        build_scene(schema_from_env("M02")),
        out="outputs/renders",
        resolution=(1920, 1080),
        world_up=(0.0, 0.0, 1.0),
        framing=HERO_3Q,
        transparent_background=True,
    )
