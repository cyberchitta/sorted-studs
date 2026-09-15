"""sorted-studs — static GLB export of the whole machine for the subsite.

Mirrors the per-module ``export_glb.py`` scripts, but composes
``cad/assembly.py``'s ``build_top`` — m05 diverter + m03 scanner + the
m02 chain on its posts — so the article can show the three units in
their integrated placement. Static (t = 0 of every joint); the
per-module animated panes stay the module scripts' job.

Detail (V-slot profiles, fasteners) is applied per unit *after*
composition: both ``default_detail`` bundles key their overrides by
the qualified path they read off ``placed_parts``, so handing them the
composed tree qualifies the keys for free. m02 carries no detail
bundle yet.
"""

from datetime import date
from pathlib import Path

HERE = Path(__file__).resolve().parent

from .assembly import build_top  # noqa: E402
from .m03_scanner import detail_variations as m03_detail  # noqa: E402
from .m05_diverter import detail_variations as m05_detail  # noqa: E402

from cad_khana.export import export_glb  # noqa: E402

from .design_systems import (  # noqa: E402
    fill_palette_neutrals,
    neutral_of,
    schema_from_env,
)

# The neutral fill is a property of the palette's axis, not of a unit;
# m05's binding of the shared palettes answers for the whole machine.
from .m05_diverter.design_systems import design_system  # noqa: E402

OUT_DIR = HERE.parent / "subsite" / "assets" / "exhibits"


def main() -> None:
    # Entry point owns env resolution; one name feeds both the geometry
    # and the filename tag.
    schema = schema_from_env()
    top = build_top(schema)
    detailed = m05_detail.apply_default_detail(
        m03_detail.apply_default_detail(top)
    )
    filled = fill_palette_neutrals(detailed, neutral_of(design_system(schema)))
    tag = schema.removeprefix("cyberchitta-")
    path = export_glb(
        filled,
        out=OUT_DIR,
        name=f"machine-{date.today():%y%m%d}-{tag}.glb",
    )
    print(f"wrote {path} ({path.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
