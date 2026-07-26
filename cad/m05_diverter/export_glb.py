"""M05 diverter — GLB export for the subsite diverter-interactive pane.

Mirrors ``m03_scanner/export_glb_shape.py``: geometry-truth static GLB
from the *exhibit* assembly (2-floor stripped variant), not the
production 4-floor tower. The model-viewer in
``subsite/index.md`` reads this artifact for the interactive,
non-animated diverter pane; an animated variant is deferred.
"""

from datetime import date
from pathlib import Path

HERE = Path(__file__).resolve().parent

from .exhibit_assembly import build_exhibit  # noqa: E402
from .detail_variations import apply_default_detail  # noqa: E402

from cad_khana.export import export_glb  # noqa: E402

from ..design_systems import (  # noqa: E402
    fill_palette_neutrals,
    neutral_of,
    schema_from_env,
)

from .design_systems import design_system  # noqa: E402

OUT_DIR = HERE.parent.parent / "subsite" / "assets" / "exhibits"


def main() -> None:
    # Entry point owns env resolution; one name feeds both the geometry
    # and the filename tag.
    schema = schema_from_env("M05")
    detailed = apply_default_detail(build_exhibit(schema))
    filled = fill_palette_neutrals(detailed, neutral_of(design_system(schema)))
    tag = schema.removeprefix("cyberchitta-")
    path = export_glb(
        filled,
        out=OUT_DIR,
        name=f"diverter-{date.today():%y%m%d}-{tag}.glb",
    )
    print(f"wrote {path} ({path.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
