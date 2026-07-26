"""M03 scanner — static GLB export for the subsite scanner pane.

Geometry-truth static still at ``t=0`` of the animation parameter,
mirroring ``m05_diverter/export_glb.py``. The model-viewer in
``subsite/index.md`` reads this artifact when the still variant is
the right call (e.g. light/dark palette stills for review); the
animated variant lives in ``export_glb_animated.py``.

Filename embeds the active palette so palette sweeps don't overwrite
each other.
"""

from datetime import date
from pathlib import Path

HERE = Path(__file__).resolve().parent

from .animated_assembly import build_m03_at  # noqa: E402
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
    # This script is the entry point, so it is what resolves the schema
    # from the environment — and it passes the *same* name to the
    # builder and to the filename tag, so the two can't disagree.
    schema = schema_from_env("M03")
    detailed = apply_default_detail(build_m03_at(0.0, schema))
    filled = fill_palette_neutrals(detailed, neutral_of(design_system(schema)))
    tag = schema.removeprefix("cyberchitta-")
    path = export_glb(
        filled,
        out=OUT_DIR,
        name=f"scanner-{date.today():%y%m%d}-{tag}.glb",
    )
    print(f"wrote {path} ({path.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
