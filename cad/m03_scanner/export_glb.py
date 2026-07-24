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

from ..design_systems import Role, fill_palette_neutrals, resolve_name  # noqa: E402

from .design_systems import DESIGN_SYSTEM  # noqa: E402

OUT_DIR = HERE.parent.parent / "subsite" / "assets" / "exhibits"


def main() -> None:
    detailed = apply_default_detail(build_m03_at(0.0))
    filled = fill_palette_neutrals(detailed, DESIGN_SYSTEM.palette[Role.SECONDARY_STRUCTURE])
    palette = resolve_name("M03")
    tag = palette.removeprefix("cyberchitta-")
    path = export_glb(
        filled,
        out=OUT_DIR,
        name=f"scanner-{date.today():%y%m%d}-{tag}.glb",
    )
    print(f"wrote {path} ({path.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
