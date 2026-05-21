"""M05 diverter — GLB export for the subsite diverter-interactive pane.

Mirrors ``m03_scanner/export_glb_shape.py``: geometry-truth static GLB
from the *exhibit* assembly (2-floor stripped variant), not the
production 4-floor tower. The model-viewer in
``subsite/index.md`` reads this artifact for the interactive,
non-animated diverter pane; an animated variant is deferred.
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from exhibit_assembly import assembly  # noqa: E402
from detail_variations import apply_default_detail  # noqa: E402

from cad_khana.export import export_glb  # noqa: E402

OUT_DIR = HERE.parent.parent / "subsite" / "assets" / "exhibits"


def main() -> None:
    detailed = apply_default_detail(assembly)
    path = export_glb(detailed, out=OUT_DIR, name="diverter-260520.glb")
    print(f"wrote {path} ({path.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
