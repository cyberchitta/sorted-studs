"""M03 scanner — GLB export for the subsite scanner-interactive pane.

Geometry-truth artifact: OCP/OpenCascade's ``RWGltf_CafWriter`` writes
the tessellated assembly with per-part names and flat sRGB colors
straight from the Build123d ``Color`` on each ``PlacedPart`` — no
Blender, no PBR. Writes directly into the subsite asset tree so the
local dev server picks it up without an extra copy step.

For the render-truth path (Cycles Principled BSDF baked to glTF PBR
via chitra-cad's material catalog), see ``export_glb_scene.py``.
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from assembly import assembly  # noqa: E402
from detail_variations import apply_default_detail  # noqa: E402

from cad_khana.export import export_glb  # noqa: E402

OUT_DIR = HERE.parent.parent / "subsite" / "assets" / "exhibits" / "scanner"


def main() -> None:
    detailed = apply_default_detail(assembly)
    path = export_glb(detailed, out=OUT_DIR, name="scanner-shape.glb")
    print(f"wrote {path} ({path.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
