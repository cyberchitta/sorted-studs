"""M03 scanner — GLB export from a chitra-cad ``Scene``.

Render-truth artifact: Blender's glTF exporter bakes each part's
Cycles Principled BSDF (driven by chitra-cad's material catalog +
``DEFAULT_OVERRIDES``) to glTF PBR (baseColor, metallic-roughness,
IOR, transmission). Heavier than the shape path
(``export_glb_shape.py``) but carries the same material story as the
photo-real renders. Pick this when the subsite viewer should look
like the renders; pick the shape path when it should look like the
engineering model.

Writes directly into the subsite asset tree so the local dev server
picks it up without an extra copy step.
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from assembly import assembly  # noqa: E402
from detail_variations import apply_default_detail  # noqa: E402
from material_variations import DEFAULT_OVERRIDES  # noqa: E402

from chitra_cad.export import export_glb  # noqa: E402
from chitra_cad.scene import Scene  # noqa: E402

OUT_DIR = HERE.parent.parent / "subsite" / "assets" / "exhibits" / "scanner"


def main() -> None:
    detailed = apply_default_detail(assembly)
    scene = Scene.from_assembly(detailed).with_materials(DEFAULT_OVERRIDES)
    path = export_glb(scene, out=OUT_DIR, name="scanner-scene.glb")
    print(f"wrote {path} ({path.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
