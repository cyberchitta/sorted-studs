"""M03 scanner — photo-real render via chitra-cad.

Imports the composed top-level assembly (already module-level in
``assembly.py``) and applies the render-only material overrides for
parts whose real-world material isn't yet representable in chitra-cad's
table. Settled materials live on the assembly itself; see
``material_variations.py`` for the override list and the promotion
convention. Lighting and camera framing come from chitra-cad's
defaults; do not author them here.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from assembly import assembly  # noqa: E402
from material_variations import DEFAULT_OVERRIDES  # noqa: E402

from chitra_cad.render import render  # noqa: E402
from chitra_cad.scene import Scene  # noqa: E402


scene = Scene.from_assembly(assembly).with_materials(DEFAULT_OVERRIDES)


if __name__ == "__main__":
    render(scene, out="renders", resolution=(1920, 1080))
