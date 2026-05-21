"""Render the M03 scanner under every available design system.

Run directly (not via ``chitra render``):

    python cad/m03_scanner/render_all_ds.py

Outputs land flat in the renders directory, one PNG per design system:

    cad/m03_scanner/outputs/renders/<ds_name>.png

Each invocation is an isolated subprocess so design-system module state
doesn't bleed between renders.
"""

import os
import subprocess
import sys
import tempfile
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_RENDER_SCRIPT = _HERE / "render.py"
_CHITRA = Path(__file__).resolve().parents[2] / ".venv" / "bin" / "chitra"
_OUT_DIR = _HERE / "outputs" / "renders"

sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parents[1]))
from cad.design_systems import bind_all  # noqa: E402
from design_systems import MODULE_OF, ROLE_OF  # noqa: E402

# Derived from the canonical bind_all registry so adding a palette in
# cad/design_systems.py automatically picks up here too.
DESIGN_SYSTEMS = list(bind_all(ROLE_OF, MODULE_OF).keys())


def main() -> None:
    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    for ds in DESIGN_SYSTEMS:
        dest = _OUT_DIR / f"{ds}.png"
        with tempfile.TemporaryDirectory() as tmp:
            env = {
                **os.environ,
                "SORTED_STUDS_DS": ds,
                "SORTED_STUDS_RENDERS_OUT": tmp,
            }
            print(f"[{ds}] → {dest}")
            subprocess.run(
                [str(_CHITRA), "render", str(_RENDER_SCRIPT)],
                env=env,
                check=True,
            )
            (Path(tmp) / "render.png").replace(dest)


if __name__ == "__main__":
    main()
