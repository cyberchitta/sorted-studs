"""Render the M03 scanner under every available design system.

    python -m cad.m03_scanner.render_all_ds        # from the repo root

Outputs land flat in the renders directory, one PNG per design system:

    cad/m03_scanner/outputs/renders/<ds_name>.png

One process, one Blender, a plain loop over the palettes. Until
2026-07-26 each palette needed its own ``chitra render`` subprocess
purely to get a fresh set of module-level design-system bindings —
the schema was captured at import, so nothing short of a new
interpreter could change it. Now ``render_m03`` takes the schema as an
argument.

That subprocess path had also been broken since the 2026-07-24 package
refactor: it invoked ``chitra render cad/m03_scanner/render.py``, and
``chitra render`` is ``runpy.run_path(..., run_name="__main__")``,
which cannot resolve this file's relative imports
(``ImportError: attempted relative import with no known parent
package``). Hence ``python -m`` above — a package-aware entry point.
"""

from __future__ import annotations

import tempfile
from pathlib import Path

from ..design_systems import PALETTE_NAMES
from .render import render_m03

_HERE = Path(__file__).resolve().parent
_OUT_DIR = _HERE / "outputs" / "renders"


def main() -> None:
    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    for ds in PALETTE_NAMES:
        dest = _OUT_DIR / f"{ds}.png"
        # chitra-cad names its single-frame output ``render.png``, so
        # each pass renders into a scratch dir and moves the result to
        # the palette-named destination.
        with tempfile.TemporaryDirectory() as tmp:
            print(f"[{ds}] → {dest}")
            render_m03(ds, tmp)
            (Path(tmp) / "render.png").replace(dest)


if __name__ == "__main__":
    main()
