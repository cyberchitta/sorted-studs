"""Printability batch for the turret drive bracket — run, don't import.

    .venv/bin/khana run cad/m03_scanner/drive/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../drive/assembly.py`); this script
runs the per-part `inspect()` pass.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..params import WALL_MIN
from .assembly import bracket


# Print orientation: bottom face on the build plate, +Z up
# (build123d default). All bores are vertical and clean. The
# cantilever plate's underside is a 90° overhang requiring print-
# time supports — accepted at the 90° threshold per the slice
# docstring.
inspect(
    bracket(),
    method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
    out="outputs",
    name="drive_bracket",
)
