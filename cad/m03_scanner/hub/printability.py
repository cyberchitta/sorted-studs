"""Printability batch for the turret hub housing — run, don't import.

    .venv/bin/khana run cad/m03_scanner/hub/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../hub/assembly.py`); this script
runs the per-part `inspect()` pass.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..params import WALL_MIN
from .assembly import housing


# Print orientation: bottom face on the build plate, +Z up
# (build123d default). All bores are vertical cylinders → no
# overhangs to lift the threshold for; default 45° is enough.
inspect(
    housing(),
    method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=45.0),
    out="outputs",
    name="hub_housing",
)
