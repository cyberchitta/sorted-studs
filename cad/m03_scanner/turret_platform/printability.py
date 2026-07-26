"""Printability batch for the turret platform frame — run, don't import.

    .venv/bin/khana run cad/m03_scanner/turret_platform/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../turret_platform/assembly.py`);
this script runs the per-part `inspect()` pass.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..params import WALL_MIN
from .assembly import frame


inspect(
    frame(),
    method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
    out="outputs",
    name="frame",
)
