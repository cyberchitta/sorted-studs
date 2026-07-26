"""Printability batch for the basket ring — run, don't import.

    .venv/bin/khana run cad/m05_diverter/basket/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../basket/assembly.py`); this script
runs the per-part `inspect()` pass.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import WALL_MIN, basket


# Inspect the geometrically most interesting variant — N=9 MIDDLE,
# which is the only basket with a column notch cut through its
# interior rather than at a boundary.
inspect(
    basket(half_angle_deg=180.0 / 9, notch_middle=True),
    method=FDM(wall_min_mm=WALL_MIN),
    out="outputs",
    name="basket_090_09",
)
