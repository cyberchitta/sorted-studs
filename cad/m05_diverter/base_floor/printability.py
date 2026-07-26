"""Printability batch for the base floor's printed parts — run, don't import.

    .venv/bin/khana run cad/m05_diverter/base_floor/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../base_floor/assembly.py`); this
script runs the per-part `inspect()` pass.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import WALL_MIN, motor_mount, ring_gear


method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0)
# pinion is bought (metal, on the motor shaft) — not inspected.
inspect(ring_gear(), method=method, out="outputs", name="ring_gear")
inspect(motor_mount(), method=method, out="outputs", name="motor_mount")
