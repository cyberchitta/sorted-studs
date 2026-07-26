"""Printability batch for the dump servo bracket + arm — run, don't import.

    .venv/bin/khana run cad/m03_scanner/dump_servo/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../dump_servo/assembly.py`); this
script runs the per-part `inspect()` pass.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..params import WALL_MIN
from .assembly import servo_arm, servo_bracket


inspect(
    servo_bracket(),
    method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
    out="outputs",
    name="servo_bracket",
)
inspect(
    servo_arm(),
    method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
    out="outputs",
    name="servo_arm",
)
