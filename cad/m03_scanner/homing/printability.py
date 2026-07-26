"""Printability batch for the homing flag + bracket — run, don't import.

    .venv/bin/khana run cad/m03_scanner/homing/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../homing/assembly.py`); this script
runs the per-part `inspect()` pass.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..params import WALL_MIN
from .assembly import bracket, flag


# Print orientation: bottom face on the build plate, +Z up
# (build123d default). Flag is a thin disc with a centered bore +
# offset detect hole — all vertical bores, clean print at default
# 45° overhang threshold.
inspect(
    flag(),
    method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=45.0),
    out="outputs",
    name="homing_flag",
)
# Bracket: foot + vertical post, no horizontal cantilever. Default
# 45° threshold suffices.
inspect(
    bracket(),
    method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=45.0),
    out="outputs",
    name="homing_bracket",
)
