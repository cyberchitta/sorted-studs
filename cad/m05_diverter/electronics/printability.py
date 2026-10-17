"""Printability batch for the electronics plate — run, don't import.

    .venv/bin/khana run cad/m05_diverter/electronics/printability.py

Orchestration only: `assembly.py` declares the plates and their claims
(`khana check .../electronics/assembly.py`); this script runs the
per-part `inspect()` pass. Both plates are the same body; the hangers
and boards are bought.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import board_plate

WALL_MIN = 1.5

# Prints flat, +Z up: a plain slab, nothing faces down above the bed.
inspect(
    board_plate(), method=FDM(wall_min_mm=WALL_MIN), out="outputs",
    name="board_plate", solid_count=1,
)
