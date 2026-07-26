"""Printability batch for the inner vertex bracket — run, don't import.

    .venv/bin/khana run cad/m05_diverter/inner_vertex_bracket/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../inner_vertex_bracket/assembly.py`);
this script runs the per-part `inspect()` pass.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import WALL_MIN, bracket


# Real overhangs under the hub and arms when printed with +Z up;
# keep overhang check informational (matches outer_vertex_bracket).
inspect(
    bracket(),
    method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
    out="outputs",
    name="bracket",
)
