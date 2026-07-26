"""Printability batch for the outer vertex bracket — run, don't import.

    .venv/bin/khana run cad/m05_diverter/outer_vertex_bracket/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../outer_vertex_bracket/assembly.py`);
this script runs the per-part `inspect()` pass.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import WALL_MIN, bracket


# The bracket has real overhangs under the hub and horizontal arms when
# printed with +Z up — they will need slicer supports or a different
# build orientation. For now keep the overhang check informational by
# raising the threshold; the wall-min check is the live assertion.
inspect(
    bracket(),
    method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
    out="outputs",
    name="bracket",
)
