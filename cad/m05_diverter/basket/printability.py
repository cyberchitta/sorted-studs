"""Printability batch for the basket ring — run, don't import.

    .venv/bin/khana run cad/m05_diverter/basket/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../basket/assembly.py`); this script
runs the per-part `inspect()` pass.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..params import FLOOR_LAYOUTS
from .assembly import WALL_MIN, basket


# Every distinct printed body across all four floor layouts: a basket
# is fixed by its size N and which boundaries carry a column channel.
# Until 2026-09-22 only the N=6 with both channels was inspected.
bodies = sorted({
    (n, "left" in notches, "right" in notches)
    for layout in FLOOR_LAYOUTS.values()
    for _, n, notches in layout
})
for n, left, right in bodies:
    inspect(
        basket(half_angle_deg=180.0 / n, notch_left=left, notch_right=right),
        method=FDM(wall_min_mm=WALL_MIN),
        out="outputs",
        name=f"basket_{n:02d}{'_L' if left else ''}{'_R' if right else ''}",
    )
