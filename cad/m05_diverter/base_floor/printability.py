"""Printability batch for the base floor's printed parts — run, don't import.

    .venv/bin/khana run cad/m05_diverter/base_floor/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../base_floor/assembly.py`); this
script runs the per-part `inspect()` pass.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import WALL_MIN, catch_cradle, catch_tray, motor_mount, ring_gear


# Default 45° threshold: no part here has a downward face above the
# bed (overhang reads null at any threshold, 2026-09-22), so the 90°
# this used to carry switched off nothing — and now a new overhang
# fails instead of passing unseen. Every part prints as installed, up
# +Z: the mount on its bottom face, the
# tray on its floor, the cradle on its shelf with the posts rising.
method = FDM(wall_min_mm=WALL_MIN)
# pinion is bought (metal, on the motor shaft) — not inspected.
inspect(ring_gear(), method=method, out="outputs", name="ring_gear")
inspect(motor_mount(), method=method, out="outputs", name="motor_mount")
inspect(catch_tray(), method=method, out="outputs", name="catch_tray")
inspect(catch_cradle(), method=method, out="outputs", name="catch_cradle")
