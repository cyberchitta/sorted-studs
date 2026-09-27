"""Printability batch for the ESP32-CAM clamp mount — run, don't import.

    .venv/bin/khana run cad/m03_scanner/camera_mount_esp32cam/printability.py

Orchestration only: `assembly.py` declares the geometry; its mechanism
claims run under `khana check .../camera_mount_esp32cam/assembly.py`
(the below-left body only), and the per-camera clamp-bolt access and
clamp-base claims under `khana check cad/m03_scanner/assembly.py`
(`build_camera_rig`, all four). This script runs the per-part
`inspect()` pass, which neither of those covers.

All four placed bodies are inspected, each with the `mount_in_clamp`
the rig gives it — one function, four different bodies — and **no
waivers**. Until 2026-09-27 this ran red: a 0.70 mm wall at the
below-* outer clamp bolt (the plate's unbounded front trim had sliced
the clamp base), a 0.16 mm knife-edge where the back-upper -x bolt bore
cut the plate's low corner, and overhangs on three of the four plates
(front-low 65°, back-upper 45.5°, below-* 55.4° under a waiver). The
fixes are geometry in `clamp_mount` — base exempt from the front trim,
a plate-edge notch at the back-upper -x bolt, edge chamfers, a back
wedge and teardrop M2 bores, all shaped against `PRINT_UP` — see
`_notes/state/m03-scanner.md`.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..assembly import build_m03
from ..params import WALL_MIN
from .assembly import PRINT_UP


# Print orientation: clamp base on the build plate, clamp-local +Y up
# — the direction `clamp_mount` shapes each plate against.
method = FDM(up_axis=PRINT_UP, wall_min_mm=WALL_MIN)

for pp in build_m03().placed_parts:
    if not pp.name.endswith("_clamp_mount"):
        continue
    leaf = pp.name.rsplit(".", 1)[-1]
    inspect(pp.part, method=method, out="outputs", name=leaf)
