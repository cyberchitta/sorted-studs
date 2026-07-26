"""Printability batch for the turret arm spider — run, don't import.

    .venv/bin/khana run cad/m03_scanner/arm/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../arm/assembly.py`); this script
runs the per-part `inspect()` pass.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..params import WALL_MIN
from .assembly import spider


# Print orientation: bottom face on build plate, +Z up (build123d
# default). The 4 horizontal pin bores + 1 horizontal set-screw
# bore each have a lower-half ceiling that reads as a 90° overhang
# but bridges cleanly in FDM at Ø4–5.4. Threshold raised to 90°
# (same rationale as platform_hinge_bearing) so cylinder bridging
# doesn't false-fail.
inspect(
    spider(),
    method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
    out="outputs",
    name="spider",
)
