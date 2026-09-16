"""Printability batch for the drive's printed parts — run, don't import.

    .venv/bin/khana run cad/m02_c_channel_singulator/drive/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../drive/assembly.py`); this script
runs the per-part `inspect()` pass. Bearing and motor are bought and
are not inspected.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import WALL_MIN, bowl, pinion


# The bowl prints floor-down: skirt and slot faces are vertical. The
# only down-facing area above the bed is the skirt foot's first layer
# over the rims of the two outboard M3 counterbores, which reach
# 0.26 mm past the skirt's inner face (params NEMA17_BOLT_CBORE_D).
inspect(
    bowl(), method=FDM(wall_min_mm=WALL_MIN), out="outputs", name="bowl",
    waive={
        "overhang_max": "0.83 mm² at 90° (measured 2026-09-16): two "
        "circular segments of the Ø6 counterbores about the motor "
        "pattern's outboard corners (r ≈ 149.3) that lie past the "
        "skirt's inner face at r 152 — sagitta 0.26 mm, chord ≈ 2.4 mm "
        "each. The skirt's first layer bridges that sliver; nothing "
        "else in the part faces down.",
    },
)
# The pinion prints gear-face down (hub up): the step from the pitch
# cylinder down to the hub is then a top face, not a 90° overhang.
inspect(
    pinion(), method=FDM(wall_min_mm=WALL_MIN, up_axis=(0, 0, -1)),
    out="outputs", name="pinion",
)
