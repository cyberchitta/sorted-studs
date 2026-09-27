"""Printability batch for the drive's printed parts — run, don't import.

    .venv/bin/khana run cad/m02_c_channel_singulator/drive/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../drive/assembly.py`); this script
runs the per-part `inspect()` pass. Bearing and motor are bought and
are not inspected.

The pinion is the same body on every stage, so it is inspected here.
The three bowl sectors are not: each stage cuts its bowl at its own
seam phase and motor azimuth, which only the chain knows, so every
stage's sectors — and each motor sector's overhang waiver, witnessed on
that stage's body — are in `cad/m02_chain/printability.py`.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import WALL_MIN, pinion


# The pinion prints gear-face down (hub up): the step from the pitch
# cylinder down to the hub is then a top face, not a 90° overhang.
inspect(
    pinion(), method=FDM(wall_min_mm=WALL_MIN, up_axis=(0, 0, -1)),
    out="outputs", name="pinion",
)
