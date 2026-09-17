"""Printability batch for the drive's printed parts — run, don't import.

    .venv/bin/khana run cad/m02_c_channel_singulator/drive/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../drive/assembly.py`); this script
runs the per-part `inspect()` pass. Bearing and motor are bought and
are not inspected.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..params import MOTOR_AZIMUTH_DEG
from .assembly import WALL_MIN, bowl_part, bowl_part_at, bowl_sectors, pinion


# The bowl prints floor-down: skirt and slot faces are vertical. The
# only down-facing area above the bed is the skirt foot's first layer
# over the rims of the two outboard M3 counterbores, which reach
# 0.26 mm past the skirt's inner face (params NEMA17_BOLT_CBORE_D).
# Inspected as the three printed sectors at the defaults (S1's seam
# phase and motor azimuth); the ears are vertical-walled blocks, so
# nothing new faces down. The waiver belongs to the motor's sector.
_MOTOR_WAIVER = {
    "overhang_max": "0.83 mm² at 90° (measured 2026-09-16): two "
    "circular segments of the Ø6 counterbores about the motor "
    "pattern's outboard corners (r ≈ 149.3) that lie past the "
    "skirt's inner face at r 152 — sagitta 0.26 mm, chord ≈ 2.4 mm "
    "each. The skirt's first layer bridges that sliver; nothing "
    "else in the part faces down.",
}
for k, sector in enumerate(bowl_sectors()):
    name = bowl_part(k)
    inspect(
        sector, method=FDM(wall_min_mm=WALL_MIN), out="outputs", name=name,
        waive=_MOTOR_WAIVER if name == bowl_part_at(MOTOR_AZIMUTH_DEG) else None,
    )
# The pinion prints gear-face down (hub up): the step from the pitch
# cylinder down to the hub is then a top face, not a 90° overhang.
inspect(
    pinion(), method=FDM(wall_min_mm=WALL_MIN, up_axis=(0, 0, -1)),
    out="outputs", name="pinion",
)
