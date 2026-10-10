"""Printability of the tower's own printed part — run, don't import.

    .venv/bin/khana run cad/m05_diverter/printability.py

`khana check` on the sibling `assembly.py` covers the tower's mechanism
claims and no printability at all. This script inspects the one printed
part the tower root itself owns, the ceiling sleeve; every other
printed part is inspected by the sub-assembly that builds it
(`ramp_mechanism/role_sweep.py`, `basket/printability.py`, …).
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..design_systems import DEFAULT_SCHEMA
from .assembly import CEILING_SLEEVE_UP_AXIS, build_m05
from .ramp_mechanism.assembly import WALL_MIN


# The body the tower places, in its own frame: lip down, as installed
# and as printed. The lip's underside is the bed and the barrel steps
# *in* above it, so nothing faces down off the bed.
inspect(
    build_m05(DEFAULT_SCHEMA).part("ceiling_sleeve").part,
    method=FDM(up_axis=CEILING_SLEEVE_UP_AXIS, wall_min_mm=WALL_MIN),
    out="outputs", name="ceiling_sleeve", solid_count=1,
)
