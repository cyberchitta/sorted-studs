"""Every floor's basket layout, held over the stack turn.

    .venv/bin/khana run cad/m05_diverter/floor_layout_sweep.py

`khana check` on `single_floor.py` covers the **base floor's layout
only** (`FLOOR_LAYOUTS[0]`, the factory default). Floors 1-3 differ on
the basket side, and the tower (`assembly.py`) declares no
basket-vs-rotating pair at all — so without this script three of the
four layouts are never checked against the turning ramp, column and
trap servo at any azimuth.

Each member declares its own `stack_turn` motion (it rides
`build_single_floor`), so every run here is 181 poses, not one. Budget
~2 min for the four.
"""
from cad_khana.mechanism.check import check

from ..design_systems import DEFAULT_SCHEMA
from .params import FLOOR_LAYOUTS
from .single_floor import build_single_floor

for floor in sorted(FLOOR_LAYOUTS):
    check(build_single_floor(DEFAULT_SCHEMA, floor), out=f"outputs/floor_{floor}")
