"""Printability batch for the chain's printed joints — run, don't import.

    .venv/bin/khana run cad/m02_chain/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check cad/m02_chain/assembly.py`); this
script runs the per-part `inspect()` pass over the five printed joint
bodies from `joints.py`, each at the angles the chain actually places
it at. The 2020 members and the corner brackets are bought and are
not inspected. The LOAD ramp is not inspected here either (it never
has been — open work, not this script's omission to hide).
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .joints import brace_foot, brace_head, post_seat
from .params import (
    ARM_POST,
    BRACE_COLS,
    BRACE_FOOT_START_H,
    BRACE_FOOT_STOP_H,
    BRACE_HEAD_STOP_H,
    PROP_POST,
    S1,
    S3,
    brace_run,
    m05_column_rot_deg,
    prop_bar_azimuth_deg,
)


# Every joint body is built in its print orientation: the seats
# socket-down (the socket's ceiling over the column's protruding
# corners is roofed at 40° from vertical), the brace collar on its
# flat shared bottom, the feet on their deck face. Nothing faces down
# above the bed, so the default 45° overhang threshold stands.
for s in (S1, S3):
    inspect(
        post_seat(prop_bar_azimuth_deg(s), m05_column_rot_deg(PROP_POST[s.name])),
        method=FDM(), out="outputs", name=f"{s.name}_post_seat",
    )
inspect(
    brace_head(
        tuple(brace_run(col)[:2] for col in BRACE_COLS), BRACE_HEAD_STOP_H, m05_column_rot_deg(ARM_POST),
    ),
    method=FDM(), out="outputs", name="brace_head",
)
for col in BRACE_COLS:
    az, el, _ = brace_run(col)
    inspect(
        brace_foot(az + 180.0, el, BRACE_FOOT_STOP_H, BRACE_FOOT_START_H[col], flange=(col == 3)),
        method=FDM(), out="outputs", name=f"brace_foot_{col}",
    )
