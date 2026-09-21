"""Printability batch for the chain's printed joints — run, don't import.

    .venv/bin/khana run cad/m02_chain/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check cad/m02_chain/assembly.py`); this
script runs the per-part `inspect()` pass over the five printed joint
bodies from `joints.py`, each at the angles the chain actually places
it at. The 2020 members and the corner brackets are bought and are
not inspected. The LOAD ramp is not inspected here either (it never
has been — open work, not this script's omission to hide).

Each body comes out of the assembly by name (`Assembly.part`) rather
than from a re-typed constructor call. The two forms were byte-equal
on volume and bbox when this changed (2026-09-20), so nothing measured
moves — but the re-typed form was a mirror of nine `params` values, and
a drift in it would have silently inspected a body the chain does not
place.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import build_chain
from .params import BRACE_COLS, S1, S3


# Every joint body is built in its print orientation: the seats
# socket-down (the socket's ceiling over the column's protruding
# corners is roofed at 40° from vertical), the brace collar on its
# flat shared bottom, the feet on their deck face. Nothing faces down
# above the bed, so the default 45° overhang threshold stands. The
# placement the chain gives each body is a translation onto its
# column; `PlacedPart.part` is the body itself, in that print
# orientation.
chain = build_chain()

for name in (
    f"{S1.name}_post_seat",
    f"{S3.name}_post_seat",
    "brace_head",
    *(f"brace_foot_{col}" for col in BRACE_COLS),
):
    inspect(
        chain.part(f"gantry.{name}").part,
        method=FDM(), out="outputs", name=name,
    )
