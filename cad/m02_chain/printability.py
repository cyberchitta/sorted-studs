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

# A vertical cut meeting a face parallel to the 41.9° brace leaves a
# 48.1° edge (90° − elevation) in each foot: the stop wall's top ridge,
# and the channel floor's lip at the block's end face. The crease rays
# read ~0.59 there (foot_3 at the ridge, foot_5 at the lip; each foot
# has both, and only the thinner shows). Alignment 0.65 ≈ cos 48.1° =
# 0.67 is the edge's own dihedral, not two near-parallel faces, and a
# chamfer on the ridge would take height off the face the brace bears
# on. Owner's call, 2026-09-22.
FOOT_WAIVE = {
    "wall_min": "48.1° wedge tip, not a sliver: min_wall_at on the stop "
                "wall's top ridge (foot_3) or the channel floor's end lip "
                "(foot_5), alignment 0.65 ≈ cos 48.1°; the edge prints as "
                "an ordinary 48° corner",
}

for name, waive in (
    (f"{S1.name}_post_seat", None),
    (f"{S3.name}_post_seat", None),
    ("brace_head", None),
    *((f"brace_foot_{col}", FOOT_WAIVE) for col in BRACE_COLS),
):
    inspect(
        chain.part(f"gantry.{name}").part,
        method=FDM(), out="outputs", name=name, waive=waive,
    )
