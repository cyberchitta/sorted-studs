"""
m02 chain — the LOAD beam-break against the brick that has to break it.
A CHECK module: probe fixture + the claims it exists for.

    khana check cad/m02_chain/check_load_beam.py

`khana check cad/m02_chain/assembly.py` covers the beam-break's
geometry: the beam's core clear of the ramp (`load_beam_clear_of_ramp`),
each half on its face, the tab and the receiver inside the stator cut
and clear of S1. It does not say a brick breaks the beam — that needs a
brick, and a probe solid lives only here (nothing imports a check
module, so no export ever carries it).

The claim: the thinnest brick there is, a plate or tile lying flat
(`params.py` THINNEST_BRICK_H × a 1×1's THINNEST_BRICK_W), resting on
the floor at the beam station, covers the beam's core seen along the
beam — every ray of the core meets it. The beam is parallel to the
floor and square to the run, so where across the chute the plate lies
does not matter; it is placed mid-chute. Whether covering the core is
what the receiver counts as a break is the placeholder in
`BEAM_CORE_D`, not a measurement — a bench question.
"""

from __future__ import annotations

from build123d import Box, Pos

from cad_khana.mechanism.assembly import Assembly

from ..design_systems import DEFAULT_SCHEMA
from .assembly import build_chain, load_beam_core, load_beam_frame
from .params import BEAM_S, S1, RAMP_WIDTH, RAMP_WALL_T, THINNEST_BRICK_H, THINNEST_BRICK_W


def build_load_beam_check(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The ramp as the chain places it, the beam's core and a flat
    plate on the floor under it."""
    ramp = build_chain(schema).part("load_ramp")
    frame = load_beam_frame()
    across = (frame * Pos(0, 1, 0)).position - frame.position
    mid = (RAMP_WIDTH / 2.0 - RAMP_WALL_T) / 2.0      # between the +Y wall and the floor's centreline
    plate = Pos(BEAM_S, mid, THINNEST_BRICK_H / 2.0) * Box(THINNEST_BRICK_W, THINNEST_BRICK_W, THINNEST_BRICK_H)
    return (
        Assembly()
        .with_part("load_ramp", ramp.part, location=ramp.location, material=ramp.material)
        .with_part("beam_core", load_beam_core(S1))
        .with_part("thinnest_plate", plate, location=frame)
        # The plate rests on the floor (so the probe is where a brick is) …
        .assert_tangent_contact("thinnest_plate", "load_ramp")
        # … and hides the whole core from the receiver.
        .assert_within("beam_core", "thinnest_plate", along=tuple(across))
    )


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``.
assembly = build_load_beam_check(DEFAULT_SCHEMA)
