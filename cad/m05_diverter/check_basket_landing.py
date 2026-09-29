"""Which basket a brick lands in — a probe fixture, not the floor.

    .venv/bin/khana check cad/m05_diverter/check_basket_landing.py

The collision claims (`single_floor.py`, `floor_layout_sweep.py`) hold
the turning ramp clear of the baskets; none of them says the brick
lands in the basket the ramp faces. A brick leaves the ramp anywhere
across the channel's exit width, so for every basket of every floor
layout this fixture turns the stack to the aim `aim.py` computes for it
and stands a probe as wide as that floor's ramp exit
(`aim.ramp_exit_w_for` the layout) under the tip: from `RAMP_TIP_X` out
`LANDING_DEPTH`, through the band just below the rim where the radial
walls and the column channels' linings stand. The probe must clear its
own basket — whose side walls and linings are the only thing between
it and a neighbour or a floorless column channel — by `LANDING_MARGIN`.
`aim.py` predicts that clearance from two offset lines; this measures
the real faces, so the two check each other.

Conservative on both axes: the probe starts at the plate centreline's
tip, not the top-face edge the brick slides off (≈ 0.8 mm further out,
where the slot is wider), and a brick crossing the rim plane has
already moved outward. `aim.LANDING_MARGIN` is the lateral scatter a
falling brick may add beyond the exit width. It is a claim, not a
measurement: `backlog/bench-validation.md`.

Scope: every floor of `FLOOR_LAYOUTS`, each ring at its own floor
height so the four never meet. The ramp itself is not in the fixture —
the probe stands where its exit is.
"""

from build123d import Box, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from .aim import LANDING_MARGIN, aim_offset_deg, ramp_exit_w_for
from .basket import assembly as _bkt
from .params import BASKET_RIM_Z, FLOOR_H, FLOOR_LAYOUTS, basket_name
from .ramp_mechanism.assembly import RAMP_TIP_X

LANDING_DEPTH = 20.0                 # radial reach of the probe past the tip
LANDING_BAND = 20.0                  # vertical: the probe's depth below the rim


def build_basket_landing() -> Assembly:
    """Every floor's basket ring, plus one exit-wide probe per basket,
    at that basket's aim."""
    a = Assembly()
    for floor, layout in sorted(FLOOR_LAYOUTS.items()):
        a = a.with_subassembly(
            f"f{floor}", _bkt.build_basket_ring(layout),
            location=Pos(0, 0, floor * FLOOR_H),
        )
    for floor, layout in sorted(FLOOR_LAYOUTS.items()):
        exit_w = ramp_exit_w_for(layout)
        probe = Box(LANDING_DEPTH, exit_w, LANDING_BAND)
        z = floor * FLOOR_H + BASKET_RIM_Z - LANDING_BAND / 2
        for center_deg, n_size, notches in layout:
            aim_deg = center_deg + aim_offset_deg(n_size, notches, exit_w)
            leaf = basket_name(center_deg, n_size)
            name = f"landing_f{floor}_{leaf}"
            a = (
                a
                .with_part(
                    name, probe,
                    location=Rot(0, 0, aim_deg)
                    * Pos(RAMP_TIP_X + LANDING_DEPTH / 2, 0, z),
                )
                .assert_distance(
                    name, f"f{floor}.{leaf}", min_mm=LANDING_MARGIN, name=name,
                )
            )
    return a


assembly = build_basket_landing()
