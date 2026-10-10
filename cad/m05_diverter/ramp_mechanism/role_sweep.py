"""Three-role check + printability suite for the ramp mechanism —
run, don't import.

    .venv/bin/khana run cad/m05_diverter/ramp_mechanism/role_sweep.py

`assembly.py` declares one floor's geometry and claims; `make_assembly`
is parameterized by floor role, so the unit's real check surface is a
*family* of three members (base / middle / top), not one. The CLI
addresses a single member with defaults, and `make_assembly` takes a
required `role`, so the family is orchestrated here instead:

  * `khana check .../ramp_mechanism/assembly.py` evaluates the
    degenerate `assembly` attribute — the **middle** role only.
  * this script evaluates all three roles into `outputs/<role>/`, and
    runs the role×part printability pass beside each.

Everything under `khana run` gets the CLI's failure deferral, so all
three roles and every inspect run and every JSON on disk is
current before the roll-up exits nonzero.
"""

from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM
from cad_khana.printability.waiver import Waiver

from .assembly import (
    FLOOR_ROLES,
    RAMP_EXIT_WIDTHS,
    RAMP_W,
    WALL_MIN,
    bearing_spider,
    column,
    make_assembly,
    ramp,
    ring_bearing,
    trap,
    trap_coupler,
)


for role in FLOOR_ROLES:
    check(make_assembly(role), out=f"outputs/{role}")

method = FDM(wall_min_mm=WALL_MIN)
waivers = {
    "bearing_spider": {
        "overhang_max": "flat 90° ceilings are real: saddle jaws drop "
        "to z=-7 below the hub ring (z=0..5), so the hub underside and "
        "jaw-channel ceilings face down — printed jaws-down with "
        "support in the jaw channels",
    },
    "column": {
        "overhang_max": "flat 90° ceilings are real: collar / joint-"
        "flange / hinge-boss undersides, the exit-opening lintel, and "
        "the servo cage's web underside and cut-out lintel on a "
        "vertically printed tube — printed base-down with support "
        "under the annular features and the cage web",
    },
}
# The top column is a different body since the flare (2026-10-10): no
# joint flange, and a funnel on top. The waiver above was written
# against a flanged column, so here it is bound to what the top column
# reads: the pre-flare 11400.94 mm² in 19 regions, less the flange
# underside (5634.58 mm², one region), plus the lintel's growth. The
# opening cut runs out through the flare's root, so the lintel ceiling
# is 774.08 mm² (11.4 mm past the wall) where it was 240, and the
# reading is 6300.44 mm². The flare's underside is cut at
# FLARE_UNDER_DEG so that it adds nothing. The deeper lintel is accepted
# as the same supported ceiling (owner, 2026-10-10); the bound is the
# reading, so any further growth is red.
top_column_waivers = {
    "overhang_max": Waiver(
        reason=waivers["column"]["overhang_max"]
        + " (top role: no joint flange; the exit-opening lintel runs "
        "11.4 mm out through the flare's root, supported like the rest "
        "of it)",
        max_area_mm2=6305.0, max_regions=18,
    ),
}
for role in FLOOR_ROLES:
    printed = [
        ("bearing_spider", bearing_spider(role)),
        ("column", column(role)),
    ]
    if role != "base":
        # The base floor's ring bearing is the bought turntable stub.
        printed.append(("ring_bearing", ring_bearing(role)))
    for name, part in printed:
        waive = waivers.get(name)
        if (role, name) == ("top", "column"):
            waive = top_column_waivers
        inspect(
            part, method=method, out=f"outputs/{role}", name=name,
            waive=waive,
        )
# Trap and ramp are role-independent — inspect once at the top.
inspect(
    trap(), method=method, out="outputs", name="trap", solid_count=1,
    waive={
        "overhang_max": Waiver(
            reason="real ceilings: KNUCKLE_OD (8) exceeds TRAP_T "
            "(4), and the plate is centred on the hinge axis, so the whole "
            "plate underside floats 2 mm above the knuckle's lowest point "
            "— printed with support under the plate; pin-bore top bridges. "
            "Read 2026-10-04 on the stepped plate: 7940 mm² in 3 regions "
            "(plate underside 7269, knuckle 478, bore flat 193)",
            max_area_mm2=7950.0, max_regions=3,
        ),
    },
)
inspect(ramp(), method=method, out="outputs", name="ramp")
# The narrow variants — one printed ramp per exit width.
for exit_w in RAMP_EXIT_WIDTHS:
    if exit_w < RAMP_W:
        inspect(
            ramp(exit_w), method=method, out="outputs",
            name=f"ramp_exit_{exit_w:.0f}", solid_count=1,
        )
inspect(trap_coupler(), method=method, out="outputs", name="trap_coupler")
