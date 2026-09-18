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
three roles and all eleven inspects run and every JSON on disk is
current before the roll-up exits nonzero.
"""

from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import (
    FLOOR_ROLES,
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
for role in FLOOR_ROLES:
    printed = [
        ("bearing_spider", bearing_spider(role)),
        ("column", column(role)),
    ]
    if role != "base":
        # The base floor's ring bearing is the bought turntable stub.
        printed.append(("ring_bearing", ring_bearing(role)))
    for name, part in printed:
        inspect(
            part, method=method, out=f"outputs/{role}", name=name,
            waive=waivers.get(name),
        )
# Trap and ramp are role-independent — inspect once at the top.
inspect(
    trap(), method=method, out="outputs", name="trap",
    waive={
        "overhang_max": "real ceilings: KNUCKLE_OD (10) exceeds TRAP_T "
        "(4), so with the knuckle tangent on the plate the whole plate "
        "underside floats 3 mm up — printed with support under the "
        "plate; pin-bore top bridges",
    },
)
inspect(ramp(), method=method, out="outputs", name="ramp")
inspect(trap_coupler(), method=method, out="outputs", name="trap_coupler")
