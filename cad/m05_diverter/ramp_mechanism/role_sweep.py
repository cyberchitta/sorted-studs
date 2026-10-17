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

from build123d import Compound

from cad_khana.mechanism.check import check
from cad_khana.printability.feature import Feature
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
    exit_opening_cutter,
    hinge_bosses,
    hinge_pin_bore,
    joint_depressions,
    joint_flange,
    make_assembly,
    ramp,
    ring_bearing,
    rotating_collar,
    saddle_cutouts,
    servo_cage_parts,
    spider_hub,
    trap,
    trap_coupler,
    trap_knuckle,
    trap_pin_bore,
    trap_plate,
)


for role in FLOOR_ROLES:
    check(make_assembly(role), out=f"outputs/{role}")

method = FDM(wall_min_mm=WALL_MIN)
# The bearing spider's ceilings, per feature: the saddle jaws drop to
# z = -7 below the hub (z = 0..5, or the base seat's 0..8), so the hub
# underside and the six jaw-channel ceilings face down. Each is waived
# against the solid that makes it (`spider_hub`, `saddle_cutouts`, the
# same ones `bearing_spider` builds from), so a ceiling anywhere else on
# the body traces to no feature and fails. Read 2026-10-15, up +Z: the
# channels 4059.06 mm² in 6 regions (676.51 each) on every role; the
# hub 2362.74 mm² (middle / top) and 13580.02 (base, the seat), one
# region each.
_SPIDER_REASON = (
    "flat 90° ceilings are real: saddle jaws drop to z=-7 below the hub "
    "ring, so the hub underside and jaw-channel ceilings face down — "
    "printed jaws-down with support in the jaw channels"
)
_SPIDER_HUB_MAX_MM2 = {"base": 13581.0, "middle": 2363.5, "top": 2363.5}


def _spider_features(role: str) -> dict[str, Feature]:
    hub_mm2 = _SPIDER_HUB_MAX_MM2[role]
    return {
        "hub": Feature(spider_hub(role), waive={"overhang_max": Waiver(
            reason=f"{_SPIDER_REASON}: the hub underside, one face",
            max_area_mm2=hub_mm2, max_regions=1,
        )}),
        "saddle_channels": Feature(Compound(saddle_cutouts()), waive={"overhang_max": Waiver(
            reason=f"{_SPIDER_REASON}: the 6 channel ceilings, 676.51 mm² each",
            max_area_mm2=4060.0, max_regions=6,
        )}),
    }


# The column's ceilings, per feature, each against the block or cutter
# `column()` builds it from, so a ceiling anywhere else on the tube
# traces to no feature and fails. Printed base-down, vertically: support
# under the annular features and the cage web; the small crowns bridge.
# Read 2026-10-15, up +Z: base 10902.09 mm² in 15 regions, middle
# 11398.99 in 19, top 6298.49 in 18. The top role has no joint flange,
# and its exit-opening lintel runs out through the flare's root (the
# flare's underside is cut at FLARE_UNDER_DEG so it adds nothing).


def _waive(reason: str, max_area_mm2: float, max_regions: int) -> dict:
    return {"overhang_max": Waiver(
        reason=reason, max_area_mm2=max_area_mm2, max_regions=max_regions,
    )}


def _column_features(role: str) -> dict[str, Feature]:
    blocks, cage_cut, tab_holes = servo_cage_parts()
    features = {
        "rotating_collar": Feature(rotating_collar(), waive=_waive(
            "the rotating collar's underside at z 220, cut back by the "
            "exit opening and the Ø9 harness channel — supported. "
            "4239.96 mm², one face (2026-10-17)",
            4240.5, 1,
        )),
        "exit_opening": Feature(exit_opening_cutter(), waive=_waive(
            "the exit opening's lintel at z 267.5, an 80 mm span — "
            "supported. 240.00 mm², one face",
            240.5, 1,
        ) if role != "top" else _waive(
            "the exit opening's lintel at z 267.5, which runs 11.4 mm out "
            "through the flare's root — supported like the rest of it "
            "(owner, 2026-10-10). 774.08 mm², one face",
            774.5, 1,
        )),
        "servo_cage": Feature(Compound(blocks), waive=_waive(
            "the cage web's underside at z 252.3 (470.52 mm², over the "
            "horn) — supported; and the plate and cheek undersides at "
            "z 234.8, level with the case bottom (67.05 + 16.95) — "
            "bridged (owner, 2026-10-15). 554.52 mm², 3 faces",
            555.0, 3,
        )),
        "cage_cutout": Feature(cage_cut, waive=_waive(
            "the case cut-out's lintel at z 247.2, 22.9 mm across the "
            "plate. 68.70 mm², one face",
            69.0, 1,
        )),
        "cage_tab_holes": Feature(Compound(tab_holes), waive=_waive(
            "crowns of the 2 Ø1.5 M2 tab pilots, 3.53 mm² each — bridged "
            "(owner, 2026-10-15). 7.06 mm²",
            7.1, 2,
        )),
        "hinge_bosses": Feature(Compound(hinge_bosses()), waive=_waive(
            "the 2 hinge-boss undersides at z 234.9, 30.00 mm² each — "
            "supported with the cage below",
            60.5, 2,
        )),
        "hinge_pin_bore": Feature(hinge_pin_bore(), waive=_waive(
            "crowns of the Ø3.6 pin bore through both bosses and both "
            "±Y walls, 8.47 mm² each — bridged (owner, 2026-10-15). "
            "33.90 mm², 4 faces",
            34.0, 4,
        )),
    }
    if role != "top":
        features["joint_flange"] = Feature(joint_flange(), waive=_waive(
            "the top joint flange's underside at z 268, less the Ø9 "
            "harness channel — supported. 5571.20 mm², one face "
            "(2026-10-17)",
            5571.5, 1,
        ))
    if role != "base":
        features["joint_depressions"] = Feature(joint_depressions(role), waive=_waive(
            "roofs of the 4 Ø12.6 joint depressions in the bottom "
            "flange, 2.0 deep (z 55.0, 3.0 mm of flange above) over a "
            "1.5 bump — each a 12.6 mm bridge, accepted with ≥0.5 mm "
            "sag allowance over the bump (owner, 2026-10-15; the "
            "allowance is a claim, unverified until printed). 124.23 mm² "
            "each, 496.90 mm²",
            497.5, 4,
        ))
    return features


for role in FLOOR_ROLES:
    printed = [
        ("bearing_spider", bearing_spider(role)),
        ("column", column(role)),
    ]
    if role != "base":
        # The base floor's ring bearing is the bought turntable stub.
        printed.append(("ring_bearing", ring_bearing(role)))
    for name, part in printed:
        features = None
        if name == "bearing_spider":
            features = _spider_features(role)
        elif name == "column":
            features = _column_features(role)
        inspect(
            part, method=method, out=f"outputs/{role}", name=name,
            features=features,
        )
# Trap and ramp are role-independent — inspect once at the top.
# The trap's ceilings, per feature, each against the solid `trap()`
# builds it from: KNUCKLE_OD (8) exceeds TRAP_T (4) and the plate is
# centred on the hinge axis, so the whole plate underside floats 2 mm
# above the knuckle's lowest point, the knuckle's lower half faces down
# beside it, and the D-bore's flat roofs the bore. Read 2026-10-15 on the
# stepped plate (unchanged from 2026-10-10): 7964.01 mm² in 3 regions.
inspect(
    trap(), method=method, out="outputs", name="trap", solid_count=1,
    features={
        "plate": Feature(trap_plate(), waive={"overhang_max": Waiver(
            reason="real ceiling: the plate underside, 2 mm above the "
            "knuckle's lowest point — printed with support under the "
            "plate. 7269.15 mm², one face",
            max_area_mm2=7270.0, max_regions=1,
        )}),
        "knuckle": Feature(trap_knuckle(), waive={"overhang_max": Waiver(
            reason="the knuckle's underside below the plate, supported "
            "with it. 502.19 mm², one face",
            max_area_mm2=503.0, max_regions=1,
        )}),
        "pin_bore": Feature(trap_pin_bore(), waive={"overhang_max": Waiver(
            reason="the D-bore's flat, the bore's roof: bridged. "
            "192.67 mm², one face",
            max_area_mm2=193.0, max_regions=1,
        )}),
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
