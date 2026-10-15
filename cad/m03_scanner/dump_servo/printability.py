"""Printability batch for the dump servo bracket + arm — run, don't import.

    .venv/bin/khana run cad/m03_scanner/dump_servo/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../dump_servo/assembly.py`); this
script runs the per-part `inspect()` pass.

The arm comes out of `build_turret_dump_servo_arm()` by name, so the
body inspected is the one the turret places under its servo joint
(`turret.servo_arm.servo_arm`); `PlacedPart.part` is that body at its
rest pose in turret-local coordinates.
"""

from cad_khana.printability.feature import Feature
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM
from cad_khana.printability.waiver import Waiver

from ..params import WALL_MIN
from .assembly import (
    build_turret_dump_servo_arm,
    hub_disc,
    servo_bracket,
    shaft_bore_cutter,
)


# servo_bracket has no downward face above the bed (overhang null,
# 2026-09-22), so it takes the default threshold.
inspect(
    servo_bracket(),
    method=FDM(wall_min_mm=WALL_MIN),
    out="outputs",
    name="servo_bracket",
)

# servo_arm prints pad-down: the pad's top face (the lifter contact,
# 22 × 26 at z −45) on the bed, turret −Z up — swept 2026-09-27
# against ±X/±Y/±Z and both beam-face normals. The 46.1° beam's
# underside then stands at 43.9° and passes; +Z leaves no flat bed
# face, ±Y hangs the 8 mm-wide beam's side face 9 mm over the bed
# (556 mm² at 90°), ±X 614–716 mm². "Beam flat on the bed" (the
# module docstring's old plan) has no bed face at all: the Ø16 hub
# and the pad both stand proud of the 6 mm beam.
#
# The two ceilings are waived per feature, each against the solid that
# made it (the same solids `servo_arm()` builds from), so a new ceiling
# anywhere else on the body traces to no feature and fails. Measured
# 2026-10-15, up turret −Z: 102.55 mm² over 2 regions.
inspect(
    build_turret_dump_servo_arm().part("servo_arm").part,
    method=FDM(wall_min_mm=WALL_MIN, up_axis=(0, 0, -1)),
    out="outputs",
    name="servo_arm",
    features={
        "hub_disc": Feature(hub_disc(), waive={"overhang_max": Waiver(
            reason="the Ø16 hub disc's outboard arc (x −86…−77, the part "
            "the beam does not meet), 74.30 mm² to 88.8° (read "
            "2026-10-15, up turret −Z). Print plan: tree support from the "
            "bed under the hub arc only (a cosmetic face).",
            max_area_mm2=74.6, max_regions=1,
        )}),
        "shaft_bore": Feature(shaft_bore_cutter(), waive={"overhang_max": Waiver(
            reason="the Ø4.5 servo-shaft bore's ceiling, 28.25 mm² to "
            "89.3° (read 2026-10-15, up turret −Z). Print plan: the bore "
            "bridges, then a Ø4.5 drill through it.",
            max_area_mm2=28.4, max_regions=1,
        )}),
    },
)
