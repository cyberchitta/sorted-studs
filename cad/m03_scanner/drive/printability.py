"""Printability batch for the turret drive bracket — run, don't import.

    .venv/bin/khana run cad/m03_scanner/drive/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../drive/assembly.py`); this script
runs the per-part `inspect()` pass.

The bracket comes out of `build_turret_drive_static()` by name, so the
body inspected is the one the turret places (`turret.drive.drive_bracket`);
`PlacedPart.part` is that body in turret-local coordinates (the bracket
is placed at identity).
"""

from build123d import Compound

from cad_khana.printability.feature import Feature
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM
from cad_khana.printability.waiver import Waiver

from ..params import WALL_MIN
from .assembly import (
    boss_pilot_cutter,
    build_turret_drive_static,
    foot_bolt_cutters,
    motor_bolt_cutters,
)


# Print orientation: on its side, turret +Y up, the flat L-profile face
# at y = −25 (1635.6 mm²) on the bed — owner decision 2026-09-27, from a
# sweep of ±X/±Y/±Z. The body is mirror-symmetric about y = 0, so −Y is
# the same print; +Y is named so the bed face is the operator-side one.
# Upright (+Z) leaves 2075.44 mm² under the plate the motor flange seats
# on, upside down (−Z) 2057.66 mm² under the foot's bolt-head seat, and
# ±X 2881 mm². On its side every layer holds the whole L, so the motor's
# cantilever moment runs along the layers.
#
# The ceilings are those of the seven holes that print sideways and
# nothing else, waived per feature against the cutters that made them
# (the same solids `bracket()` cuts with), so a new ceiling anywhere
# else on the body traces to no feature and fails. Measured 2026-10-15,
# up turret +Y: 295.03 mm² over 7 regions. Every ceiling is bridged,
# no supports.
inspect(
    build_turret_drive_static().part("drive_bracket").part,
    method=FDM(wall_min_mm=WALL_MIN, up_axis=(0, 1, 0)),
    out="outputs",
    name="drive_bracket",
    features={
        "boss_pilot": Feature(boss_pilot_cutter(), waive={"overhang_max": Waiver(
            reason="the Ø22 motor centering pilot's ceiling in the plate, "
            "138.10 mm² (read 2026-10-15, up turret +Y). Bridged; ream "
            "the pilot to fit the motor boss.",
            max_area_mm2=138.3, max_regions=1,
        )}),
        "foot_bolt_holes": Feature(Compound(list(foot_bolt_cutters())), waive={"overhang_max": Waiver(
            reason="the two Ø5.5 M5 foot-bolt hole ceilings at x 106.1 "
            "and 124.1, 34.52 mm² each, 69.05 mm² (read 2026-10-15, up "
            "turret +Y). Bridged; drill Ø5.5 if they bind.",
            max_area_mm2=69.2, max_regions=2,
        )}),
        "motor_bolt_holes": Feature(Compound(list(motor_bolt_cutters())), waive={"overhang_max": Waiver(
            reason="the four Ø3.5 M3 motor hole ceilings at (x 43.1 / "
            "74.1, y ±15.5), 21.97 mm² each, 87.88 mm² (read 2026-10-15, "
            "up turret +Y). Bridged; drill if they bind.",
            max_area_mm2=88.1, max_regions=4,
        )}),
    },
)
