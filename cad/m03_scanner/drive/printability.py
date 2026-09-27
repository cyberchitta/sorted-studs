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

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..params import WALL_MIN
from .assembly import build_turret_drive_static


# Print orientation: on its side, turret +Y up, the flat L-profile face
# at y = −25 (1635.6 mm²) on the bed — owner decision 2026-09-27, from a
# sweep of ±X/±Y/±Z. The body is mirror-symmetric about y = 0, so −Y is
# the same print; +Y is named so the bed face is the operator-side one.
# Upright (+Z) leaves 2075.44 mm² under the plate the motor flange seats
# on, upside down (−Z) 2057.66 mm² under the foot's bolt-head seat, and
# ±X 2881 mm². On its side every layer holds the whole L, so the motor's
# cantilever moment runs along the layers.
inspect(
    build_turret_drive_static().part("drive_bracket").part,
    method=FDM(wall_min_mm=WALL_MIN, up_axis=(0, 1, 0)),
    out="outputs",
    name="drive_bracket",
    waive={
        "overhang_max": "309.08 mm² at up to 90.0° (measured 2026-09-27, "
            "up turret +Y): the ceilings of the seven holes that print "
            "sideways and nothing else — the Ø22 motor centering pilot "
            "in the plate, 144.68 mm²; the two Ø5.5 M5 foot-bolt holes "
            "at x 106.1 and 124.1, 36.17 mm² each; the four Ø3.5 M3 "
            "motor holes at (x 43.1 / 74.1, y ±15.5), 23.02 mm² each. "
            "Print plan: every ceiling bridged, no supports; ream the "
            "Ø22 pilot to fit the motor boss; drill the Ø5.5 and M3 "
            "holes if they bind.",
    },
)
