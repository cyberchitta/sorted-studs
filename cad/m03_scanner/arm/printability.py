"""Printability batch for the turret arm spider — run, don't import.

    .venv/bin/khana run cad/m03_scanner/arm/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../arm/assembly.py`); this script
runs the per-part `inspect()` pass.

The spider comes out of `build_turret_arm()` by name, so the body
inspected is the one the turret places (`turret.rotor.arm.spider`);
`PlacedPart.part` is the body in spider-local coordinates (origin on
the hub axis at the pin-axis plane, disc spanning z ±12.5).
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..params import WALL_MIN
from .assembly import build_turret_arm


# Print orientation: flat, the disc's bottom face (4418 mm²) on the
# bed, +Z up — swept 2026-09-27 against ±X/±Y/±Z. On its side (±X,
# ±Y) the bed is one 500 mm² block end and ~3680 mm² faces down; flat
# leaves only the horizontal bores. The body is mirror-symmetric in
# Z, so −Z is the same print (388.9 vs 389.4 mm², tessellation noise).
# The Ø8.4 shaft bore prints vertical. Every orientation prints at
# least two of the four pin bores sideways, so that is the part's,
# not this choice's.
inspect(
    build_turret_arm().part("spider").part,
    method=FDM(wall_min_mm=WALL_MIN, up_axis=(0, 0, 1)),
    out="outputs",
    name="spider",
    solid_count=1,
    waive={
        "overhang_max": "389.35 mm² at up to 90.0° (measured 2026-09-27, "
            "up +Z): the ceilings of the five horizontal bores and "
            "nothing else — the four Ø5.4 hinge-pin bores through the "
            "bushing blocks at r 62.3–67.7, 88.78 mm² each, and the Ø4 "
            "set-screw insert bore at heading +45°, 34.23 mm². Print "
            "plan: bridged, no supports; run a Ø5.4 drill through each "
            "pin bore after printing to restore the 0.2 mm radial pin "
            "clearance the bridge droop eats.",
    },
)
