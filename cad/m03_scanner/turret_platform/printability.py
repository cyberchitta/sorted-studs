"""Printability batch for the turret platform frame — run, don't import.

    .venv/bin/khana run cad/m03_scanner/turret_platform/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../turret_platform/assembly.py`);
this script runs the per-part `inspect()` pass. The four
`rotor.platform_<station>.frame` placements are one body from one
no-argument builder, so one inspect covers all four.

Print orientation: top ring down (cartridge −Z up). Bed contact is
the ring top plus the kicker stem's gusset, 3218 mm². Every ceiling
left facing the bed is bridged without supports. That includes the
acrylic's seat faces, by owner decision 2026-09-27: each corner lip
is a short 10 × 10 bridge held on two edges, and the seats are
sanded or cleaned flat after printing because they take the
adhesive dabs. The waiver below lists each feature and its area.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..params import WALL_MIN
from .assembly import frame


inspect(
    frame(),
    method=FDM(up_axis=(0, 0, -1), wall_min_mm=WALL_MIN),
    out="outputs",
    name="frame",
    solid_count=1,
    waive={
        "overhang_max": (
            "measured 2026-09-27, up cartridge −Z: 550.35 mm² at 90°, all "
            "flat ceilings. Seat faces, included by owner decision "
            "2026-09-27, 430.0 mm²: 4 corner-lip tops, 100 each, and 2 "
            "in-pocket bracket tops at (±13, −53.5, −4), 15 each. "
            "Non-functional, 120.36 mm²: 2 outboard bracket tops, 40 "
            "each, and 2 Ø5.4 pin-bore ceilings, 20.18 each. Print plan: "
            "every ceiling bridged, no supports; sand or clean the seat "
            "faces flat after printing; drill Ø5.4 through the pin bores. "
            "This waiver is kind-wide, so it covers any overhang on this "
            "body. The total area is the drift tell: if it is not "
            "550.35 mm², re-measure."
        ),
    },
)
