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
adhesive dabs.

The ceilings are waived per feature, each against the cutter or
block that made it (the same solids `frame()` builds from), so a new
ceiling anywhere else on the body traces to no feature and fails.
Measured 2026-09-27, up cartridge −Z, all at 90°, 550.35 mm² over 10
regions. The two in-pocket bracket tops lie on both the seat plane
and the bracket tops, so they count toward both `seat` and
`brackets`.
"""

from build123d import Compound

from cad_khana.printability.feature import Feature
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM
from cad_khana.printability.waiver import Waiver

from ..params import WALL_MIN
from .assembly import bracket_blocks, frame, pin_bore_cutters, seat_cutter


inspect(
    frame(),
    method=FDM(up_axis=(0, 0, -1), wall_min_mm=WALL_MIN),
    out="outputs",
    name="frame",
    solid_count=1,
    features={
        "seat": Feature(seat_cutter(), waive={"overhang_max": Waiver(reason=(
            "the acrylic seat, by owner decision 2026-09-27: 430.0 mm² — 4 "
            "corner-lip tops, 100 each, and 2 in-pocket bracket tops at "
            "(±13, −53.5, −4), 15 each. Bridged, no supports; sanded or "
            "cleaned flat after printing (it takes the adhesive dabs)."
        ), max_area_mm2=434.0, max_regions=6)}),
        "brackets": Feature(Compound(list(bracket_blocks())), waive={"overhang_max": Waiver(reason=(
            "bracket tops, 110.0 mm²: 2 outboard tops past the frame edge, "
            "40 each, non-functional, and the 2 in-pocket seat ends, 15 "
            "each (also `seat`'s). Bridged, no supports."
        ), max_area_mm2=111.0, max_regions=4)}),
        "pin_bores": Feature(Compound(list(pin_bore_cutters())), waive={"overhang_max": Waiver(reason=(
            "2 Ø5.4 pin-bore crowns, 20.18 each, 40.35 mm²: bridged, then "
            "drilled Ø5.4 through."
        ), max_area_mm2=40.8, max_regions=2)}),
    },
)
