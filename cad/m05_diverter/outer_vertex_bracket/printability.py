"""Printability batch for the outer vertex bracket — run, don't import.

    .venv/bin/khana run cad/m05_diverter/outer_vertex_bracket/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../outer_vertex_bracket/assembly.py`);
this script runs the per-part `inspect()` pass.

One body covers every placement: `bracket()` with defaults at all 24
floor outer vertices (4 floors × 6) and the two `top_cap` brackets
under the deck, rotated and translated only. Checked 2026-09-27
against the m05 placed tree — every `*.outer_vertex_bracket_k` has
this body's volume, area, face count and local bbox.
"""

from build123d import Compound

from cad_khana.printability.feature import Feature
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM
from cad_khana.printability.waiver import Waiver

from .assembly import WALL_MIN, bracket, clamp_bolt_holes


# Prints as installed, +Z up: the hub + three arms' bottom face
# (z = -26, 3111 mm²) is the bed and the clamp arm rises from it.
# Every other candidate (±X, ±Y, -Z, each large face's normal) leaves
# 935–3040 mm² facing down, against 49 mm² here. The residue is the
# crown of the clamp arm's two horizontal M5 holes (Ø5.5 along X at
# z = 10 and 30); nothing else faces down above the bed.
# Waived against the two hole cutters `bracket()` subtracts
# (`clamp_bolt_holes`), so a ceiling anywhere else on the body traces
# to no feature and fails.
inspect(
    bracket(),
    method=FDM(wall_min_mm=WALL_MIN, up_axis=(0, 0, 1)),
    out="outputs",
    name="bracket",
    features={"clamp_bolt_holes": Feature(
        Compound(list(clamp_bolt_holes())),
        waive={"overhang_max": Waiver(
            reason="crowns of the clamp arm's two Ø5.5 M5 clearance "
            "holes (horizontal, along X at z = 10 and 30) — "
            "25.89 mm² each, 51.79 mm² past 45°, nothing else "
            "(read 2026-10-10 and again 2026-10-15; the exact quarter-arc crown of a "
            "Ø5.5 × 6 bore is 25.92); "
            "print plan: bridge the 5.5 mm crowns unsupported "
            "(clearance holes tolerate the sag; chase with a "
            "5.5 drill if a bolt binds), no support material",
            max_area_mm2=52.3, max_regions=2,
        )},
    )},
)
