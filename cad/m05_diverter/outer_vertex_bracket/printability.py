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

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM
from cad_khana.printability.waiver import Waiver

from .assembly import WALL_MIN, bracket


# Prints as installed, +Z up: the hub + three arms' bottom face
# (z = -26, 3111 mm²) is the bed and the clamp arm rises from it.
# Every other candidate (±X, ±Y, -Z, each large face's normal) leaves
# 935–3040 mm² facing down, against 49 mm² here. The residue is the
# crown of the clamp arm's two horizontal M5 holes (Ø5.5 along X at
# z = 10 and 30); nothing else faces down above the bed.
inspect(
    bracket(),
    method=FDM(wall_min_mm=WALL_MIN, up_axis=(0, 0, 1)),
    out="outputs",
    name="bracket",
    waive={
        "overhang_max": Waiver(
            reason="crowns of the clamp arm's two Ø5.5 M5 clearance "
            "holes (horizontal, along X at z = 10 and 30) — "
            "49.32 mm² past 45°, max 85.71°, nothing else; "
            "print plan: bridge the 5.5 mm crowns unsupported "
            "(clearance holes tolerate the sag; chase with a "
            "5.5 drill if a bolt binds), no support material",
            max_area_mm2=50.0, max_regions=2,
        ),
    },
)
