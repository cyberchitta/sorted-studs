"""Printability batch for the inner vertex bracket — run, don't import.

    .venv/bin/khana run cad/m05_diverter/inner_vertex_bracket/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../inner_vertex_bracket/assembly.py`);
this script runs the per-part `inspect()` pass.

One body covers every placement: the frame places `bracket()` with
defaults at all 24 inner vertices (4 floors × 6), rotated and
translated only. Checked 2026-09-27 against the m05 placed tree —
every `*.inner_vertex_bracket_k` has this body's volume, area, face
count and local bbox.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import WALL_MIN, bracket


# Prints as installed, +Z up: the hub + four arms' bottom face
# (z = -26, 3622 mm²) is the bed and the clamp arm rises from it.
# Every other candidate (±X, ±Y, -Z, each large face's normal) leaves
# 1185–3556 mm² facing down, against 54 mm² here. The residue is the
# crown of the clamp arm's two horizontal M5 holes (Ø5.5 along Y at
# z = 10 and 30); nothing else faces down above the bed.
inspect(
    bracket(),
    method=FDM(wall_min_mm=WALL_MIN, up_axis=(0, 0, 1)),
    out="outputs",
    name="bracket",
    waive={
        "overhang_max": "crowns of the clamp arm's two Ø5.5 M5 clearance "
                        "holes (horizontal, along Y at z = 10 and 30) — "
                        "54.25 mm² past 45°, max 90.0°, nothing else; "
                        "print plan: bridge the 5.5 mm crowns unsupported "
                        "(clearance holes tolerate the sag; chase with a "
                        "5.5 drill if a bolt binds), no support material",
    },
)
