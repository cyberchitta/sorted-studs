"""Printability batch for the ESP32-CAM clamp mount — run, don't import.

    .venv/bin/khana run cad/m03_scanner/camera_mount_esp32cam/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../camera_mount_esp32cam/assembly.py`);
this script runs the per-part `inspect()` pass and its waivers.

**This script is red on purpose.** The `wall_min` failure is a real
0.70 mm wall at the M4 clamp bolt (alignment 0.82 — two near-parallel
faces genuinely that close), not a measurement artifact, and it is
left unwaived until the design slice that fixes it lands. See
`_notes/state/m03-scanner.md` Open work.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..params import WALL_MIN
from .assembly import clamp_mount


# Print orientation: +Y up (clamp base on the build plate). FDM
# default up_axis is +Z, so override.
inspect(
    clamp_mount(),
    method=FDM(up_axis=(0, 1, 0), wall_min_mm=WALL_MIN),
    out="outputs",
    name="clamp_mount",
    # Facet audit 2026-07-25: the rev-4 "near-90° degenerate sliver"
    # this site's threshold was loosened for no longer exists —
    # nothing above 56° is flagged. What remains is real geometry.
    waive={
        "overhang_max": "real overhangs, accepted: oblique-beam "
        "underside facets at 55.4° (~153 mm²) plus 48-54° bore/access-"
        "cylinder ceiling bands (~21 mm²) — steep-but-printable with "
        "cooling or minimal support; no near-90° sliver remains",
    },
)
