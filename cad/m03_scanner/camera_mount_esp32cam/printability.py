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

All four placed bodies are inspected, each with the `mount_in_clamp`
the rig gives it — one function, four different bodies. Until
2026-09-22 only the default (the two `below` poses) was, and the other
two carry failures of their own (see the waiver note below).
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from ..assembly import build_m03
from ..params import WALL_MIN


# Print orientation: +Y up (clamp base on the build plate). FDM
# default up_axis is +Z, so override.
method = FDM(up_axis=(0, 1, 0), wall_min_mm=WALL_MIN)

# Facet audit 2026-07-25: the rev-4 "near-90° degenerate sliver"
# this site's threshold was loosened for no longer exists —
# nothing above 56° is flagged. What remains is real geometry.
# Re-measured 2026-09-22 per pose: the two `below` bodies read
# exactly this (174 mm², max 55.4°). `front_low` (184 mm², max 65°)
# and `back_upper` (1579 mm², max 45.5°) are different overhangs the
# audit never saw, so the waiver is NOT extended to them — a waiver
# is per kind and would silently cover them.
_BELOW_WAIVER = {
    "overhang_max": "real overhangs, accepted: oblique-beam "
    "underside facets at 55.4° (~153 mm²) plus 48-54° bore/access-"
    "cylinder ceiling bands (~21 mm²) — steep-but-printable with "
    "cooling or minimal support; no near-90° sliver remains",
}

for pp in build_m03().placed_parts:
    if not pp.name.endswith("_clamp_mount"):
        continue
    leaf = pp.name.rsplit(".", 1)[-1]
    inspect(
        pp.part, method=method, out="outputs", name=leaf,
        waive=_BELOW_WAIVER if "_below_" in leaf else None,
    )
