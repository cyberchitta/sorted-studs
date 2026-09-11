"""Printability batch for the rotor — run, don't import.

    .venv/bin/khana run cad/m02_c_channel_singulator/rotor/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../rotor/assembly.py`); this script
runs the per-part `inspect()` pass and its waivers.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import WALL_MIN, rotor


# 50° threshold passes the 45° conical flank (outer face base-down,
# cavity inner face matching inward lean). The cavity ceiling still
# flags as a 90° overhang — see waiver below. The pre-flange rim
# knife-edge waiver (wall_min 1.29 at the z=0 base annulus) retired
# 2026-09-11: the flange's flat underside is now the base face and the
# thinnest real wall reads 3.46 mm at the shell (alignment 0.98).
method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=50.0)
inspect(
    rotor(), method=method, out="outputs", name="rotor",
    waive={
        "overhang_max": "real 90° ceiling: the cavity ceiling (top-cap "
        "underside) faces down printed base-down — print plan in the "
        "module docstring (breakaway support through the open base, "
        "which is why the base stays open)",
    },
)
