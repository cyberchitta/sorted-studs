"""Printability batch for the rotor — run, don't import.

    .venv/bin/khana run cad/m02_c_channel_singulator/rotor/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../rotor/assembly.py`); this script
runs the per-part `inspect()` pass and its waivers.
"""

from cad_khana.printability.feature import Feature
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM
from cad_khana.printability.waiver import Waiver

from .assembly import WALL_MIN, rotor, rotor_cavity


# 50° threshold passes the 45° conical flank (outer face base-down,
# cavity inner face matching inward lean). The cavity ceiling still
# flags as a 90° overhang — see waiver below. The pre-flange rim
# knife-edge waiver (wall_min 1.29 at the z=0 base annulus) retired
# 2026-09-11: the flange's flat underside is now the base face and the
# thinnest real wall reads 3.46 mm at the shell (alignment 0.98).
#
# The ceiling is waived on the cavity cutter that makes it, so a new
# down-facing face anywhere else on the rotor traces to no feature and
# fails. Measured 2026-10-15: 3072.15 mm², one region at (0, 0, 80).
method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=50.0)
inspect(
    rotor(), method=method, out="outputs", name="rotor",
    features={
        "cavity": Feature(rotor_cavity(), waive={"overhang_max": Waiver(
            reason="real 90° ceiling: the cavity ceiling (top-cap "
            "underside), 3072.15 mm² in one region at z 80, faces down "
            "printed base-down — print plan in the module docstring "
            "(breakaway support through the open base, which is why the "
            "base stays open)",
            max_area_mm2=3075.0, max_regions=1,
        )}),
    },
)
