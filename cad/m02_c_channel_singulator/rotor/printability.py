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
# cavity inner face matching inward lean). The cavity ceiling, boss
# bottom, and blind-bore top will still flag as 90° overhangs — see
# waiver below.
method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=50.0)
inspect(
    rotor(), method=method, out="outputs", name="rotor",
    waive={
        "wall_min": "wedge tip at the base rim, not a wall — min_wall_at "
        "witness (40.6, 111.5, 0.0) r=118.7 z=0 with alignment 0.72 ≈ "
        "cos(V_HALF_ANGLE_DEG=45°), the dihedral between the flat base "
        "annulus (z=0) and the outer flank that slopes up-and-inward "
        "from the true-round r=120 rim. Rays leave the base face and "
        "exit the flank after (120 − r), so the reading is set by how "
        "close a base facet centroid gets to the rim: all 20 minima are "
        "the identical 1.2937 at z=0, one per star sub-facet, at θ = "
        "±2° either side of each valley azimuth. Not the star-ridge "
        "crease the pre-0.8 metric pointed at (that 0.0900 witness at "
        "z=21 was the chord-sag artifact). The shell wall itself is "
        "≥ 3.887 mm — worst of 11 448 radial probes over "
        "z ∈ [0.25, 79.25] × 72 off-symmetry azimuths — vs "
        "ROTOR_SHELL_WALL_T = 2.5. The feather rim is the V-trough "
        "geometry (the rotor flank and the stator inner face both come "
        "to a point at the V-bottom) and rides the apex-up print plan "
        "in the module docstring.",
        "overhang_max": "real 90° ceilings: cavity ceiling, boss "
        "bottom, and blind-bore top all face down printed apex-up — "
        "print plan in the module docstring (breakaway support through "
        "the open base, or split at z=BOSS_BOTTOM_Z)",
    },
)
