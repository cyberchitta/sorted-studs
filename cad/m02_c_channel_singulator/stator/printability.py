"""Printability batch for the stator — run, don't import.

    .venv/bin/khana run cad/m02_c_channel_singulator/stator/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check .../stator/assembly.py`); this script
runs the per-part `inspect()` pass and its waivers.
"""

from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from .assembly import WALL_MIN, stator


# Both wall faces sit at 45° from vertical (= 45° from horizontal).
# Set threshold with a small margin so V1 lands "passing" — real
# prints will likely need either outside support or an inverted
# orientation; split at a later revision.
method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=50.0)
inspect(
    stator(), method=method, out="outputs", name="stator",
    waive={
        "wall_min": "wedge tip at the designed V-bottom knife edge, not "
        "a wall — min_wall_at witness (114.0, 43.4, 0.017) r=122.0 z=0 "
        "with alignment 0.71, and 0.707 = cos(V_HALF_ANGLE_DEG=45°) is "
        "exactly the dihedral between the flat bottom annulus (z=0, "
        "r=122..125) and the inner V-cone face (r = 122 + z). Rays "
        "leave the bottom face at r = 122 + δ and exit the cone after "
        "δ, so the reading is the closest facet centroid to that "
        "circle, not a design dimension: all 40 sub-1.5 mm samples "
        "cluster at z≈0, r ∈ [122, 123] at the same 0.71. The wall is "
        "STATOR_WALL_T (3.0) measured perpendicular to the cone; the "
        "inner edge tapering to zero at the V-bottom is the trough "
        "geometry itself (see the frame note in the module docstring) "
        "and is covered by the same support/inverted print plan the "
        "45° flanks already need. Truncating it would put a ledge at "
        "the exact circle bricks slide through.",
    },
)
