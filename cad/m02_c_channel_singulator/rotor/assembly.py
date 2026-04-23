"""
M02 C-channel singulator — rotor sub-assembly.

Faceted frustum coaxial with the stator V-trough. Bricks drop onto
the flat top, slide down the outward-tilted flanks, and settle into
the V between the rotor's base edge and the stator's outer wall. As
the rotor spins, each facet drags bricks against the outer stator
wall; transitions between facets create a "wave" that helps break
up clumps (see docs/m02-c-channel-singulator.md).

Outside-ride geometry: bricks ride on the OUTSIDE of the rotor
flank, pressed against the outer stator wall — not through a
centre drop hole. The rotor top is a closed platform; the only
axial opening is the drive-shaft bore.

V1 is a solid frustum with a central shaft bore; hub reinforcement
and D-shaft retention are deferred until the drive sub-assembly
lands.

Standalone validation: rotor alone, printability check (FDM).
Cross-boundary checks (rotor-vs-stator gap, shaft coaxiality) live
in the top-level ../assembly.py once sibling sub-assemblies exist.

Coordinate frame:
    origin = rotor axis ∩ base plane (widest section, at the
             V-bottom level of the stator)
    +Z     = up (top-platform / drop-surface direction)
    z=0    = rotor base plane (coincident with stator V-bottom)
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build123d import Cylinder, Part, Pos, RegularPolygon, extrude

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from params import (
    ROTOR_BASE_DIAMETER,
    ROTOR_FACET_COUNT,
    ROTOR_HEIGHT,
    SHAFT_BORE_CLEARANCE,
    SHAFT_DIAMETER,
    V_HALF_ANGLE_DEG,
)


WALL_MIN = 1.5


# --- Part functions ----------------------------------------------------

def rotor(
    *,
    base_diameter: float = ROTOR_BASE_DIAMETER,
    height: float = ROTOR_HEIGHT,
    n_facets: int = ROTOR_FACET_COUNT,
    flank_angle_deg: float = V_HALF_ANGLE_DEG,
    shaft_bore_d: float = SHAFT_DIAMETER + 2 * SHAFT_BORE_CLEARANCE,
) -> Part:
    """Faceted frustum centered on the Z axis, base at z=0, apex up.

    Closed-top regular n-gonal frustum with a central shaft bore.
    `base_diameter` is the circumscribing-circle diameter of the base
    polygon; facet corners lie on that circle and facet flats sit
    inside it by a factor of cos(π/n_facets).

    The flank narrows as it rises at `flank_angle_deg` from vertical,
    giving a top face diameter of
    `base_diameter − 2·height·tan(flank_angle_deg)`.
    """
    base_sketch = RegularPolygon(
        radius=base_diameter / 2, side_count=n_facets,
    )
    cone = extrude(base_sketch, amount=height, taper=flank_angle_deg)
    # Through-bore with 1 mm poke-through each end for a clean boolean.
    bore = Pos(0, 0, height / 2) * Cylinder(
        shaft_bore_d / 2, height + 2,
    )
    return cone - bore


# --- Assembly ----------------------------------------------------------

assembly = Assembly().add("rotor", rotor())


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print base-down: the flank's 45°-from-vertical slope is on the
    # friendly side (each layer smaller than the one below), so a 50°
    # overhang threshold gives a small margin without being generous.
    method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=50.0)
    try:
        inspect(rotor(), method=method, out="outputs", name="rotor")
    except SystemExit:
        pass
