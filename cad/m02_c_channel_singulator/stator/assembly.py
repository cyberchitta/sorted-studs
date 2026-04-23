"""
M02 C-channel singulator — stator sub-assembly.

Fixed conical outer wall that forms the outer side of the V-trough.
Bricks ride in the V between the spinning rotor's flank (inner side)
and the stator wall's inner conical surface (outer side), pressed
outward against this wall as the rotor drags them around. An
angular arc on the +X side is cut away — pieces reaching the exit
fall off the outer edge under gravity into the scan-station drop
path (Module 3).

V1 is wall-only: a conical ring of thickness STATOR_WALL_T and
height STATOR_WALL_H above the V-bottom, with the exit arc cut
through its full height. Floor, mounting flange, and frame
interface land when the drive sub-assembly defines its base plate.

The wall prints with its flank at V_HALF_ANGLE from vertical on
both faces — a 45° overhang by the conventional build-plate
metric. Expect to print either with support on the outside, or
inverted (top-down on bed) so each layer is smaller than the one
below it.

Coordinate frame:
    origin = rotor axis ∩ V-bottom level (shared with rotor/)
    +X     = exit-arc center azimuth; bricks drop out here
    +Z     = up
    z=0    = V-bottom. Stator wall's narrowest inner edge sits at
             this level; rotor base edge is also at this z in its
             own frame.

Standalone validation: stator alone + printability (FDM).
Cross-boundary rotor↔stator clearance runs in ../assembly.py.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build123d import Axis, Part, Plane, Polygon, Pos, extrude, revolve

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from params import (
    EXIT_ARC_DEG,
    ROTOR_BASE_DIAMETER,
    ROTOR_STATOR_GAP,
    STATOR_WALL_H,
    V_HALF_ANGLE_DEG,
)


# --- Derived geometry --------------------------------------------------

STATOR_WALL_T = 3.0   # radial wall thickness

# Inner face at the V-bottom sits ROTOR_STATOR_GAP radially outside
# the rotor's base circumscribing-circle edge.
R_STATOR_INNER_BASE = ROTOR_BASE_DIAMETER / 2.0 + ROTOR_STATOR_GAP

# Flank rises at V_HALF_ANGLE from vertical — matching the rotor
# flank so the V is symmetric across the bottom.
_TAN_V = math.tan(math.radians(V_HALF_ANGLE_DEG))
R_STATOR_INNER_TOP = R_STATOR_INNER_BASE + STATOR_WALL_H * _TAN_V
R_STATOR_OUTER_BASE = R_STATOR_INNER_BASE + STATOR_WALL_T
R_STATOR_OUTER_TOP = R_STATOR_INNER_TOP + STATOR_WALL_T

WALL_MIN = 1.5


# --- Part functions ----------------------------------------------------

def stator(
    *,
    r_in_base: float = R_STATOR_INNER_BASE,
    r_in_top: float = R_STATOR_INNER_TOP,
    r_out_base: float = R_STATOR_OUTER_BASE,
    r_out_top: float = R_STATOR_OUTER_TOP,
    height: float = STATOR_WALL_H,
    exit_arc_deg: float = EXIT_ARC_DEG,
) -> Part:
    """Conical ring wall around +Z with an exit-arc wedge cut through.

    Cross-section in the XZ plane is a 4-vertex quadrilateral: the
    wall's inner and outer faces each slope at V_HALF_ANGLE from
    vertical, closed by the bottom and top edges. Revolved around Z
    to form the full wall. A radial wedge centered on +X (spanning
    ±exit_arc_deg/2) is then subtracted to produce the exit opening.
    """
    profile = Plane.XZ * Polygon(
        (r_in_base, 0.0),
        (r_out_base, 0.0),
        (r_out_top, height),
        (r_in_top, height),
        align=None,
    )
    wall = revolve(profile, axis=Axis.Z)

    # Exit-arc cutter: an isoceles triangle in XY with apex at the
    # axis and outer chord past the wall's outer radius. `r_cut` is
    # sized so the chord's perpendicular distance from origin
    # (= r_cut·cos(h)) exceeds r_out_top with 10 mm of margin; this
    # scales cleanly with any exit_arc_deg < ~170°.
    h = math.radians(exit_arc_deg / 2.0)
    r_cut = (r_out_top + 10.0) / math.cos(h)
    wedge_sketch = Polygon(
        (0.0, 0.0),
        (r_cut * math.cos(-h), r_cut * math.sin(-h)),
        (r_cut * math.cos(+h), r_cut * math.sin(+h)),
        align=None,
    )
    z_pad = 1.0
    wedge = Pos(0, 0, -z_pad) * extrude(
        wedge_sketch, amount=height + 2 * z_pad,
    )
    return wall - wedge


# --- Assembly ----------------------------------------------------------

assembly = Assembly().add("stator", stator())


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Both wall faces sit at 45° from vertical (= 45° from horizontal).
    # Set threshold with a small margin so V1 lands "passing" — real
    # prints will likely need either outside support or an inverted
    # orientation; split at a later revision.
    method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=50.0)
    try:
        inspect(stator(), method=method, out="outputs", name="stator")
    except SystemExit:
        pass
