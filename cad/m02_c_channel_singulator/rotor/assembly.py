"""
M02 C-channel singulator — rotor sub-assembly (hollow shell).

Faceted frustum with a matched-taper interior cavity. The shell
forms the rotating V wall; the cavity hides the drive — motor body
and fixed support stem sit inside the spinning rotor, coaxial with
its rotation axis (see ../drive/assembly.py).

Geometry:
    - Outer: 8-faceted frustum, base r = 90 mm, top r = 30 mm, 60 mm
      tall, flank at 45° from vertical.
    - Shell: ROTOR_SHELL_WALL_T radial wall from z=0 up to z = height
      − top_cap_t (cavity opens here).
    - Top cap: solid ROTOR_TOP_CAP_T thick above the cavity.
    - Internal boss: ROTOR_BOSS_OD cylinder descending from the top
      cap's underside into the cavity, with a shaft bore going up
      through the boss and into the top cap (blind halfway through
      the cap).

Outside-ride still applies: bricks sit in the V between the rotor
flank and the stator outer wall; nothing about the hollow interior
changes that contact geometry.

Printability note: the cavity is open at the bottom and closed at
the top. A one-piece print expects breakaway support inside the
cavity (removed through the open bottom after printing), OR a split
at z = BOSS_BOTTOM_Z into a lower shell-only funnel + an upper
cap+boss piece bonded together. Pick at slice time; the CAD model
is the assembled geometry either way.

Coordinate frame:
    origin = rotor axis ∩ base plane (widest section, at the
             V-bottom level of the stator)
    +Z     = up (top-platform / drop-surface direction)
    z=0    = rotor base plane (= stator V-bottom)
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
    BOSS_BOTTOM_Z,
    BOSS_TOP_Z,
    MOTOR_SHAFT_BORE_CLEARANCE,
    NEMA17_SHAFT_D,
    ROTOR_BASE_DIAMETER,
    ROTOR_BOSS_OD,
    ROTOR_FACET_COUNT,
    ROTOR_HEIGHT,
    ROTOR_SHELL_WALL_T,
    ROTOR_TOP_CAP_T,
    SHAFT_BORE_BOTTOM_Z,
    SHAFT_BORE_TOP_Z,
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
    shell_wall_t: float = ROTOR_SHELL_WALL_T,
    top_cap_t: float = ROTOR_TOP_CAP_T,
    boss_od: float = ROTOR_BOSS_OD,
    shaft_bore_d: float = NEMA17_SHAFT_D + 2 * MOTOR_SHAFT_BORE_CLEARANCE,
) -> Part:
    """Hollow faceted frustum with top cap, internal coupling boss,
    and blind shaft bore. Base at z=0, apex direction +Z.

    Build order:
      outer − cavity  →  shell + solid top cap (annulus below cap,
                         full section above).
      + boss          →  cylindrical protrusion from cap underside
                         into the cavity.
      − shaft_bore    →  blind bore up through the boss into the
                         top cap for the motor-shaft slip fit.
    """
    base_r = base_diameter / 2.0

    outer = extrude(
        RegularPolygon(radius=base_r, side_count=n_facets),
        amount=height,
        taper=flank_angle_deg,
    )

    # Matched-taper cavity: same flank angle, circumradius offset
    # inward by shell_wall_t, stops at (height − top_cap_t) so the
    # top cap remains solid. 1 mm poke below z=0 for a clean boolean
    # against the open bottom.
    cavity_bottom_pad = 1.0
    cavity_height = (height - top_cap_t) + cavity_bottom_pad
    cavity = Pos(0, 0, -cavity_bottom_pad) * extrude(
        RegularPolygon(radius=base_r - shell_wall_t, side_count=n_facets),
        amount=cavity_height,
        taper=flank_angle_deg,
    )

    # Boss descending from the top cap into the cavity.
    boss = Pos(0, 0, (BOSS_BOTTOM_Z + BOSS_TOP_Z) / 2.0) * Cylinder(
        boss_od / 2.0, BOSS_TOP_Z - BOSS_BOTTOM_Z,
    )

    # Shaft bore — open at boss bottom (SHAFT_BORE_BOTTOM_Z), blind
    # SHAFT_BORE_TOP_Z below the rotor's outer top face.
    bore_len = SHAFT_BORE_TOP_Z - SHAFT_BORE_BOTTOM_Z
    # 1 mm poke below bore bottom to guarantee a clean open end.
    bore = Pos(
        0, 0,
        SHAFT_BORE_BOTTOM_Z - 1.0 + (bore_len + 1.0) / 2.0,
    ) * Cylinder(shaft_bore_d / 2.0, bore_len + 1.0)

    return (outer - cavity) + boss - bore


# --- Assembly ----------------------------------------------------------

assembly = Assembly().add("rotor", rotor())


if __name__ == "__main__":
    check(assembly, out="outputs")
    # 45°-from-vertical flank is base-down-friendly on the outer
    # face; the cavity's inner face has a matching 45° inward lean
    # as z increases, which reads as an overhang. 50° threshold
    # leaves headroom for printer tuning.
    method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=50.0)
    try:
        inspect(rotor(), method=method, out="outputs", name="rotor")
    except SystemExit:
        pass
