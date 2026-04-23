"""
M03 scanner — kinematic mirror mount (rev 2, pitch/yaw adjustable).

Three-point kinematic mount. A back plate bolts to the enclosure wall,
a front plate holds the 80×80 mm first-surface mirror, and three M4
adjuster screws pass through the back plate and bear on cone seats in
the front plate. Compression springs on each screw shaft between the
two plates preload the front plate against the screw tips; turning
the adjusters tilts the front plate in pitch + yaw. After calibration,
jam nuts on the outside of the back plate lock each screw.

Adjuster layout: equilateral triangle at radius R_KIN around the
mirror centre. All three adjusters have identical hardware; in
operation one is the "pivot" (locked first), the other two are the
pitch + yaw knobs.

Hardware (stand-ins only in CAD):
  * Adjuster screws — M4 ball-tip or cone-point screws; thread engages
    heat-set M4 inserts pressed into the BP pilot holes.
  * Springs — compression, OD 6 / ID 4.3, free length slightly longer
    than GAP so they preload at the nominal setting.
  * Lock — M4 jam nut on each adjuster outside the back plate.

Drop-in bolt pattern: back plate footprint + corner bolts match the
rigid rev (`../mirror_mount/`). Z-stack is deeper — the kinematic BP
sits further behind the mirror face than the rigid plate — so the
enclosure wall behind it has a different standoff.

Coordinate frame (local to this sub-assembly):
    origin = centre of mirror reflective face
    +Z     = mirror normal (outward, toward camera/brick)
    z = 0                                 : mirror reflective face
    z ∈ [-MIRROR_T, 0]                    : mirror body
    z ∈ [_FP_BACK_Z, _FP_FRONT_Z]         : front plate body
    z ∈ [_BP_FRONT_Z, _FP_BACK_Z]         : spring / screw-shaft gap
    z ∈ [_BP_BACK_Z, _BP_FRONT_Z]         : back plate body
    adjuster axis = -Z at each triangle vertex (radius R_KIN)
    corner-bolt axis = -Z at each BP corner (inset BOLT_INSET)

The scanner top-level assembly maps local +Z to each mirror's
world-space normal and places the local origin at each mirror's world
centre.
"""

import math

from build123d import Box, Cone, Cylinder, Part, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM


# --- Parameters ---------------------------------------------------------

# Mirror — first-surface, square, 80 × 80 × 3.
MIRROR_W = 80.0
MIRROR_H = 80.0
MIRROR_T = 3.0

# Back plate — matches the rigid rev footprint.
BP_W = 110.0
BP_H = 110.0
BP_T = 5.0

# Front plate — smaller than BP so it can tilt freely.
FP_W = 100.0
FP_H = 100.0
FP_T = 4.0

# Nominal gap between FP back face and BP front face (≈ free spring length).
GAP = 10.0

# Adjuster triangle — equilateral, radius from mirror centre.
R_KIN = 25.0

# Adjuster hardware (M4).
INSERT_PILOT_D = 5.7           # BP pilot hole for heat-set M4 insert
SPRING_OD = 6.0
SPRING_ID = 4.3                # spring rides the screw shaft
SCREW_OD = 4.0                 # M4 shaft

# Cone seat in FP back face (receives screw tip). A small flat tip
# (frustum, not pure cone) keeps the wall-thickness ray sampler from
# generating spurious thin-wall readings at a mathematically sharp
# apex. CONE_TIP_D must stay below the kinematic seating limit —
# large enough and the ball bottoms on the flat instead of wedging
# on the cone walls. For the current geometry (mouth 5 mm, depth
# 2 mm, M4 ball r ≈ 1.5 mm) the analytical limit is tip_d ≈ 1.3 mm.
CONE_MOUTH_D = 5.0
CONE_TIP_D = 1.0
CONE_DEPTH = 2.0

# Enclosure mounting — same pattern as the rigid rev.
BOLT_HOLE_D = 4.5
BOLT_INSET = 8.0

# FDM printability floor.
WALL_MIN = 1.5


# --- Derived Z planes ---------------------------------------------------

_FP_FRONT_Z = -MIRROR_T
_FP_BACK_Z = _FP_FRONT_Z - FP_T
_BP_FRONT_Z = _FP_BACK_Z - GAP
_BP_BACK_Z = _BP_FRONT_Z - BP_T


# --- Parametric sanity checks -------------------------------------------

assert FP_T - CONE_DEPTH >= WALL_MIN, (
    f"FP wall below cone apex {FP_T - CONE_DEPTH:.2f} mm < {WALL_MIN} mm"
)
assert SPRING_ID > SCREW_OD, (
    f"spring ID {SPRING_ID} ≤ screw OD {SCREW_OD}"
)
assert CONE_MOUTH_D > SCREW_OD, (
    f"cone mouth {CONE_MOUTH_D} ≤ screw OD {SCREW_OD}"
)
assert R_KIN + INSERT_PILOT_D / 2 + WALL_MIN <= BP_W / 2, (
    "adjuster triangle extends past BP edge"
)
assert FP_W >= MIRROR_W and FP_H >= MIRROR_H, "front plate smaller than mirror"
assert BOLT_INSET - BOLT_HOLE_D / 2 >= WALL_MIN, (
    f"corner-bolt-to-edge wall {BOLT_INSET - BOLT_HOLE_D / 2:.2f} mm"
    f" < {WALL_MIN} mm"
)


# --- Adjuster triangle --------------------------------------------------

_ADJ_ANGLES_DEG = (0.0, 120.0, 240.0)


def _adj_xy(angle_deg: float, r: float = R_KIN) -> tuple[float, float]:
    rad = math.radians(angle_deg)
    return (r * math.cos(rad), r * math.sin(rad))


# --- Part functions -----------------------------------------------------

def _through_hole_z(hole_d: float, plate_t: float) -> Part:
    return Cylinder(hole_d / 2, plate_t * 3)


def _cone_pocket(mouth_d: float = CONE_MOUTH_D,
                 tip_d: float = CONE_TIP_D,
                 depth: float = CONE_DEPTH) -> Part:
    """Frustum with mouth at z=0 opening in -Z and flat tip at z=+depth.
    Caller translates so the mouth plane lands on the target face."""
    return Pos(0, 0, depth / 2) * Cone(mouth_d / 2, tip_d / 2, depth)


def back_plate() -> Part:
    """Printed rear plate. Four corner enclosure-bolt through-holes +
    three adjuster-screw pilot holes in the equilateral triangle."""
    cz = (_BP_BACK_Z + _BP_FRONT_Z) / 2
    plate = Pos(0, 0, cz) * Box(BP_W, BP_H, BP_T)
    dx = BP_W / 2 - BOLT_INSET
    dy = BP_H / 2 - BOLT_INSET
    for sx in (-1, 1):
        for sy in (-1, 1):
            plate -= (
                Pos(sx * dx, sy * dy, cz)
                * _through_hole_z(BOLT_HOLE_D, BP_T)
            )
    for angle in _ADJ_ANGLES_DEG:
        x, y = _adj_xy(angle)
        plate -= Pos(x, y, cz) * _through_hole_z(INSERT_PILOT_D, BP_T)
    return plate


def front_plate() -> Part:
    """Printed mirror-side plate. Mirror glues to the front face;
    three cone pockets in the back face receive adjuster screw tips."""
    cz = (_FP_BACK_Z + _FP_FRONT_Z) / 2
    plate = Pos(0, 0, cz) * Box(FP_W, FP_H, FP_T)
    for angle in _ADJ_ANGLES_DEG:
        x, y = _adj_xy(angle)
        plate -= Pos(x, y, _FP_BACK_Z) * _cone_pocket()
    return plate


def mirror_stub() -> Part:
    """First-surface mirror stand-in. Reflective face at z=0."""
    return Pos(0, 0, -MIRROR_T / 2) * Box(MIRROR_W, MIRROR_H, MIRROR_T)


def adjuster_screw_stub() -> Part:
    """M4 adjuster screw shaft stand-in. Shaft from behind BP (bolt-head
    clearance) to the FP back face plane. Tip detail (ball/cone) is
    not modelled — the FP cone pocket accommodates either."""
    z_top = _FP_BACK_Z
    z_bot = _BP_BACK_Z - 3.0
    length = z_top - z_bot
    cz = (z_top + z_bot) / 2
    return Pos(0, 0, cz) * Cylinder(SCREW_OD / 2, length)


def spring_stub() -> Part:
    """Compression spring stand-in: tube between BP front and FP back
    riding the adjuster shaft."""
    length = GAP
    cz = (_FP_BACK_Z + _BP_FRONT_Z) / 2
    shell = Cylinder(SPRING_OD / 2, length)
    bore = Cylinder(SPRING_ID / 2, length * 1.1)
    return Pos(0, 0, cz) * (shell - bore)


# --- Assembly -----------------------------------------------------------

assembly = (
    Assembly()
    .add("back_plate", back_plate())
    .add("front_plate", front_plate())
    .add("mirror", mirror_stub())
)

for idx, angle in enumerate(_ADJ_ANGLES_DEG):
    x, y = _adj_xy(angle)
    assembly = (
        assembly
        .add(f"screw_{idx}", adjuster_screw_stub(), location=Pos(x, y, 0))
        .add(f"spring_{idx}", spring_stub(), location=Pos(x, y, 0))
    )

assembly = (
    assembly
    .assert_no_interference("back_plate", "front_plate")
    .assert_no_interference("back_plate", "mirror")
    .assert_no_interference("front_plate", "mirror")
)

for idx in range(3):
    screw = f"screw_{idx}"
    spring = f"spring_{idx}"
    assembly = (
        assembly
        .assert_no_interference(screw, "back_plate")
        .assert_no_interference(screw, "front_plate")
        .assert_no_interference(screw, "mirror")
        .assert_no_interference(spring, "back_plate")
        .assert_no_interference(spring, "front_plate")
        .assert_no_interference(spring, screw)
    )


if __name__ == "__main__":
    check(assembly, out="outputs")
    inspect(
        back_plate(),
        method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=45.0),
        out="outputs",
        name="back_plate",
    )
    # Front plate is intended to print FRONT-face-down (mirror-glue
    # side on the bed): the cone pockets then open upward and print
    # support-free. `up_axis=-Z` tells the inspector to check overhangs
    # in that orientation. It also yields the smoothest possible
    # finish on the mirror-glue face.
    inspect(
        front_plate(),
        method=FDM(
            up_axis=(0, 0, -1),
            wall_min_mm=WALL_MIN,
            overhang_max_deg=45.0,
        ),
        out="outputs",
        name="front_plate",
    )
