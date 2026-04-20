"""
M05 hex-tower — rotating ramp mechanism (single floor).

One floor's ramp + shaft + floor-plate assembly. The ramp tilts between
a DOWN state (straight down from pivot — bricks fall past) and an UP
state (slightly below horizontal — intercepts bricks and slides them
outward toward a basket). Shaft rotation (not modeled here) re-aims the
ramp in azimuth to address different baskets.

Tilt mechanism choice: **separate servo** (not cam). The servo body
lives on the hub and is outside the scope of this geometry check; we
just model the pivot axis (tilt pin) and the swept states.

Coordinate frame:
    origin = shaft axis ∩ floor-plate top plane
    +Z     = up (shaft axis)
    +X     = ramp outward at azimuth 0 (the radial direction)
    +Y     = tilt-pin axis (perpendicular to the radial)

Pivot layout (yoke + tang):
    The hub is a cube on the shaft with a **yoke** — two arms separated
    by a slot — extending from its +X face. The ramp is a **tang** that
    sits in the slot and pivots on a pin passing through yoke-arm →
    tang → yoke-arm. The pivot axis is at x = PIN_X > HUB_SIZE/2, so it
    is clear of the shaft and of the hub body. Because the pivot sits
    outside the hub, the ramp can be symmetric about Y = 0 — the swing
    plane of the ramp tang lies on the radial line, not offset to one
    side.

Tilt convention:
    tilt = 0°  — ramp horizontal, pointing +X.
    Rot(0, tilt, 0) rotates the outer tip toward -Z.
    tilt = 90° — ramp straight down (DOWN state).
    tilt = 10° — slightly below horizontal, sloping outward-down (UP).

Structural notes (to revisit):
    - Bearing boss is modeled floating at the shaft; in reality it is
      connected to the floor plate via radial spokes. Spokes deferred.
    - Floor plate is a square annulus with a large square central hole
      sized so the ramp's swing sweep at floor-plate z stays inside it.
      Hex shape is a later refinement.
    - Ramp plate is 10 mm wide — just wide enough to fit through the
      yoke slot. For reliable catching of 80 mm bricks a fanned outer
      section or a wider plate (with a narrower central tang) is a
      follow-up.
"""

from build123d import Box, Cylinder, Location, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM


# --- Parameters ---------------------------------------------------------

# Shaft — 6 mm steel rod (common NEMA 17 output shaft size).
SHAFT_D = 6.0
SHAFT_Z_BOT = -30.0
SHAFT_Z_TOP = 60.0

# Floor plate — square plate minus square central hole. Hole must be
# large enough that the ramp's swing sweep at z ∈ [-FLOOR_T, 0] stays
# entirely inside the hole at every tilt in [TILT_UP, TILT_DOWN].
FLOOR_OUTER_HALF = 100.0
FLOOR_INNER_HALF = 60.0
FLOOR_T = 5.0

# Bearing boss — cylindrical tower on the shaft, sitting on top of the
# floor plate. (Structural connection to the floor plate via spokes is
# deferred; for now it floats at the shaft.)
BOSS_OD = 20.0
BOSS_ID = 6.3                         # 0.3 mm radial clearance over shaft
BOSS_H = 12.0

# Hub + yoke — one printed part. The cube portion clamps to the shaft
# and carries the yoke, which protrudes from the hub's +X face and holds
# the tilt pin.
HUB_SIZE = 22.0                       # cube side, centered on shaft axis at PIVOT_Z
HUB_BORE_D = 6.3                      # shaft clearance
PIN_HOLE_D = 3.6                      # tilt pin clearance

# Yoke geometry — two arms on either side of the tang slot, both rising
# from the hub's +X face.
# YOKE_ARM_L chosen so: (1) PIN_X centered in the yoke gives equal walls
# on both sides of the pin bore; (2) the ramp's knuckle (radius
# KNUCKLE_OD/2) starts at x >= HUB_SIZE/2 + clearance, well clear of the
# hub face.
YOKE_ARM_L = 12.0                     # yoke extension from hub face (+X)
YOKE_ARM_T = 4.0                      # thickness of each yoke arm along Y
YOKE_SLOT_Y = 12.0                    # clear width of the slot along Y
YOKE_H = 12.0                         # yoke height along Z (centered on pivot)
YOKE_TOTAL_W = YOKE_SLOT_Y + 2 * YOKE_ARM_T  # 20

# Pivot placement. Pin centered in the yoke arm (midway between hub face
# and yoke tip) so both walls around the pin bore are comfortable.
PIN_X = HUB_SIZE / 2 + YOKE_ARM_L / 2  # 17.0
YOKE_END_X = HUB_SIZE / 2 + YOKE_ARM_L  # 23.0
PIVOT_Z = 24.0                        # pivot height — hub bottom clears the boss

# Tilt pin — through both yoke arms and the ramp tang, symmetric about Y=0.
PIN_D = 3.0
PIN_LEN = 24.0                        # covers yoke (20 wide) with 2 mm margin each side

# Ramp (tang + plate). Canonical pose: pivot at origin, plate extends +X.
# Width RAMP_W sized to fit through YOKE_SLOT_Y with slot clearance.
RAMP_L = 45.0
RAMP_W = 10.0                         # fits in 12-mm slot with 1 mm clearance each side
RAMP_T = 4.0
KNUCKLE_OD = 10.0                     # cylindrical boss around the pivot for pin-hole wall

# Tilt angles (deg) about Y through the pivot. The two endpoints are the
# operational states; the intermediate samples exist only to verify that
# the ramp's sweep between them clears the floor plate.
TILT_UP = 10.0                        # 10° below horizontal, sloping outward-down
TILT_DOWN = 90.0                      # straight down
TILT_SAMPLES = (30.0, 45.0, 60.0)     # intermediate sweep samples for clearance check

WALL_MIN = 1.5
RAMP_FLOOR_CLEARANCE = 1.0


# --- Helpers ------------------------------------------------------------

def _cyl_along_y(r: float, length: float) -> Part:
    """Cylinder centered at origin, axis along Y."""
    return Rot(90, 0, 0) * Cylinder(r, length)


# --- Part functions -----------------------------------------------------

def shaft() -> Part:
    length = SHAFT_Z_TOP - SHAFT_Z_BOT
    z_center = (SHAFT_Z_TOP + SHAFT_Z_BOT) / 2
    return Pos(0, 0, z_center) * Cylinder(SHAFT_D / 2, length)


def floor_plate() -> Part:
    outer = Pos(0, 0, -FLOOR_T / 2) * Box(FLOOR_OUTER_HALF * 2, FLOOR_OUTER_HALF * 2, FLOOR_T)
    hole = Pos(0, 0, -FLOOR_T / 2) * Box(FLOOR_INNER_HALF * 2, FLOOR_INNER_HALF * 2, FLOOR_T * 2)
    return outer - hole


def bearing_boss() -> Part:
    z_center = BOSS_H / 2
    body = Pos(0, 0, z_center) * Cylinder(BOSS_OD / 2, BOSS_H)
    bore = Pos(0, 0, z_center) * Cylinder(BOSS_ID / 2, BOSS_H * 2)
    return body - bore


def ramp_hub() -> Part:
    """Hub cube on the shaft + yoke that extends from the +X face and
    holds the tilt pin. Placed at (0, 0, PIVOT_Z) in the assembly.

    Local frame: hub cube centered on origin. Yoke occupies the region
    x ∈ [HUB_SIZE/2, YOKE_END_X], split along Y by a slot.
    """
    # Central cube on shaft, with through-bore for the shaft.
    cube = Box(HUB_SIZE, HUB_SIZE, HUB_SIZE)
    shaft_bore = Cylinder(HUB_BORE_D / 2, HUB_SIZE * 3)

    # Yoke body: a block from the +X face outward; the slot between the
    # two arms is cut out. Yoke is centered on z=0 (pivot height) with
    # height YOKE_H.
    yoke_x_center = HUB_SIZE / 2 + YOKE_ARM_L / 2
    yoke_block = Pos(yoke_x_center, 0, 0) * Box(YOKE_ARM_L, YOKE_TOTAL_W, YOKE_H)
    yoke_slot = Pos(yoke_x_center, 0, 0) * Box(YOKE_ARM_L * 2, YOKE_SLOT_Y, YOKE_H * 2)
    yoke = yoke_block - yoke_slot

    # Pin bore: through both yoke arms along Y at the pivot axis.
    pin_bore = Pos(PIN_X, 0, 0) * _cyl_along_y(PIN_HOLE_D / 2, YOKE_TOTAL_W * 2)

    return cube + yoke - shaft_bore - pin_bore


def tilt_pin() -> Part:
    return _cyl_along_y(PIN_D / 2, PIN_LEN)


def ramp() -> Part:
    """Tang-and-plate ramp. Canonical pose: pivot at origin, plate along
    +X, symmetric about Y=0. The inner region (tang) sits in the yoke
    slot; the plate extends outward.
    """
    plate = Pos(RAMP_L / 2, 0, 0) * Box(RAMP_L, RAMP_W, RAMP_T)
    knuckle = _cyl_along_y(KNUCKLE_OD / 2, RAMP_W)
    body = plate + knuckle
    pin_hole = _cyl_along_y(PIN_HOLE_D / 2, RAMP_W * 2)
    return body - pin_hole


# --- Assembly -----------------------------------------------------------

ramp_pivot_loc = Location((PIN_X, 0, PIVOT_Z))

assembly = Assembly()
assembly = (
    assembly.add("shaft", shaft())
    .add("floor_plate", floor_plate())
    .add("bearing_boss", bearing_boss())
    .add("ramp_hub", ramp_hub(), location=Location((0, 0, PIVOT_Z)))
    .add("tilt_pin", tilt_pin(), location=Location((PIN_X, 0, PIVOT_Z)))
    .add("ramp_down", ramp(), location=ramp_pivot_loc * Rot(0, TILT_DOWN, 0))
    .add("ramp_up", ramp(), location=ramp_pivot_loc * Rot(0, TILT_UP, 0))
)
# Intermediate-tilt samples — included so the floor-plate clearance check
# covers the full sweep, not just the two operational endpoints.
for angle in TILT_SAMPLES:
    assembly = assembly.add(
        f"ramp_at_{int(angle)}",
        ramp(),
        location=ramp_pivot_loc * Rot(0, angle, 0),
    )

assembly = (
    assembly
    # Fixed parts must not interfere with each other.
    .assert_no_interference("shaft", "floor_plate")
    .assert_no_interference("shaft", "bearing_boss")
    .assert_no_interference("shaft", "ramp_hub")
    .assert_no_interference("shaft", "tilt_pin")
    .assert_no_interference("floor_plate", "bearing_boss")
    .assert_no_interference("floor_plate", "ramp_hub")
    .assert_no_interference("bearing_boss", "ramp_hub")
    .assert_no_interference("ramp_hub", "tilt_pin")
    # Ramp DOWN state clears every fixed part.
    .assert_no_interference("ramp_down", "shaft")
    .assert_no_interference("ramp_down", "floor_plate")
    .assert_no_interference("ramp_down", "bearing_boss")
    .assert_no_interference("ramp_down", "ramp_hub")
    .assert_no_interference("ramp_down", "tilt_pin")
    # Ramp UP state clears every fixed part.
    .assert_no_interference("ramp_up", "shaft")
    .assert_no_interference("ramp_up", "floor_plate")
    .assert_no_interference("ramp_up", "bearing_boss")
    .assert_no_interference("ramp_up", "ramp_hub")
    .assert_no_interference("ramp_up", "tilt_pin")
    # Ramp edges stay clear of the floor plate across the full sweep —
    # both endpoints plus intermediate samples.
    .assert_clearance("ramp_down", "floor_plate", min_mm=RAMP_FLOOR_CLEARANCE)
    .assert_clearance("ramp_up", "floor_plate", min_mm=RAMP_FLOOR_CLEARANCE)
)
for angle in TILT_SAMPLES:
    assembly = assembly.assert_clearance(
        f"ramp_at_{int(angle)}", "floor_plate", min_mm=RAMP_FLOOR_CLEARANCE
    )


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Shaft, tilt pin, and extrusion stubs are stock; only the printed
    # parts get printability checks. The hub's yoke arms and the ramp's
    # knuckle have real downward-facing undersides — slicer supports or
    # re-orientation at print time will handle them, so keep the overhang
    # check informational here and assert only on wall thickness.
    method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0)
    inspect(floor_plate(),  method=method, out="outputs", name="floor_plate")
    inspect(bearing_boss(), method=method, out="outputs", name="bearing_boss")
    inspect(ramp_hub(),     method=method, out="outputs", name="ramp_hub")
    inspect(ramp(),         method=method, out="outputs", name="ramp")
