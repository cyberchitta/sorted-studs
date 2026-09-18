"""
M05 hex-tower — rotating square-column trap + fixed ramp (generic floor).

Column-side sub-assembly of the rotating-column delivery design for
Approach F. The column is a square tube; a circular **rotating collar**
at z_local = 220..225 of *each* column rides inside the **bearing of
the floor above** (a short ring at that floor's z_local = 0..5, held
coaxial with the column by a 6-saddle spider that clamps onto radial
2020 aluminum extrusions). Columns couple end-to-end at z_world =
N·220 + 273 via face flanges integrated into each column's top.

Delivery is split into two decoupled jobs:
- **Trap** — a flat plate hinged at the −X interior wall (the wall
  opposite the exit), free end toward +X. At rest it **hangs** straight
  down along the −X wall, out of the fall path: a brick bound for a
  lower floor falls past it. A servo on the column's +Y face, coaxial
  with the hinge, swings it up to **catch** — tilted down toward +X,
  so a brick landing on it slides off the free end onto the ramp root.
  Gravity biases it to pass-through: a dead servo, a missed command or
  a power loss degrades to "brick falls to the bottom fallback bin",
  never to a wrong basket (`_notes/decisions/m05-hex-cascade.md`,
  "Default-down, fail-through"). The servo holds the catch pose
  against impact on its own — no hard stop, because a brick's load
  pushes the plate back toward hanging, the direction no stop in the
  swing path can resist.
- **Ramp** — a fixed sloped plate rooted just inside the +X wall,
  with a side wall along each edge so a brick cannot leave it
  sideways. Catches the brick as it slides off the catching trap and
  delivers it through the exit opening to the target basket (basket
  geometry lives in the basket sub-assembly).

Decoupling trap motion from ramp trajectory: the ramp angle is a
free design parameter sized for the basket, not coupled to the
trap's catch-state geometry.

Standalone validation here covers the column-side parts only. Basket
placement + ramp-vs-basket clearance are checked in the top-level
`../assembly.py`.

The model is parameterized on **floor role** ∈ {"base", "middle",
"top"}. Under the **top-bearing architecture** the bearing+spider
geometry is uniform across all roles (a short ring at z_local = 0..5
gripping the lower column's body where its rotating collar sits at
z_lower_local = 220..225). Roles differ only in the column itself:

- **base** — column body extends BASE_COL_Z_BOT (16.5)..273 in its own
  local frame, standing on a bought turntable bearing: the spider's hub
  is the turntable seat (z 0..8), the turntable sits on it (8..16.5),
  and the column's Ø204 bottom flange (16.5..21.5) sits on the
  turntable. The base_floor ring gear bolts to that flange's underside
  and reaches the off-axis pinion below the rails through the
  turntable's and seat's bores. Rotating collar at 220..225, top flange
  at 268..273.
- **middle**, **top** — column body extends 53..273; bottom face at
  z = 53 mates with the lower column's top flange (z_lower_local =
  268..273) at z_world = N·220 + 273. No bottom collar. Top floor
  additionally has its rotating collar engaged from above by the
  ceiling bearing in the hopper sub-assembly (not modeled here).

The bearing slot on **this** floor sits at z_local = 0..5 and engages
the *lower* column's rotating collar — represented in the standalone
middle/top assembly by `lower_column` placed at z = -FLOOR_H. Without
that stand-in there is nothing here for this floor's bearing to grip.

Motor pinion + ring gear and the ceiling bearing are deferred — see
the rolling NEXT_STEPS docs. The trap servo is a bought-part envelope
(MG90S-class); its mount bracket and the horn-to-pin coupling are not
modeled. Column-to-column joint
torque transfer uses bas-relief bumps + depressions (Option A from
NEXT_STEPS "Column-to-column joint"); outboard castellation
(Option B) is held in reserve.

Coordinate frame:
    origin = column axis ∩ floor datum (z=0 = bearing/spider base)
    +Z     = up
    +X     = radial outward toward the exit opening
    +Y     = tangent at the opening (hinge axis direction)

Trap tilt convention (`Rot(0, θ, 0)` about the hinge; +θ tips the
+X free end down):
    θ = 90° — hanging straight down (rest / pass-through).
    θ = TRAP_CATCH_DEG (30°) — free end below the hinge, sloped toward
            the exit (servo-driven / catching).

Gravity bias: the free end wants to fall to 90° ⇒ pass-through is
stable without active force.
"""

import math
from typing import Callable


from build123d import Box, Cylinder, Location, Part, Plane, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from ..params import (
    BASE_COL_Z_BOT,
    BASE_FLANGE_OD,
    BASKET_RIM_Z,
    COL_INTERIOR_SIDE,
    COL_OUTER_SIDE,
    COLLAR_CLEARANCE,
    COLLAR_OD,
    FLOOR_H,
    MAX_BRICK_DIM,
    N_BASKETS_PER_FLOOR,
    RAIL_INNER_R,
    RAIL_SIZE,
    R_BASKET_INNER,
    TURNTABLE_ID,
    TURNTABLE_OD,
    TURNTABLE_Z_BOT,
    TURNTABLE_Z_TOP,
)


# --- Derived parameters -------------------------------------------------

# Circular features on the column (rotating collar at mid-height,
# top flange at the joint plane, optional bottom collar on base).
# COLLAR_OD / COLLAR_CLEARANCE live in params (shared with base_floor,
# whose drive pinion meshes with the base column's bottom collar).
COLLAR_T = 5.0

# Hinge pin + knuckle
KNUCKLE_OD = 10.0
PIN_D = 3.0
PIN_HOLE_D = 3.6
PIN_LEN = COL_OUTER_SIDE + 4.0

# Trap — flat plate, knuckle at hinge axis, plate body in +X
TRAP_T = 4.0
TRAP_W = 80.0                                 # ≤ MAX_BRICK_DIM; leaves ±Y for bosses
TRAP_HINGE_X = -(COL_INTERIOR_SIDE / 2 - 7.0)  # −38 — inset so knuckle clears −X wall
TRAP_REST_DEG = 90.0                          # hanging: pass-through
# Catch tilt — the brick must slide off the trap the way it slides off
# the ramp (tan > MU_STATIC_BUDGET, below); 30° (tan 0.577) clears it
# with more room than the ramp's 25°, since the trap is where the
# brick lands and first has to start moving.
TRAP_CATCH_DEG = 30.0
TRAP_SAMPLES = (45.0, 60.0, 75.0)             # swing samples between catch and rest
_TRAP_CATCH_RAD = math.radians(TRAP_CATCH_DEG)
# At the catch tilt the free end reaches to TRAP_TIP_X, short of the
# ramp root (RAMP_ROOT_X = 43) so the brick steps down onto the ramp
# rather than the trap overlapping it. That fixes the hinge-to-tip
# reach, and so the plate length.
TRAP_TIP_X = COL_INTERIOR_SIDE / 2 - 4.0      # 41
_TRAP_REACH = (TRAP_TIP_X - TRAP_HINGE_X) / math.cos(_TRAP_CATCH_RAD)  # ≈ 91.2
TRAP_L = _TRAP_REACH - KNUCKLE_OD / 2         # ≈ 86.2

# Hinge bosses — mounted on inside of −X wall, extend inward past hinge,
# sit outside the trap knuckle's Y extent so they clear the swinging plate.
BOSS_Y = 3.0
BOSS_Y_OFFSET = TRAP_W / 2 + BOSS_Y / 2 + 0.5           # 42
BOSS_X = COL_INTERIOR_SIDE / 2 - (-TRAP_HINGE_X - KNUCKLE_OD / 2 - 1.0)  # 13
BOSS_X_CENTER = -(COL_INTERIOR_SIDE / 2 - BOSS_X / 2)   # −38.5
BOSS_H_Z = 12.0

# Ramp — fixed sloped plate, root just inside the +X wall, extends
# through the exit. The ramp root is the datum the trap is hung from
# (TRAP_HINGE_Z below), not the other way round: the ramp is sized
# against the basket rim and the exit opening.
RAMP_DEG = 25.0                               # slope below horizontal
RAMP_T = 4.0
RAMP_W = 70.0                                 # channel width between the side walls
RAMP_L = 112.0                                # length along slope direction
# Side walls — outboard of the channel, full length, standing on the
# plate. They keep a brick that lands off-centre from leaving the plate
# sideways (2026-09-11 fabrication review). Outboard rather than inboard
# so the channel stays the full RAMP_W; the cost is the outer width,
# which is what the opening and the basket slot have to pass.
RAMP_WALL_T = 2.0                             # ≥ FDM wall floor (1.5); takes brick impacts
RAMP_WALL_H = 15.0                            # above the plate top — a brick height + margin
RAMP_OUTER_W = RAMP_W + 2 * RAMP_WALL_T       # 74 — what the opening must pass
RAMP_ROOT_X = COL_INTERIOR_SIDE / 2 - 2.0     # 43 — just inside +X wall
RAMP_ROOT_Z = 185.0

# Hinge height — the catching trap's free-end underside sits
# TRAP_HANDOFF_STEP above the ramp's top face at its root, so the brick
# drops a short step from trap to ramp. Measured, not just derived: the
# `trap_catch_to_ramp_step` assert_distance in `make_assembly`.
TRAP_HANDOFF_STEP = 5.0
_RAMP_ROOT_TOP_Z = RAMP_ROOT_Z + (RAMP_T / 2) / math.cos(math.radians(RAMP_DEG))
TRAP_HINGE_Z = (
    _RAMP_ROOT_TOP_Z + TRAP_HANDOFF_STEP
    + _TRAP_REACH * math.sin(_TRAP_CATCH_RAD)
    + (TRAP_T / 2) * math.cos(_TRAP_CATCH_RAD)
)                                             # ≈ 239.1

# Trap servo — bought MG90S-class micro servo (metal gears: it takes
# every catch impact through its gear train, since nothing else holds
# the catch pose). Envelope only. Mounted outside the +Y wall with its
# output shaft coaxial with the hinge, long axis along +X, so its
# height stays a body width (≈ 12) about the hinge: the floor above's
# bearing spider occupies the band just below it outside the column.
SERVO_BODY_L = 22.5                           # along X (long axis)
SERVO_BODY_W = 12.2                           # along Z
SERVO_BODY_H = 22.7                           # along Y (shaft axis), case only
SERVO_TAB_L = 32.2                            # mounting-tab span along X
SERVO_TAB_T = 2.5
SERVO_TAB_FROM_TOP = 6.7                      # tab top face below the shaft face
SERVO_SHAFT_FROM_END = SERVO_BODY_W / 2       # shaft axis from the body's −X end
SERVO_STANDOFF = 6.0                          # shaft face → column outer wall: horn + hub
SERVO_STALL_NM = 0.22                         # MG90S, 2.2 kg·cm at 4.8 V (datasheet)
# Static hold budget: the plate's own weight (solid PLA — conservative)
# plus a heavy brick at the free end, at the catch tilt. Impact is not
# in this number; it is a bench question.
PLA_DENSITY_G_PER_MM3 = 1.24e-3
BRICK_MASS_BUDGET_G = 30.0
SERVO_HOLD_MARGIN_MIN = 3.0

# Slide condition: the brick must overcome static friction on the ramp,
# i.e. tan(RAMP_DEG) > µ_s. ABS brick on a printed-PLA ramp is
# µ_s ≈ 0.3–0.4; budget 0.45 adds headroom for dirt, wear, and layer
# texture. 25° (tan = 0.466) clears the budget; 20° (tan = 0.364) sat
# inside the µ_s range itself — bricks could stall on the ramp.
# Asserted as `ramp_slide_margin` in `build_column_side_rotating`.
MU_STATIC_BUDGET = 0.45

# Ramp tip — targeting interface. The sweep-clearance claim (every
# point of the rotating ramp stays ≥ RAMP_SWEEP_CLEARANCE above the
# basket rim plane at all azimuths — rotation about the column axis is
# Z-invariant, so the plane form covers every azimuth with no sweep)
# is the `ramp` → `Plane.XY.offset(BASKET_RIM_Z)` assert_distance in
# `build_column_side_rotating`; if it fails, rebalance RAMP_DEG /
# RAMP_L / RAMP_ROOT_Z (TRAP_HINGE_Z follows RAMP_ROOT_Z).
RAMP_SWEEP_CLEARANCE = 5.0
_RAMP_DEG_RAD = math.radians(RAMP_DEG)
RAMP_TIP_X = RAMP_ROOT_X + RAMP_L * math.cos(_RAMP_DEG_RAD)
RAMP_TIP_Z = RAMP_ROOT_Z - RAMP_L * math.sin(_RAMP_DEG_RAD)

# Exit opening in the +X column wall — must pass the ramp plus a brick
# sliding on top of it. Ramp enters the wall at x = COL_OUTER_SIDE/2.
_RAMP_WALL_OFFSET = COL_OUTER_SIDE / 2 - RAMP_ROOT_X
_RAMP_WALL_Z = RAMP_ROOT_Z - _RAMP_WALL_OFFSET * math.tan(_RAMP_DEG_RAD)
OPENING_MARGIN = 5.0                          # vertical: below the plate, above the brick
# Side margin is tighter than the vertical one: when the hinge bosses
# sat on this wall they capped the opening at 83 mm, and the side walls
# spent the difference. The bosses have since moved to the −X wall;
# 3 mm each side of a fixed part through a printed cut is still
# comfortable, so the margin stayed.
OPENING_SIDE_MARGIN = 3.0
OPENING_W = RAMP_OUTER_W + 2 * OPENING_SIDE_MARGIN   # 80
# Vertical rise of the wall top above the plate centreline, measured at
# the column wall: the walls are tilted with the plate, so their top edge
# sits (RAMP_T/2 + RAMP_WALL_H) perpendicular to it. The opening top is
# centreline + MAX_BRICK_DIM + OPENING_MARGIN, so the wall tops clear the
# cut by the full margin iff this rise ≤ MAX_BRICK_DIM (asserted in
# `column`).
_RAMP_WALL_TOP_RISE = (RAMP_T / 2 + RAMP_WALL_H) / math.cos(_RAMP_DEG_RAD)
OPENING_Z_BOT = _RAMP_WALL_Z - RAMP_T - OPENING_MARGIN
OPENING_Z_TOP = _RAMP_WALL_Z + MAX_BRICK_DIM + OPENING_MARGIN


# Basket angular-slot width at ramp tip must admit the ramp's outer width.
# This is an UPPER BOUND on the real slot: basket/assembly.py shrinks
# each basket's angular span below 360°/N so its radial walls clear
# the radial-rail corridor, so the real slot at the tip is narrower
# than this formula reports. The tight cross-boundary check lives in
# the top-level ../assembly.py as basket_plus_x-vs-ramp
# no-interference, which will fail if ramp geometry actually clashes
# with the (shrunk) basket walls. Sibling-imports are forbidden, so
# we can't pull BASKET_HALF_DELTA_DEG in here; keep this as a cheap
# parametric sanity check only.
BASKET_SLOT_W_AT_TIP = 2 * RAMP_TIP_X * math.tan(math.pi / N_BASKETS_PER_FLOOR)

# Column height — pinned by the column-to-column joint plane. Joints
# stack at z_world = N·FLOOR_H + COL_Z_TOP, with each column extending
# (COL_Z_TOP - FLOOR_H) above the floor's own bracket level to form the
# overhang that the floor *above* grips. The 53 mm overhang must clear
# the +X opening top — asserted in `column()`. The trap no longer
# reaches up into it: its highest point at every pose is the knuckle.
RAIL_TOP_Z = 0.0                                     # frame datum (rail top face)
COL_Z_TOP = 273.0                                    # joint plane in this floor's frame

# The opening top is a parametric check: the opening is a cut feature,
# not a part.
JOINT_ZONE_CLEARANCE = 5.0

# Joint flange thickness at the column top. Top 5 mm of the column
# carries a circular face flange (OD = COLLAR_OD) for end-to-end
# coupling with the column above. Bas-relief interlock (Option A;
# see _notes/state/m05-diverter.md "Column-to-column joint") — bumps on
# this flange's top face slot into matching depressions on the upper
# column's bottom-flange face for tangential torque lock.
JOINT_FLANGE_T = COLLAR_T                            # 5.0
JOINT_FLANGE_Z_BOT = COL_Z_TOP - JOINT_FLANGE_T      # 268
JOINT_FLANGE_Z_TOP = COL_Z_TOP                       # 273

# Bas-relief interlock features at the column-to-column joint plane.
# Four cylindrical bumps on the lower column's top-flange face fit
# into four matching depressions on the upper column's bottom-flange
# face. 4-fold symmetry at cardinal azimuths {0°, 90°, 180°, 270°}
# both maximizes the radial overhang band (column interior is square,
# so cardinal azimuths give r ∈ [45, 69] vs. r ∈ [67.9, 69] at the
# diagonals) and is a refinement of the 2-fold lock that an
# alternating-opening install would need.
N_JOINT_BUMPS = 4
JOINT_BUMP_CENTERLINE_R = 56.0    # midway between r=45 (column hole) and r=69 (OD)
JOINT_BUMP_D = 12.0
JOINT_BUMP_H = 1.5
JOINT_BUMP_CLEARANCE_R = 0.3      # depression is this much larger in radius (per side)
JOINT_BUMP_CLEARANCE_Z = 0.2      # depression is this much deeper than the bump is tall
JOINT_DEPRESSION_D = JOINT_BUMP_D + 2 * JOINT_BUMP_CLEARANCE_R
JOINT_DEPRESSION_H = JOINT_BUMP_H + JOINT_BUMP_CLEARANCE_Z

# Bump radial extent vs column interior (square hole, edge at r=45 on
# cardinal axes) and flange OD (r ≈ 69). Keep both edges of the bump
# inside the solid overhang band so the bump roots in flange material.
_BUMP_R_INNER = JOINT_BUMP_CENTERLINE_R - JOINT_BUMP_D / 2
_BUMP_R_OUTER = JOINT_BUMP_CENTERLINE_R + JOINT_BUMP_D / 2

# Rotating collar — circular OD on the column at mid-height that the
# bearing on the floor *above* grips (so this floor's bearing engages
# the *lower* column's rotating collar). Sits at z_local = 220..225,
# which falls inside `OPENING_Z_BOT.._TOP ≈ 174..268` — so the collar
# is split by the +X opening into a ~270° partial ring. Mechanically
# fine for low RPM / low load.
ROT_COLLAR_T = COLLAR_T                              # 5.0
ROT_COLLAR_Z_BOT = FLOOR_H                           # 220
ROT_COLLAR_Z_TOP = ROT_COLLAR_Z_BOT + ROT_COLLAR_T   # 225

# Bearing engagement plane on this floor — short ring at z = 0..5,
# uniform across all roles. Engages the lower column's rotating collar
# (z_lower_local 220..225 = z_local 0..5) for middle/top, or the base
# column's own bottom collar (z_base_local 0..5) for the base slot.
BEARING_T = COLLAR_T                                 # 5.0
BEARING_Z_BOT = RAIL_TOP_Z                           # 0
BEARING_Z_TOP = BEARING_Z_BOT + BEARING_T            # 5

LOWER_COL_TOP_Z_MAX = COL_Z_TOP - FLOOR_H            # 53 — joint plane

FLOOR_ROLES = ("base", "middle", "top")


def _col_z_bot(role: str) -> float:
    """Bottom of the rotating column for a given floor role.

    Base stands on the turntable (z = BASE_COL_Z_BOT = 16.5, see
    params). Middle/top sit at the joint plane (z=53); they meet the
    lower column's top flange tangent at that plane. No clearance gap
    — the columns are designed to mate face-to-face under gravity
    preload.
    """
    if role == "base":
        return BASE_COL_Z_BOT
    if role in ("middle", "top"):
        return LOWER_COL_TOP_Z_MAX
    raise ValueError(f"unknown floor role: {role!r} (expected one of {FLOOR_ROLES})")

# Ring bearing
RING_BEARING_ID = COLLAR_OD + 2 * COLLAR_CLEARANCE
RING_BEARING_OD = RING_BEARING_ID + 20.0

# Bearing spider
N_SPIDER_SADDLES = 6
SPIDER_HUB_ID = RING_BEARING_OD + 1.0
SPIDER_HUB_OD = SPIDER_HUB_ID + 10.0

# Base-role hub = the turntable seat: a wider, taller annulus whose top
# face (TURNTABLE_Z_BOT) is flush with the saddle top plates, so the
# turntable's bottom ring lands on one plane. Its bore passes the
# base_floor ring gear's standoff (pitch Ø ≈ COLLAR_OD) with
# SEAT_CLEARANCE radial; the tower root measures that gap.
SEAT_CLEARANCE = 3.0
SEAT_ID = COLLAR_OD + 2 * SEAT_CLEARANCE                   # ≈ 144
SEAT_OD = TURNTABLE_OD + 4.0                               # 204 — 2 mm land outside the ring
SEAT_T = TURNTABLE_Z_BOT                                   # 8

EXT_SIDE = RAIL_SIZE                          # 2020 aluminum extrusion (== frame)

SADDLE_T_TOP = 3.0
SADDLE_T_WALL = 3.0
SADDLE_COVER_H = 12.0
# Saddles sit at azimuth (i+0.5)·(360°/N_SPIDER_SADDLES) — along basket
# boundaries = inner-hex vertex direction = radial-rail axis. Each
# saddle spans from the hub outward past RAIL_INNER_R to grip
# SADDLE_GRIP_LEN of rail at its outer end. The cantilever section
# (hub → rail inner end) is a U-channel in cross-section because the
# extrusion cutout runs the full saddle length; upgrading that to a
# solid bridge is a structural refinement for a later pass.
SADDLE_GRIP_LEN = 20.0
SADDLE_L = RAIL_INNER_R + SADDLE_GRIP_LEN - SPIDER_HUB_OD / 2
SADDLE_HUB_OVERLAP = 2.0
SADDLE_BOLT_D = 4.5
SADDLE_BOLT_OFFSET = RAIL_INNER_R + SADDLE_GRIP_LEN / 2 - SPIDER_HUB_OD / 2

WALL_MIN = 1.5


# --- Helpers ------------------------------------------------------------

def _cyl_along_y(r: float, length: float) -> Part:
    return Rot(90, 0, 0) * Cylinder(r, length)


# --- Part functions -----------------------------------------------------

def bearing_spider(role: str = "middle") -> Part:
    """Printed hub that holds the ring bearing coaxial with the column.
    Six integrated saddles grip the tops of radial 2020 aluminum
    extrusions; the extrusions carry the span out to the inner hex.

    The six saddles are **uniform across all roles**; the hub is not:

    - middle / top — a short annular ring at z = 0..BEARING_T (5 mm)
      holding the printed radial ring bearing, which engages the
      *lower* column's rotating collar (mapped from z_lower_local
      220..225).
    - base — the turntable **seat**: annulus SEAT_ID..SEAT_OD at
      z = 0..SEAT_T, top face flush with the saddle top plates, on
      which the bought turntable (`ring_bearing("base")`) sits. The
      base_floor ring gear's standoff passes down through its bore.
    """
    if role == "base":
        # The turntable must land on the seat, not on the saddle plates
        # poking up through it; and the seat must cover the turntable's
        # bottom ring. Parameter bounds over a bought part's footprint —
        # bare, fired here where the seat is cut.
        assert SEAT_T >= BEARING_Z_TOP + SADDLE_T_TOP, (
            f"Seat top z={SEAT_T} sits below the saddle top plates "
            f"(z={BEARING_Z_TOP + SADDLE_T_TOP}); the turntable would rock "
            f"on six pads. Raise TURNTABLE_Z_BOT."
        )
        assert SEAT_ID <= TURNTABLE_ID and SEAT_OD >= TURNTABLE_OD, (
            f"Seat annulus Ø{SEAT_ID:.1f}..Ø{SEAT_OD:.1f} does not cover the "
            f"turntable's Ø{TURNTABLE_ID}..Ø{TURNTABLE_OD} ring."
        )
        hub_id, hub_od, hub_z_top = SEAT_ID, SEAT_OD, SEAT_T
    else:
        hub_id, hub_od, hub_z_top = SPIDER_HUB_ID, SPIDER_HUB_OD, BEARING_Z_TOP
    hub_z_bot = BEARING_Z_BOT
    hub_t = hub_z_top - hub_z_bot
    hub_z_center = (hub_z_bot + hub_z_top) / 2
    ext_top_z = BEARING_Z_TOP                          # saddle clamps just above rail

    hub_outer = Pos(0, 0, hub_z_center) * Cylinder(hub_od / 2, hub_t)
    hub_inner = Pos(0, 0, hub_z_center) * Cylinder(hub_id / 2, hub_t * 2)
    body = hub_outer - hub_inner

    saddle_w = EXT_SIDE + 2 * SADDLE_T_WALL
    saddle_h = SADDLE_COVER_H + SADDLE_T_TOP
    saddle_x_in = SPIDER_HUB_OD / 2 - SADDLE_HUB_OVERLAP
    saddle_x_out = SPIDER_HUB_OD / 2 + SADDLE_L
    saddle_z_bot = ext_top_z - SADDLE_COVER_H
    saddle_z_top = ext_top_z + SADDLE_T_TOP

    cutout_x_in = SPIDER_HUB_OD / 2
    cutout_x_out = saddle_x_out + 5.0
    cutout_z_top = ext_top_z
    cutout_z_bot = saddle_z_bot - 1.0

    bolt_x = SPIDER_HUB_OD / 2 + SADDLE_BOLT_OFFSET
    bolt_z_bot = ext_top_z - 1.0
    bolt_z_top = saddle_z_top + 1.0

    # Saddles at (i+0.5) * step = basket boundaries / inner-hex vertex
    # azimuths, aligned with the radial rails they grip.
    for i in range(N_SPIDER_SADDLES):
        theta = (i + 0.5) * 360.0 / N_SPIDER_SADDLES
        rot = Rot(0, 0, theta)
        saddle_env = rot * Pos(
            (saddle_x_in + saddle_x_out) / 2, 0,
            (saddle_z_bot + saddle_z_top) / 2,
        ) * Box(saddle_x_out - saddle_x_in, saddle_w, saddle_h)
        cutout = rot * Pos(
            (cutout_x_in + cutout_x_out) / 2, 0,
            (cutout_z_bot + cutout_z_top) / 2,
        ) * Box(cutout_x_out - cutout_x_in, EXT_SIDE, cutout_z_top - cutout_z_bot)
        bolt_hole = rot * Pos(bolt_x, 0, (bolt_z_bot + bolt_z_top) / 2) * Cylinder(
            SADDLE_BOLT_D / 2, bolt_z_top - bolt_z_bot,
        )
        body = body + saddle_env - cutout - bolt_hole
    return body


def ring_bearing(role: str = "middle") -> Part:
    """The floor's bearing ring.

    - middle / top — printed radial-only annulus at this floor's
      bracket level (z = 0..BEARING_T), see `bearing_spider` for the
      engagement story.
    - base — **bought turntable** stub (TURNTABLE_OD / ID / H from
      params, the same part as m02's rim drive): one annulus with its
      bottom face on the seat at TURNTABLE_Z_BOT. Carries the stack's
      axial load and its radial location. Not inspected — not printed.
    """
    if role == "base":
        z_center = (TURNTABLE_Z_BOT + TURNTABLE_Z_TOP) / 2
        h = TURNTABLE_Z_TOP - TURNTABLE_Z_BOT
        body = Pos(0, 0, z_center) * Cylinder(TURNTABLE_OD / 2, h)
        hole = Pos(0, 0, z_center) * Cylinder(TURNTABLE_ID / 2, h * 2)
        return body - hole
    z_center = (BEARING_Z_BOT + BEARING_Z_TOP) / 2
    body = Pos(0, 0, z_center) * Cylinder(RING_BEARING_OD / 2, BEARING_T)
    hole = Pos(0, 0, z_center) * Cylinder(RING_BEARING_ID / 2, BEARING_T * 2)
    return body - hole


def _circular_feature(z_bot: float, z_top: float, od: float = COLLAR_OD) -> Part:
    """Annular feature on the column — circular OD (default COLLAR_OD)
    with the column interior (square hole) removed. Used for the
    rotating collar, the top joint flange, and the base column's
    bottom flange (which takes the wider BASE_FLANGE_OD).
    """
    h = z_top - z_bot
    z_center = (z_top + z_bot) / 2
    disc = Pos(0, 0, z_center) * Cylinder(od / 2, h)
    hole = Pos(0, 0, z_center) * Box(
        COL_INTERIOR_SIDE, COL_INTERIOR_SIDE, h * 2
    )
    return disc - hole


def _joint_interlock(z_face: float, d: float, h: float) -> Part:
    """`N_JOINT_BUMPS` cylindrical features at cardinal azimuths,
    centered on `JOINT_BUMP_CENTERLINE_R`, occupying z ∈ [z_face,
    z_face + h]. Used twice per joint: unioned onto the lower
    column's top flange as bumps; subtracted from the upper column's
    bottom flange as depression voids (which under +Z stacking are
    the same +Z-extending pocket above the mating face).
    """
    features = None
    for i in range(N_JOINT_BUMPS):
        theta = i * 360.0 / N_JOINT_BUMPS
        feature = Rot(0, 0, theta) * Pos(
            JOINT_BUMP_CENTERLINE_R, 0, z_face + h / 2,
        ) * Cylinder(d / 2, h)
        features = feature if features is None else features + feature
    return features


def column(role: str = "middle") -> Part:
    """Square-section tube with a +X exit opening, −X hinge bosses, a
    rotating collar at mid-height (the bearing-engagement surface for
    the floor *above*), a top joint flange, and — for the base role
    only — a bottom collar where the motor pinion meets the ring gear.

    Roles differ only in what sits at the column bottom:
    - **base**: body 0..273, bottom collar at 0..5 (ring-gear interface).
    - **middle**, **top**: body 53..273, mating face at z=53.
    """
    # Packing sanity for the cut features this part carries. All of them
    # are parameter bounds over subtracted negative space (the opening,
    # the boss footprint, the bas-relief bump ring, the collar band) —
    # no part pair to measure, so they stay bare asserts and fire here,
    # when a column is actually cut, rather than at import.
    assert BOSS_Y_OFFSET + BOSS_Y / 2 <= COL_INTERIOR_SIDE / 2, (
        f"Hinge bosses at |Y| = {BOSS_Y_OFFSET + BOSS_Y/2:.1f} mm exceed column "
        f"interior half-width {COL_INTERIOR_SIDE/2:.1f} mm. Reduce TRAP_W."
    )
    assert _RAMP_WALL_TOP_RISE <= MAX_BRICK_DIM, (
        f"Ramp side-wall top rises {_RAMP_WALL_TOP_RISE:.1f} mm above the "
        f"plate centreline at the column wall, past the {MAX_BRICK_DIM:.0f} mm "
        f"brick envelope the opening is cut for. Reduce RAMP_WALL_H."
    )
    assert COL_Z_TOP >= OPENING_Z_TOP + JOINT_ZONE_CLEARANCE, (
        f"COL_Z_TOP ({COL_Z_TOP}) too low — opening top "
        f"({OPENING_Z_TOP:.1f}) reaches inside the joint zone."
    )
    assert _BUMP_R_INNER >= COL_INTERIOR_SIDE / 2 + 2.0, (
        f"Joint bump inner edge at r={_BUMP_R_INNER:.1f} crowds the column "
        f"interior boundary at r={COL_INTERIOR_SIDE/2:.1f}; bump would not "
        f"root in solid flange material on the cardinal axes."
    )
    assert _BUMP_R_OUTER + JOINT_BUMP_CLEARANCE_R <= COLLAR_OD / 2 - 2.0, (
        f"Joint bump+depression outer edge at r={_BUMP_R_OUTER + JOINT_BUMP_CLEARANCE_R:.1f} "
        f"crowds the flange OD at r={COLLAR_OD/2:.1f}; leave ≥2 mm rim."
    )
    assert OPENING_Z_BOT <= ROT_COLLAR_Z_BOT and ROT_COLLAR_Z_TOP <= OPENING_Z_TOP, (
        f"Rotating collar at z={ROT_COLLAR_Z_BOT}..{ROT_COLLAR_Z_TOP} not "
        f"inside opening z={OPENING_Z_BOT:.1f}..{OPENING_Z_TOP:.1f}; the "
        f"partial-ring split assumption no longer holds."
    )

    z_bot = _col_z_bot(role)
    col_h = COL_Z_TOP - z_bot
    col_z_center = (COL_Z_TOP + z_bot) / 2

    outer = Pos(0, 0, col_z_center) * Box(COL_OUTER_SIDE, COL_OUTER_SIDE, col_h)
    inner = Pos(0, 0, col_z_center) * Box(
        COL_INTERIOR_SIDE, COL_INTERIOR_SIDE, col_h * 2
    )
    tube = outer - inner

    # Rotating collar — circular OD at z 220..225 that the bearing on
    # the floor *above* grips. Added BEFORE the +X opening cut so the
    # opening also slices the collar into a partial ring.
    tube = tube + _circular_feature(ROT_COLLAR_Z_BOT, ROT_COLLAR_Z_TOP)

    # Top joint flange + bas-relief bumps. The disc carries 4
    # cylindrical bumps at cardinal azimuths protruding +JOINT_BUMP_H
    # above the flange face; they slot into matching depressions on
    # the upper column's bottom flange to transfer torque across the
    # joint without bolts or metal hardware.
    tube = tube + _circular_feature(JOINT_FLANGE_Z_BOT, JOINT_FLANGE_Z_TOP)
    tube = tube + _joint_interlock(
        JOINT_FLANGE_Z_TOP, JOINT_BUMP_D, JOINT_BUMP_H,
    )

    # Role-specific bottom feature.
    if role == "base":
        # Bottom flange at z 16.5..21.5, Ø BASE_FLANGE_OD — sits on the
        # turntable's top ring; the base_floor ring gear bolts to its
        # underside. No bas-relief here — the base column has no lower
        # mate.
        tube = tube + _circular_feature(
            BASE_COL_Z_BOT, BASE_COL_Z_BOT + COLLAR_T, od=BASE_FLANGE_OD,
        )
    else:
        # Middle/top: bottom face flange mating with the lower column's
        # top flange at z = LOWER_COL_TOP_Z_MAX (= 53). Same OD; the
        # two flanges contact face-to-face at the joint plane. The
        # mating face carries depression voids that receive the lower
        # column's bumps.
        tube = tube + _circular_feature(z_bot, z_bot + JOINT_FLANGE_T)
        tube = tube - _joint_interlock(
            z_bot, JOINT_DEPRESSION_D, JOINT_DEPRESSION_H,
        )

    # Exit opening on +X wall — cuts the tube AND any circular feature
    # that overlaps its z range (rotating collar at 220..225 ⊂ ≈174..268).
    cut_z_center = (OPENING_Z_TOP + OPENING_Z_BOT) / 2
    cut_h = OPENING_Z_TOP - OPENING_Z_BOT
    cut = Pos(COL_OUTER_SIDE, 0, cut_z_center) * Box(
        COL_OUTER_SIDE * 2, OPENING_W, cut_h
    )
    tube = tube - cut

    # Hinge bosses — inside surface of −X wall, ±Y of the trap, at
    # TRAP_HINGE_Z. They extend +X (inward) to anchor the pin, which
    # runs through both ±Y walls; the servo drives its +Y end.
    boss_plus = Pos(BOSS_X_CENTER, +BOSS_Y_OFFSET, TRAP_HINGE_Z) * Box(
        BOSS_X, BOSS_Y, BOSS_H_Z
    )
    boss_minus = Pos(BOSS_X_CENTER, -BOSS_Y_OFFSET, TRAP_HINGE_Z) * Box(
        BOSS_X, BOSS_Y, BOSS_H_Z
    )
    tube = tube + boss_plus + boss_minus

    # Pin bore through bosses (and the air between)
    pin_bore = Pos(TRAP_HINGE_X, 0, TRAP_HINGE_Z) * _cyl_along_y(
        PIN_HOLE_D / 2, PIN_LEN + 4.0
    )
    return tube - pin_bore


def hinge_pin() -> Part:
    """Hinge pin in canonical local frame (axis along +Z). The +Y
    orientation it needs in the assembly is composed into
    ``hinge_pin_loc``, not the part itself — see ``ramp()``'s
    docstring for why this matters for animated GLB exports.
    """
    return Cylinder(PIN_D / 2, PIN_LEN)


def trap() -> Part:
    """Flat plate hinged at the −X interior wall. In the local frame the
    hinge knuckle is at origin and the plate body extends in +X to
    x = TRAP_L + KNUCKLE_OD/2. Placed at θ = 90° it hangs (rest); at
    θ = TRAP_CATCH_DEG it catches, free end down toward the exit.
    """
    plate_center_x = TRAP_L / 2 + KNUCKLE_OD / 2
    plate = Pos(plate_center_x, 0, 0) * Box(TRAP_L, TRAP_W, TRAP_T)
    knuckle = _cyl_along_y(KNUCKLE_OD / 2, TRAP_W)
    body = plate + knuckle
    pin_hole = _cyl_along_y(PIN_HOLE_D / 2, TRAP_W * 2)
    return body - pin_hole


def ramp() -> Part:
    """Fixed delivery plate. Returned **flat and centered at origin**
    (identity part-local Location). The full pose — translation to the
    ramp root, the downward tilt at ``RAMP_DEG``, and the
    near-edge-at-origin shift of ``+RAMP_L/2`` — is composed into
    ``ramp_loc`` (assembly-level placement), not the part itself.

    Keeping the pose in the placement is what lets animated GLB
    exporters pick it up via per-frame TRS samplers, which only see
    ``PlacedPart.location`` and silently drop part-internal Locations.

    Two side walls (``RAMP_WALL_T`` × ``RAMP_WALL_H`` above the plate)
    stand outboard of the ``RAMP_W`` channel along the full length,
    bottoms flush with the plate underside; the part's outer width is
    ``RAMP_OUTER_W``. One printed body, plate down.
    """
    # Deliberately loose upper bound (see BASKET_SLOT_W_AT_TIP above —
    # the real slot is narrower, and the tight cross-boundary check is
    # the top level's basket-vs-ramp no-interference). Stays bare.
    assert RAMP_OUTER_W <= BASKET_SLOT_W_AT_TIP, (
        f"RAMP_OUTER_W ({RAMP_OUTER_W:.1f} mm) > basket angular slot upper "
        f"bound at ramp tip ({BASKET_SLOT_W_AT_TIP:.1f} mm at "
        f"X={RAMP_TIP_X:.1f}, R_BASKET_INNER={R_BASKET_INNER:.1f}). Reduce "
        f"RAMP_W / RAMP_WALL_T, increase R_BASKET_INNER, or decrease "
        f"N_BASKETS_PER_FLOOR."
    )
    plate = Box(RAMP_L, RAMP_W, RAMP_T)
    wall_y = RAMP_W / 2 + RAMP_WALL_T / 2
    wall_h = RAMP_T + RAMP_WALL_H
    wall_z = RAMP_WALL_H / 2                  # bottom flush with the plate underside
    wall = Box(RAMP_L, RAMP_WALL_T, wall_h)
    return plate + Pos(0, +wall_y, wall_z) * wall + Pos(0, -wall_y, wall_z) * wall


def trap_servo() -> Part:
    """Bought micro-servo envelope in canonical local frame: output
    shaft on +Z at the origin (the shaft face is z = 0, the case below
    it), long axis along +X with the shaft ``SERVO_SHAFT_FROM_END`` in
    from the −X end, mounting tabs across the full ``SERVO_TAB_L``. The
    pose that turns the shaft onto the hinge axis lives in
    ``servo_loc``. Not printed, so not inspected.
    """
    x_mid = SERVO_BODY_L / 2 - SERVO_SHAFT_FROM_END
    body = Pos(x_mid, 0, -SERVO_BODY_H / 2) * Box(
        SERVO_BODY_L, SERVO_BODY_W, SERVO_BODY_H
    )
    tabs = Pos(x_mid, 0, -SERVO_TAB_FROM_TOP - SERVO_TAB_T / 2) * Box(
        SERVO_TAB_L, SERVO_BODY_W, SERVO_TAB_T
    )
    return body + tabs


# --- Assembly -----------------------------------------------------------

hinge_loc = Location((TRAP_HINGE_X, 0, TRAP_HINGE_Z))
# The hinge pin needs its axis along +Y (the canonical hinge direction
# in the assembly). The rotation lives here in the placement, not in
# `hinge_pin()`, so per-frame TRS samplers carry it.
hinge_pin_loc = hinge_loc * Rot(90, 0, 0)
trap_rest_loc = hinge_loc * Rot(0, TRAP_REST_DEG, 0)
trap_catch_loc = hinge_loc * Rot(0, TRAP_CATCH_DEG, 0)
# Shaft face SERVO_STANDOFF outside the +Y wall, shaft pointing −Y
# into the hinge: Rot(90, 0, 0) turns local +Z onto −Y and local +Y
# onto +Z, leaving the long axis on +X.
servo_loc = Location(
    (TRAP_HINGE_X, COL_OUTER_SIDE / 2 + SERVO_STANDOFF, TRAP_HINGE_Z)
) * Rot(90, 0, 0)
ramp_loc = (
    Location((RAMP_ROOT_X, 0, RAMP_ROOT_Z))
    * Rot(0, RAMP_DEG, 0)
    * Pos(RAMP_L / 2, 0, 0)
)
# Lower-floor stand-ins (middle/top roles only; base has no floor below).
# Both are mapped into this floor's frame via -FLOOR_H.
# - lower_trap_servo: the floor below's servo, the only lower-floor
#   rotating part that reaches above that floor's joint plane outside
#   the column — at z ≈ 13..25 here, radially past the collar, the
#   band this floor's bearing spider occupies.
# - lower_column: the floor below's column body. Its top reaches
#   z = LOWER_COL_TOP_Z_MAX (= 53) and its rotating collar sits inside
#   this floor's bearing at z = 0..5 (engagement surface).
lower_column_loc = Location((0, 0, -FLOOR_H))
lower_servo_loc = lower_column_loc * servo_loc


def _servo_hold_margin() -> float:
    """Stall torque over the static torque at the catch tilt: the plate
    (solid PLA, lever at its centroid) plus BRICK_MASS_BUDGET_G at the
    free end. Hinge knuckle mass ignored (on the axis)."""
    g = 9.81e-3                                   # N per g
    plate_g = TRAP_L * TRAP_W * TRAP_T * PLA_DENSITY_G_PER_MM3
    arm_mm = (KNUCKLE_OD / 2 + TRAP_L / 2) * math.cos(_TRAP_CATCH_RAD)
    tip_mm = _TRAP_REACH * math.cos(_TRAP_CATCH_RAD)
    load_nm = g * (plate_g * arm_mm + BRICK_MASS_BUDGET_G * tip_mm) / 1000.0
    return SERVO_STALL_NM / load_nm


# --- Sub-assembly builders ----------------------------------------------

def build_column_side_static(
    role: str = "middle",
    material_fn: Callable[[str], str | None] | None = None,
) -> Assembly:
    """Stationary column-side parts — bearing spider + ring bearing —
    as a sub-`Assembly` in the floor-local frame (bearing base at
    z=0). The parent places the whole floor with one transform; floor
    identity lives in the parent's subtree path, so the leaf names
    here are plain. ``material_fn`` follows the same contract as the
    frame builders (leaf name in, attribute out; ``None`` leaves the
    attribute unset). Colour is not a parameter here — the unit root
    paints the finished tree (``cad/design_systems.py:paint``).
    """
    def _mt(name: str) -> str | None:
        return material_fn(name) if material_fn is not None else None

    n_sp = "bearing_spider"
    n_rb = "ring_bearing"
    a = (
        Assembly()
        .with_part(n_sp, bearing_spider(role), material=_mt(n_sp))
        .with_part(n_rb, ring_bearing(role), material=_mt(n_rb))
    )
    if role == "base":
        # The turntable rests on the seat — that rest is the stack's
        # whole axial load path, and no-interference alone would pass
        # with the turntable floating above six saddle pads.
        a = a.assert_tangent_contact(n_rb, n_sp, name="turntable_on_seat")
    return a


def build_column_side_rotating(
    role: str = "middle",
    *,
    material_fn: Callable[[str], str | None] | None = None,
) -> Assembly:
    """Rotating column-side parts — column + hinge pin + ramp + trap
    (at rest, hanging) + trap servo — as a sub-`Assembly` in the
    floor-local frame. One of each physical part: the catch pose and
    the swing samples are standalone-only (``make_assembly``), since
    the trap stays inside its own column at every pose. These parts
    revolve together with the column stack; the revolute joint that
    carries the azimuth lives on the consumer's rotating-stack
    subtree, not here.
    """
    def _mt(name: str) -> str | None:
        return material_fn(name) if material_fn is not None else None

    def _add(a: Assembly, name: str, part, **kwargs) -> Assembly:
        return a.with_part(name, part, material=_mt(name), **kwargs)

    a = Assembly()
    a = _add(a, "column", column(role))
    a = _add(a, "hinge_pin", hinge_pin(), location=hinge_pin_loc)
    a = _add(a, "ramp", ramp(), location=ramp_loc)
    a = _add(a, "trap", trap(), location=trap_rest_loc)
    a = _add(a, "trap_servo", trap_servo(), location=servo_loc)

    # Declared once here, where the ramp lives; standalone runs
    # evaluate directly and composed runs (single_floor, tower, cad
    # top-level) evaluate the propagated, placement-qualified form.
    #
    # Sweep clearance: the ramp rotates with the column over every
    # basket's radial side walls (standing at BASKET_RIM_Z). The datum
    # plane is Z-invariant under column rotation, so this one directed
    # distance covers all azimuths with no sweep — and it measures the
    # real lowest point of the ramp body, not a hand-derived corner Z.
    a = a.assert_distance(
        "ramp", Plane.XY.offset(BASKET_RIM_Z), along="-Z",
        min_mm=RAMP_SWEEP_CLEARANCE, name="ramp_sweep_over_basket_rim",
    )
    # Slide conditions — recorded scalar claims (see MU_STATIC_BUDGET).
    a = a.assert_scalar(
        "ramp_slide_margin", math.tan(_RAMP_DEG_RAD),
        ge=MU_STATIC_BUDGET, detail="µ_s budget, ABS brick on PLA ramp",
    )
    a = a.assert_scalar(
        "trap_slide_margin", math.tan(_TRAP_CATCH_RAD),
        ge=MU_STATIC_BUDGET, detail="µ_s budget, ABS brick on PLA trap at catch tilt",
    )
    # The servo is the only thing holding the catch pose.
    a = a.assert_scalar(
        "servo_static_hold_margin", _servo_hold_margin(),
        ge=SERVO_HOLD_MARGIN_MIN,
        detail=f"stall / static torque at catch: solid-PLA plate + "
               f"{BRICK_MASS_BUDGET_G:.0f} g brick at the tip; impact not included",
    )
    return a


def make_assembly(role: str = "middle") -> Assembly:
    """Build the standalone single-floor assembly for the given role.

    `role` ∈ FLOOR_ROLES. Middle and top are geometrically identical
    here (both see the same lower-floor column + servo stand-ins);
    base omits the lower-floor stand-ins and includes its bottom
    collar instead.

    Composes the same builders the tower consumes: static parts at
    the root (``build_column_side_static``), the revolving parts as
    the ``rotating`` subtree (``build_column_side_rotating``), plus
    standalone-only stand-ins, the trap's catch pose and its swing
    samples at the root. This is the only place the catch pose and
    the swing are checked — every role, via ``role_sweep.py``.
    """
    a = build_column_side_static(role)
    a = a.with_subassembly("rotating", build_column_side_rotating(role))
    if role != "base":
        # Lower floor's column placed at -FLOOR_H. Modeled as `column("middle")`
        # for both middle (lower=base|middle) and top (lower=middle); the
        # difference between base and middle lower columns lives at
        # z_lower_local = 16.5..21.5 (the base flange), which maps to
        # z_local ≈ -203 — far below anything modeled in this frame. Using "middle" as the generic stand-in keeps this
        # assembly role-independent of which floor the lower one is.
        a = (
            a
            .with_part("lower_column", column("middle"), location=lower_column_loc)
            .with_part("lower_trap_servo", trap_servo(), location=lower_servo_loc)
        )
    a = a.with_part("trap_catching", trap(), location=trap_catch_loc)
    for angle in TRAP_SAMPLES:
        a = a.with_part(
            f"trap_at_{int(angle)}", trap(),
            location=hinge_loc * Rot(0, angle, 0),
        )

    rotating_fixed = (
        "rotating.column", "rotating.ramp", "rotating.trap_servo",
        "bearing_spider", "ring_bearing",
    )
    a = (
        a
        # Fixed parts don't interfere with each other
        .assert_no_interference("bearing_spider", "ring_bearing")
        .assert_no_interference("bearing_spider", "rotating.column")
        .assert_no_interference("ring_bearing", "rotating.column")
        .assert_no_interference("rotating.hinge_pin", "rotating.column")
        .assert_no_interference("rotating.ramp", "rotating.column")
        .assert_no_interference("rotating.ramp", "bearing_spider")
        .assert_no_interference("rotating.ramp", "ring_bearing")
        .assert_no_interference("rotating.ramp", "rotating.hinge_pin")
        # Servo outside the +Y wall: clear of the column (and its
        # collar), the pin end it drives, and everything else.
        .assert_no_interference("rotating.trap_servo", "rotating.column")
        .assert_no_interference("rotating.trap_servo", "rotating.hinge_pin")
        .assert_no_interference("rotating.trap_servo", "rotating.ramp")
        .assert_no_interference("rotating.trap_servo", "bearing_spider")
        .assert_no_interference("rotating.trap_servo", "ring_bearing")
        # The catching trap hands the brick down a short step onto the
        # ramp root: clear of the ramp, but not so far that the brick
        # tumbles. Constructed at TRAP_HANDOFF_STEP vertically; the
        # true (shortest) distance is a little less, so the bounds
        # bracket it rather than pin it.
        .assert_distance(
            "trap_catching", "rotating.ramp",
            min_mm=2.0, max_mm=TRAP_HANDOFF_STEP + 1.0,
            name="trap_catch_to_ramp_step",
        )
    )
    for name in ("rotating.trap", "trap_catching",
                 *(f"trap_at_{int(t)}" for t in TRAP_SAMPLES)):
        for other in rotating_fixed:
            a = a.assert_no_interference(name, other)
    if role == "base":
        # The column's bottom flange rests on the turntable's top ring —
        # the other half of the axial load path (seat contact is
        # declared in `build_column_side_static`). Composed runs
        # (tower) declare the same rest across their subtrees.
        a = a.assert_tangent_contact(
            "ring_bearing", "rotating.column", name="column_on_turntable",
        )
    if role != "base":
        a = (
            a
            # Bearing/spider engage the LOWER column's rotating collar
            # via grease film (no interference). Spider hub and lower-
            # column body must also clear (spider sits radially outside
            # the bearing, far from the column).
            .assert_no_interference("bearing_spider", "lower_column")
            .assert_no_interference("ring_bearing", "lower_column")
            # This floor's column mates face-to-face with the lower
            # column at z=53 (the joint plane). No-interference allows
            # that contact but would equally allow a floating column,
            # so the contact itself is the separate claim below.
            .assert_no_interference("rotating.column", "lower_column")
            # Ramp / trap / hinge live well above the joint plane,
            # so they're clear of the lower column entirely.
            .assert_no_interference("rotating.ramp", "lower_column")
            .assert_no_interference("rotating.hinge_pin", "lower_column")
            .assert_no_interference("rotating.trap", "lower_column")
            # Lower-floor servo — must clear THIS floor's column-bottom
            # stack (bearing / spider / column).
            .assert_no_interference("lower_trap_servo", "rotating.column")
            .assert_no_interference("lower_trap_servo", "bearing_spider")
            .assert_no_interference("lower_trap_servo", "ring_bearing")
            # The joint-plane rest itself: bottom flange down on the
            # lower column's top flange, no clearance gap, bumps in
            # their depressions (see `_col_z_bot`). The tower declares
            # the same rest between real adjacent floors; this one
            # covers the stand-in the standalone builds against.
            .assert_tangent_contact("rotating.column", "lower_column")
        )
    return a


# Module-level "default" assembly — preserved so the top-level
# `m05_diverter/assembly.py` and other consumers can import a ready-
# built middle-floor assembly without going through `make_assembly`.
assembly = make_assembly("middle")
