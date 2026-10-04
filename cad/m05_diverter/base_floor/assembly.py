"""
M05 hex-tower — base-floor drive train (off-axis pinion + ring gear).

Stationary structure that drives the rotating column from below.
Base-floor-only: floors 2-4 use only the column-bottom stack from
ramp_mechanism (spider / ring bearing / column).

Realizes the "single motor at the base, off-axis, gear-coupled to a
ring gear on the base column's bottom collar" architectural commitment.
The brick fall path runs straight down the column axis (90×90 mm
interior), so **no central drive shaft can live there** — the obsolete
coaxial `drive_shaft` from the previous pass is gone. Drive is taken
to the column's periphery instead:

- motor — NEMA-17-style 42×42×40 box stub, mounted **off-axis** on a
  back-side radial-rail azimuth (away from the +X exit/ramp), body
  hanging below the rail datum with its output shaft pointing up.
  Final NEMA size + gearbox choice still open.
- pinion — small gear on the motor output shaft, meshing with the ring
  gear. Modeled cheap as its pitch cylinder (no teeth); the real
  involute profile is a detail-layer concern.
- ring_gear — printed annulus bolted up to the underside of the base
  column's bottom flange (z = BASE_COL_Z_BOT, above the turntable), teeth
  on its OD below the rails; the standoff between drops through the
  turntable's and seat's bores. Modeled cheap as a pitch
  annulus with the same square brick-pass-through hole as the collar.
  The "separate bolted-on ring vs teeth integral to the printed collar
  OD" choice (see `_notes/state/m05-diverter.md`) is resolved here as the
  bolt-on ring: it keeps the gear off the structural column and
  swappable if the ratio changes, and leaves ramp_mechanism's
  `column("base")` untouched. Not m02's integral ring on the flange with
  the motor on a rail above the floor: that moves the motor and the mesh
  for nothing the checker can see.
- motor_mount — printed radial arm at the rail-bottom plane; bolts up
  into one radial rail's T-slot and carries the motor below via the
  NEMA face. Detailed bolt pattern + flange interface still open. Its
  inner end is a chamfer — the deflector over the motor (see
  "Fall path" below).

**Fall path and the bottom catch (2026-10-04).** Every trap rests
hanging, so on power loss, a dead servo or a missed command a brick
falls past every floor and out through the ring gear's square hole
(bottom z = RING_GEAR_Z_BOT). The column and ring gear turn, so across
every column azimuth that exit sweeps a disc of r = FALL_SWEEP_R
(90·√2/2 ≈ 63.64) — exactly the union of the square over all
rotations, so a disc keep-out covers every azimuth with no sampling.
The motor's inner face (r = CENTER_DIST − MOTOR_BODY_W/2 ≈ 60.8) sits
inside that disc, and neither the motor's rail azimuth nor the centre
distance may move, so the mount is shaped to shield it: its flat top
stops outside the disc (DEFLECTOR_R_TOP) and its inner end slopes down
past the motor's face (DEFLECTOR_R_BOT), so a brick that lands there
slides off inward instead of resting 3 mm under the turning ring gear.
Below the motor a small tray (`catch_tray`) on a three-post cradle
(`catch_cradle`, hung from the free inner ends of three radial rails)
takes everything — the fault case, and parts wider than any floor's
ramp. It draws out toward −Y, a side nothing blocks below the motor.
`build_drive_train` declares the vertical fall as keep-outs;
`build_bottom_catch` declares the scatter cone at its mouth and its
draw-out path; the tower root holds the frame against both.

Gear geometry is the cheap two-fidelity-tier model: pitch cylinders,
no teeth. Mesh validity is a min+max `assert_distance` band ("close
but not touching") rather than a volume check, since meshing gears
are meant to touch. The pinion-tip-clears-the-rail-ring margin is the
tight constraint; it needs the rails, so it lives at the tower root
as a tip-circle (`grow_a_mm`) assert_distance.

The base bearing (axial + radial + motor-reaction torque) is the bought
8" turntable m02 also uses, built as `ring_bearing("base")` in
ramp_mechanism (params.py `TURNTABLE_*`); the middle and top
floors keep the printed ring, which carries no axial load.

Coordinate frame (matches ramp_mechanism / params):
    origin = column axis ∩ rail-top datum (z=0)
    +Z     = up
    +X     = radial outward toward the exit opening (azimuth 0°)
    z=0    = rail top face (frame datum) = turntable seat bottom
    z=16.5 = BASE_COL_Z_BOT — base flange underside, ring-gear standoff top
    z<0    = below the floor — gear mesh, pinion, mount, motor sit here
"""

import math
from typing import Callable


from build123d import Axis, Box, Cone, Cylinder, Part, Plane, Polygon, Pos, Rot, extrude

from cad_khana.mechanism.assembly import Assembly

from ..params import (
    BASE_COL_Z_BOT, COL_INTERIOR_SIDE, COLLAR_OD, RAIL_INNER_R, RAIL_SIZE,
)


# --- Gear train ---------------------------------------------------------
#
# Spur pair: a small pinion on the off-axis motor shaft drives a large
# ring gear bolted to the base column's bottom collar. The ring's pitch
# diameter tracks the collar OD so the gear stays compact on the collar.
MODULE = 1.5                          # printable spur module (low torque)
PRESSURE_ANGLE_DEG = 20.0             # documentation only — cheap model has no teeth
RING_TEETH = round(COLLAR_OD / MODULE)            # 92 — pitch D ≈ collar OD
# 17 T is the undercut floor for a 20° PA involute — and the tooth count
# m02's rim-drive pinion settled on (`_notes/decisions/m02-rotor-drive.md`),
# so the two modules share one printed pinion. The tip lands at r ≈ 96.2,
# which is what pushed RAIL_INNER_R out to 100 (params.py).
PINION_TEETH = 17
RING_PITCH_D = RING_TEETH * MODULE                 # 138.0
PINION_PITCH_D = PINION_TEETH * MODULE             # 18.0
GEAR_RATIO = RING_TEETH / PINION_TEETH             # ≈ 5.41 : 1
ADDENDUM = MODULE                                  # tip = pitch + module
MESH_BACKLASH = 0.2                   # extra center distance for FDM mesh play

# Center distance = sum of pitch radii + a touch of backlash. The cheap
# pitch cylinders then sit MESH_BACKLASH apart (no overlap) so the
# pinion-vs-ring no-interference is robust; the `gear_mesh_backlash`
# assert_distance in `build_drive_train` measures the placed gap and
# is the authoritative mesh check.
CENTER_DIST = (RING_PITCH_D + PINION_PITCH_D) / 2 + MESH_BACKLASH   # 81.95

# Pitch ≈ collar OD: the ring gear must seat on the collar's bolt face.
# Declared as a `ring_gear_collar_seat` assert_scalar in
# `build_drive_train` — the collar is not a part of this sub-assembly,
# so the claim is a recorded scalar rather than a part-pair distance.

# Tight constraint: the pinion's tip circle must stay inside the radial-
# rail ring (rails start at RAIL_INNER_R). This is the packaging knife-
# edge — the pinion is squeezed between the Ø138 collar and the Ø200
# rail-inner ring. The rails are not parts of this sub-assembly, so
# the claim lives at the tower root (`.._add_drive_train_checks`) as a
# `grow_a_mm=ADDENDUM` assert_distance measuring from the tip circle
# while the modeled body stays a pitch cylinder. If it fails: shrink
# PINION_TEETH or push the rails out.


# --- Z-band layout (top → bottom) ---------------------------------------
#
# The gear mesh is dropped BELOW the ramp_mechanism bearing spider. That
# spider's saddles clamp the radial rails at the same azimuths a
# rail-mounted motor wants, and they dip to z ≈ -7 reaching in to
# r ≈ 83.5 — straight through the pinion's tip path. The ring gear
# itself (r ≤ 69) always clears the saddle radially, but the pinion
# (tip r ≈ 96.2) does not, so the whole mesh plane sits under the saddle
# and the ring gear reaches the collar via a short standoff. The
# top-level `pinion`-vs-`bearing_spider` no-interference assert is the
# guard that catches any future saddle-depth drift.
#
#   z=16.5 base flange underside / ring-gear standoff top (BASE_COL_Z_BOT)
#   z=8    turntable bottom / seat top (ramp_mechanism)
#   z=0    rail top / seat bottom
#   z=-7   spider-saddle bottom (ramp_mechanism) — mesh stays below
#   z=-9   gear-mesh top (pinion top, ring-gear teeth top)
#   z=-17  gear-mesh bottom / ring-gear bottom
#   z=-18  pinion bottom
#   z=-20  rail bottom / motor-mount top (bolts up into rail T-slot)
#   z=-25  motor-mount bottom / motor face (NEMA bolts up into the mount)
#   z=-65  motor body bottom
GEAR_FACE_W = 8.0                                  # mesh face width
MESH_Z_TOP = -9.0                                  # below the spider saddle (≈ -7)
MESH_Z_BOT = MESH_Z_TOP - GEAR_FACE_W              # -17

RING_GEAR_Z_TOP = BASE_COL_Z_BOT                   # 16.5 — standoff bolts to the base flange underside
RING_GEAR_Z_BOT = MESH_Z_BOT                       # -17 — annulus runs collar → teeth

PINION_Z_TOP = MESH_Z_TOP                          # -9
PINION_FACE_W = GEAR_FACE_W + 1.0                  # 9 — overhangs the gear face
PINION_Z_BOT = PINION_Z_TOP - PINION_FACE_W        # -18

MOUNT_T = 5.0
MOUNT_Z_TOP = -RAIL_SIZE                           # -20 — rail bottom face
MOUNT_Z_BOT = MOUNT_Z_TOP - MOUNT_T                # -25

# Motor — NEMA-17 footprint (42.3 mm body), output face flush with the
# mount underside; body hangs below. Real NEMA-17 is 42.3 mm square.
MOTOR_BODY_W = 42.3
MOTOR_BODY_H = 40.0
MOTOR_TOP_Z = MOUNT_Z_BOT                          # -25 — face bolts up to the mount
MOTOR_BOTTOM_Z = MOTOR_TOP_Z - MOTOR_BODY_H        # -65

# Off-axis placement: on a back-side radial-rail azimuth (rails sit at
# (k+0.5)·60°). 210° is two sections back from the +X exit/ramp at 0°,
# so the motor stays clear of the delivery side. The mount arm runs
# along this rail; pinion + motor axis sit at CENTER_DIST on it.
MOTOR_AZIMUTH_DEG = 210.0

# Motor mount — radial arm under the chosen rail. Inner end covers the
# motor footprint (motor spans r ≈ CENTER_DIST ∓ MOTOR_BODY_W/2 ≈ 61..103);
# outer end overlaps the rail (r ≥ RAIL_INNER_R) by 10 mm to bolt.
# Central clearance hole passes the bare output shaft (the pinion sits
# above the mount). MOUNT_R_INNER is the blank's inner end only: the
# deflector chamfer (`_deflector_cut`, below) sets the real one at
# DEFLECTOR_R_BOT..DEFLECTOR_R_TOP.
MOUNT_R_INNER = 52.0
MOUNT_R_OUTER = RAIL_INNER_R + 10.0                    # 110
MOUNT_R_CENTER = (MOUNT_R_INNER + MOUNT_R_OUTER) / 2   # 81
MOUNT_L = MOUNT_R_OUTER - MOUNT_R_INNER                # 58
MOUNT_W = MOTOR_BODY_W + 4.0                           # 46.3 — straddles the motor
MOUNT_SHAFT_HOLE_D = 12.0                              # clears the bare output shaft

WALL_MIN = 1.5


# --- Fall path below the base column -------------------------------------
#
# A brick leaves the base column through the ring gear's square hole at
# z = FALL_EXIT_Z. The hole turns with the column, so over every azimuth
# the exit is a disc of the square's half-diagonal. The vertical fall from
# there down to the catch is a keep-out (`fall_path`, declared in
# `build_drive_train`); nothing static may stand in it but the mount's
# deflector, which is what shields the motor.
FALL_SWEEP_R = COL_INTERIOR_SIDE * math.sqrt(2) / 2    # 63.64
FALL_EXIT_Z = RING_GEAR_Z_BOT                          # -17
FALL_CLEAR = 1.0                                       # static part to the vertical fall
MOTOR_INNER_R = CENTER_DIST - MOTOR_BODY_W / 2         # 60.8 — inside FALL_SWEEP_R

# The mount's inner end, a chamfer square to the arm: its flat top stops
# FALL_CLEAR outside the disc (on the arm's axis, where the disc reaches
# furthest out), then the face slopes down and in to FALL_CLEAR inside
# the motor's (planar) inner face, ending in a vertical land so the
# edge is a printable wall, not a knife edge. Everything the disc drops
# onto the motor lands on this slope first. The slope is held against the
# ramp's friction budget at the tower root (it owns MU_STATIC_BUDGET).
DEFLECTOR_R_TOP = FALL_SWEEP_R + FALL_CLEAR                       # 64.64
DEFLECTOR_R_BOT = MOTOR_INNER_R - FALL_CLEAR                      # 59.8
DEFLECTOR_LAND = 2.0                                              # vertical face at the bottom
DEFLECTOR_DEG = math.degrees(math.atan2(
    MOUNT_T - DEFLECTOR_LAND, DEFLECTOR_R_TOP - DEFLECTOR_R_BOT,
))                                                                # ≈ 31.8


# --- Bottom catch ----------------------------------------------------------
#
# A round tray under the motor, sitting on a cradle hung from three radial
# rails. Inputs: the scatter allowance, the depth, the clearances and the
# hanger rails. Derived: the rim height (under the motor), the mouth radius
# (the swept disc grown by the scatter cone over the drop), and from those
# the capacity, which is recorded against CATCH_CAPACITY_MIN_CM3.
#
# Scatter: the vertical fall is exact; a brick also leaves the hole with
# some sideways speed from its last wall bounce. CATCH_SCATTER_DEG is the
# half-angle of the cone that allowance grows at, from the exit to the
# rim — a hand allowance, not a measurement (bench).
CATCH_SCATTER_DEG = 10.0
CATCH_MOUTH_MARGIN = 1.0              # scatter cone to the tray's inner wall, at the rim
CATCH_MOTOR_CLEAR = 2.0               # rim below the motor's bottom face
CATCH_WALL_T = 2.0
CATCH_FLOOR_T = 2.0
CATCH_DEPTH = 40.0                    # rim to floor, inside
CATCH_CAPACITY_MIN_CM3 = 500.0        # a fault's worth, not a bay: owner, "size it small"

CATCH_RIM_Z = MOTOR_BOTTOM_Z - CATCH_MOTOR_CLEAR                 # -67
CATCH_DROP = FALL_EXIT_Z - CATCH_RIM_Z                           # 50
CATCH_SCATTER_R = FALL_SWEEP_R + CATCH_DROP * math.tan(math.radians(CATCH_SCATTER_DEG))  # ≈ 72.46
CATCH_MOUTH_R = CATCH_SCATTER_R + CATCH_MOUTH_MARGIN             # ≈ 73.46, inner wall
CATCH_OUTER_R = CATCH_MOUTH_R + CATCH_WALL_T                     # ≈ 75.46
CATCH_FLOOR_Z = CATCH_RIM_Z - CATCH_DEPTH                        # -107, floor top
CATCH_BOTTOM_Z = CATCH_FLOOR_Z - CATCH_FLOOR_T                   # -109, tray underside
CATCH_CAPACITY_CM3 = math.pi * CATCH_MOUTH_R ** 2 * CATCH_DEPTH / 1000.0   # ≈ 678

# Cradle: a shelf the tray stands on, three posts up to the free inner
# ends of radial rails, and low curbs that locate the tray. Rails sit at
# (k+0.5)·60°; the motor's rail (MOTOR_AZIMUTH_DEG, 210°) is taken, so the
# posts use 30°, 150° and 330° — they flank the −Y draw-out path. A rail's
# bottom face is free between RAIL_INNER_R and the inner-vertex bracket's
# inboard arm (r ≈ 119.7, `inner_vertex_bracket` ARM_IN_L_TOTAL — a sibling,
# so not imported; the tower root measures the gap).
CATCH_POST_AZIMUTHS_DEG = (30.0, 150.0, 330.0)
CATCH_POST_W = 16.0
CATCH_POST_R = RAIL_INNER_R + 1.0 + CATCH_POST_W / 2             # 109 — r 101..117
CATCH_SHELF_T = 4.0
CATCH_CURB_GAP = 0.5                  # tray wall to curb
CATCH_CURB_W = 3.0
CATCH_CURB_H = 5.0
CATCH_CURB_L = 40.0
CATCH_PULL_AZIMUTH_DEG = 270.0        # −Y: toward the operator
CATCH_PULL_TRAVEL = 2 * CATCH_OUTER_R + CATCH_CURB_GAP           # clear of its own footprint


# --- Part functions -----------------------------------------------------

def _on_drive_axis(part: Part, z_center: float) -> Part:
    """Place a part centered on the off-axis motor/pinion axis at the
    given z. The drive axis is at radius CENTER_DIST on azimuth
    MOTOR_AZIMUTH_DEG from the column axis."""
    return Rot(0, 0, MOTOR_AZIMUTH_DEG) * Pos(CENTER_DIST, 0, z_center) * part


def motor() -> Part:
    """NEMA-17-style stub: square body, output face up at MOTOR_TOP_Z,
    body hanging below. Placed off-axis on the drive axis."""
    z = (MOTOR_TOP_Z + MOTOR_BOTTOM_Z) / 2
    return _on_drive_axis(Box(MOTOR_BODY_W, MOTOR_BODY_W, MOTOR_BODY_H), z)


def pinion() -> Part:
    """Drive pinion on the motor shaft, meshing with the ring gear.
    Cheap model: pitch cylinder (no teeth). Placed off-axis on the
    drive axis at CENTER_DIST from the column axis."""
    z = (PINION_Z_TOP + PINION_Z_BOT) / 2
    return _on_drive_axis(Cylinder(PINION_PITCH_D / 2, PINION_FACE_W), z)


def ring_gear() -> Part:
    """Ring gear bolted to the base column's bottom flange underside.
    The upper section is a standoff (flange underside z = BASE_COL_Z_BOT
    down through the turntable and seat bores to the saddle-clearing
    mesh plane); the lower section carries the teeth. Cheap
    model: a single pitch-diameter annulus with the same square brick-
    pass hole as the collar (teeth omitted). Centered on the column axis."""
    h = RING_GEAR_Z_TOP - RING_GEAR_Z_BOT
    z = (RING_GEAR_Z_TOP + RING_GEAR_Z_BOT) / 2
    disc = Pos(0, 0, z) * Cylinder(RING_PITCH_D / 2, h)
    hole = Pos(0, 0, z) * Box(COL_INTERIOR_SIDE, COL_INTERIOR_SIDE, h * 2)
    return disc - hole


def _deflector_cut() -> Part:
    """What the mount's inner end is cut by, in the mount's own radial
    frame and turned onto MOTOR_AZIMUTH_DEG: everything inboard of the
    land at DEFLECTOR_R_BOT, then a plane rising to DEFLECTOR_R_TOP at
    the mount's top face. The chamfer is planar and square to the arm —
    parallel to the motor's inner face, which is what it covers — so it
    meets the arm's side faces square, with no wedge."""
    z_land = MOUNT_Z_BOT + DEFLECTOR_LAND
    run = DEFLECTOR_R_TOP - DEFLECTOR_R_BOT
    rise = MOUNT_Z_TOP - z_land
    over = 1.0
    u0 = MOUNT_R_INNER - over
    profile = Polygon(
        (u0, MOUNT_Z_BOT - over),
        (DEFLECTOR_R_BOT, MOUNT_Z_BOT - over),
        (DEFLECTOR_R_BOT, z_land),
        (DEFLECTOR_R_TOP + over * run / rise, MOUNT_Z_TOP + over),
        (u0, MOUNT_Z_TOP + over),
        align=None,
    )
    cut = extrude(Plane.XZ * profile, amount=MOUNT_W, both=True)
    return Rot(0, 0, MOTOR_AZIMUTH_DEG) * cut


def motor_mount() -> Part:
    """Printed radial arm at the rail-bottom plane. Bolts up into one
    radial rail's T-slot at its outer end and carries the motor below
    via the NEMA face; a central hole passes the output shaft up to the
    pinion. The inner end is the fall-path deflector (`_deflector_cut`):
    flat top outside the swept exit, sloped face over the motor's inner
    edge."""
    # Cut-feature bounds over the deflector's own parameters: the slope
    # must reach inside the motor's face and the land must leave a wall.
    assert DEFLECTOR_R_BOT < MOTOR_INNER_R < FALL_SWEEP_R < DEFLECTOR_R_TOP, (
        f"Deflector r {DEFLECTOR_R_BOT:.2f}..{DEFLECTOR_R_TOP:.2f} must "
        f"bracket the motor face {MOTOR_INNER_R:.2f} and the swept exit "
        f"{FALL_SWEEP_R:.2f}."
    )
    assert WALL_MIN <= DEFLECTOR_LAND < MOUNT_T, (
        f"DEFLECTOR_LAND {DEFLECTOR_LAND} must leave a printable land and "
        f"a slope in a {MOUNT_T} mm plate."
    )
    z = (MOUNT_Z_TOP + MOUNT_Z_BOT) / 2
    arm = Rot(0, 0, MOTOR_AZIMUTH_DEG) * Pos(MOUNT_R_CENTER, 0, z) * Box(
        MOUNT_L, MOUNT_W, MOUNT_T
    )
    bore = _on_drive_axis(Cylinder(MOUNT_SHAFT_HOLE_D / 2, MOUNT_T * 2), z)
    return arm - bore - _deflector_cut()


def _disc(r: float, z_top: float, z_bot: float) -> Part:
    return Pos(0, 0, (z_top + z_bot) / 2) * Cylinder(r, z_top - z_bot)


def fall_path() -> Part:
    """Keep-out: the vertical fall of anything leaving the base column,
    at every column azimuth — the disc r = FALL_SWEEP_R from the base
    column's bottom face (above it the turning column's own walls enclose
    the brick) down to the catch's rim — less what the motor mount
    shadows. The mount is solid over its whole footprint, so nothing
    falling straight down reaches the region under it except by sliding
    off its deflector, inward; that region is cut out from the mount's
    top face down. The mount itself is held to the band above its top
    face (`fall_above_mount`)."""
    disc = _disc(FALL_SWEEP_R, BASE_COL_Z_BOT, CATCH_RIM_Z)
    foot = motor_mount().faces().sort_by(Axis.Z)[0]          # bottom face: the full footprint
    depth = MOUNT_Z_BOT - CATCH_RIM_Z + 1.0
    shadow = extrude(foot, amount=depth, dir=(0, 0, -1)) + extrude(
        foot, amount=MOUNT_T, dir=(0, 0, 1),
    )
    return disc - shadow


def fall_above_mount() -> Part:
    """Keep-out: the vertical fall between the exit and the mount's top
    face — where the mount's flat top must not reach."""
    return _disc(FALL_SWEEP_R, FALL_EXIT_Z, MOUNT_Z_TOP)


def catch_tray() -> Part:
    """Open round tray under the motor: rim at CATCH_RIM_Z, inner radius
    CATCH_MOUTH_R, floor CATCH_FLOOR_T thick. Printed floor-down."""
    outer = _disc(CATCH_OUTER_R, CATCH_RIM_Z, CATCH_BOTTOM_Z)
    cavity = _disc(CATCH_MOUTH_R, CATCH_RIM_Z + 1.0, CATCH_FLOOR_Z)
    return outer - cavity


def _post_xy(az_deg: float) -> tuple[float, float]:
    a = math.radians(az_deg)
    return CATCH_POST_R * math.cos(a), CATCH_POST_R * math.sin(a)


def catch_cradle() -> Part:
    """Shelf + three posts + locating curbs, one printed part (shelf
    down). Each post stands under a radial rail's free inner end, its
    top face on the rail's bottom face (z = MOUNT_Z_TOP, the same plane
    the motor mount bolts to); the shelf top carries the tray at
    CATCH_BOTTOM_Z. Two side curbs and a back stop locate the tray; the
    −Y side is open so it draws out toward the operator."""
    shelf_z_top = CATCH_BOTTOM_Z
    shelf_z_bot = shelf_z_top - CATCH_SHELF_T
    xs = [_post_xy(a)[0] for a in CATCH_POST_AZIMUTHS_DEG]
    ys = [_post_xy(a)[1] for a in CATCH_POST_AZIMUTHS_DEG]
    half = CATCH_POST_W / 2
    back = CATCH_OUTER_R + CATCH_CURB_GAP + CATCH_CURB_W
    x0, x1 = min(xs) - half, max(xs) + half
    y0, y1 = min(ys) - half, back
    shelf = Pos((x0 + x1) / 2, (y0 + y1) / 2, (shelf_z_top + shelf_z_bot) / 2) * Box(
        x1 - x0, y1 - y0, CATCH_SHELF_T,
    )
    body = shelf
    for az in CATCH_POST_AZIMUTHS_DEG:
        h = MOUNT_Z_TOP - shelf_z_bot
        body = body + Rot(0, 0, az) * Pos(
            CATCH_POST_R, 0, (MOUNT_Z_TOP + shelf_z_bot) / 2,
        ) * Box(CATCH_POST_W, CATCH_POST_W, h)
    curb_z = shelf_z_top + CATCH_CURB_H / 2
    curb_r = CATCH_OUTER_R + CATCH_CURB_GAP + CATCH_CURB_W / 2
    curb_full_h = CATCH_CURB_H + CATCH_SHELF_T
    curb_zc = (shelf_z_top + CATCH_CURB_H + shelf_z_bot) / 2
    for sx in (-1, 1):
        body = body + Pos(sx * curb_r, 0, curb_zc) * Box(CATCH_CURB_W, CATCH_CURB_L, curb_full_h)
    body = body + Pos(0, curb_r, curb_zc) * Box(CATCH_CURB_L, CATCH_CURB_W, curb_full_h)
    return body


def fall_scatter() -> Part:
    """Keep-out: the scatter cone — the swept exit disc growing at
    CATCH_SCATTER_DEG from the exit down to the catch's rim. The tray's
    wall must stand outside it at the rim."""
    h = CATCH_DROP
    return Pos(0, 0, (FALL_EXIT_Z + CATCH_RIM_Z) / 2) * Cone(CATCH_SCATTER_R, FALL_SWEEP_R, h)


def catch_pull_path() -> Part:
    """Keep-out: the tray drawn out along CATCH_PULL_AZIMUTH_DEG by
    CATCH_PULL_TRAVEL — the cylinder at both ends and the slab between.
    It stands on the shelf top (its `seat`)."""
    h = CATCH_RIM_Z - CATCH_BOTTOM_Z
    zc = (CATCH_RIM_Z + CATCH_BOTTOM_Z) / 2
    start = Pos(0, 0, zc) * Cylinder(CATCH_OUTER_R, h)
    end = Pos(CATCH_PULL_TRAVEL, 0, zc) * Cylinder(CATCH_OUTER_R, h)
    slab = Pos(CATCH_PULL_TRAVEL / 2, 0, zc) * Box(CATCH_PULL_TRAVEL, 2 * CATCH_OUTER_R, h)
    return Rot(0, 0, CATCH_PULL_AZIMUTH_DEG) * (start + slab + end)


# --- Assembly -----------------------------------------------------------

def build_drive_train(
    material_fn: Callable[[str], str | None] | None = None,
) -> Assembly:
    """Off-axis drive train — ring gear + pinion + motor + motor mount
    — as a sub-`Assembly` in the base-floor-local frame (rail-top
    datum at z=0; every part hangs below it). The part functions carry
    their own off-axis placement, so all four are placed at identity.
    ``material_fn`` follows the same leaf-name contract as the frame
    builders. Colour is not a parameter — the unit root paints the
    finished tree (``cad/design_systems.py:paint``).
    """
    def _mt(name: str) -> str | None:
        return material_fn(name) if material_fn is not None else None

    a = Assembly()
    for n, part in (
        ("ring_gear", ring_gear()),
        ("pinion", pinion()),
        ("motor", motor()),
        ("motor_mount", motor_mount()),
    ):
        a = a.with_part(n, part, material=_mt(n))
    # The fall path out of the base column. Declared here, where the exit
    # and the mount that shadows the motor are known; the frame's parts
    # are held against it at the tower root. The ring gear is not held:
    # it turns with the column (modeled static here), and its hole is the
    # column's, so the pass through it is `check_base_fall_path.py`'s.
    a = (
        a
        .with_keepout("fall_path", fall_path())
        .with_keepout("fall_above_mount", fall_above_mount())
        .assert_clear_of(["motor", "pinion"], "fall_path", min_mm=FALL_CLEAR,
                         name="drive_clear_of_fall_path")
        # The mount's flat top stops FALL_CLEAR outside the swept exit;
        # below its top face only the deflector slope is inside the disc
        # (the slope's angle is held at the tower root). The nearest
        # material is the slope, so the constructed distance is the
        # horizontal clearance seen normal to it.
        .assert_clear_of(
            "motor_mount", "fall_above_mount",
            min_mm=FALL_CLEAR * math.sin(math.radians(DEFLECTOR_DEG)),
            name="mount_top_clear_of_fall_path",
        )
    )
    # Mesh validity — close but not touching: the cheap pitch blanks
    # must sit in the [MESH_BACKLASH, MESH_BACKLASH + 0.1] band (too
    # far = no mesh, too close = jam; real teeth interleave within
    # it). Declared here, where both gears live, so every composed
    # run evaluates the propagated form. Authoritative over the
    # CENTER_DIST derivation above: this measures the placed geometry.
    # The placed gap equals MESH_BACKLASH by construction; bound
    # comparisons carry the library's 1e-6 tolerance, so measured ==
    # bound passes despite solver noise.
    a = a.assert_distance(
        "ring_gear", "pinion",
        min_mm=MESH_BACKLASH, max_mm=MESH_BACKLASH + 0.1,
        name="gear_mesh_backlash",
    )
    # The bolt-on ring must seat on the base collar's face, so its pitch
    # diameter has to track COLLAR_OD to within one tooth height. The
    # collar belongs to another sub-assembly, so this is a recorded
    # scalar rather than a part-pair distance — but recording it still
    # beats a bare assert: the measured offset lands in mechanism.json
    # and `khana diff` sees it drift before it crosses the bound.
    a = a.assert_scalar(
        "ring_gear_collar_seat",
        abs(RING_PITCH_D - COLLAR_OD),
        le=2 * MODULE,
        detail=(
            f"|ring pitch D {RING_PITCH_D:.1f} − base collar OD "
            f"{COLLAR_OD:.1f}| must stay within one tooth height "
            f"({2 * MODULE:.1f} mm); re-pick RING_TEETH if it fails"
        ),
    )
    return a


def build_bottom_catch(
    material_fn: Callable[[str], str | None] | None = None,
) -> Assembly:
    """The bottom catch — tray on its cradle — as a sub-`Assembly` in the
    base-floor-local frame. Static. Its claims about itself and the
    falling brick are declared here; its rests on the rails and its
    clearances to the frame and drive train are the tower root's."""
    def _mt(name: str) -> str | None:
        return material_fn(name) if material_fn is not None else None

    return (
        Assembly()
        .with_part("catch_tray", catch_tray(), material=_mt("catch_tray"))
        .with_part("catch_cradle", catch_cradle(), material=_mt("catch_cradle"))
        .with_keepout("fall_scatter", fall_scatter())
        .with_keepout("pull_path", catch_pull_path(),
                      seat=Plane.XY.offset(CATCH_BOTTOM_Z))
        # Everything the scatter cone carries to the rim lands inside the
        # tray's wall. Constructed at exactly CATCH_MOUTH_MARGIN.
        .assert_clear_of("catch_tray", "fall_scatter", min_mm=CATCH_MOUTH_MARGIN,
                         name="catch_mouth_takes_scatter")
        # The tray stands on the shelf, located by curbs it does not touch.
        .assert_tangent_contact("catch_tray", "catch_cradle")
        .assert_solid_count("catch_tray",
                            detail="the cavity cut has holed the floor or wall")
        .assert_solid_count("catch_cradle",
                            detail="a post or curb no longer meets the shelf")
        .assert_scalar(
            "catch_capacity_cm3", CATCH_CAPACITY_CM3, ge=CATCH_CAPACITY_MIN_CM3,
            detail=f"π·{CATCH_MOUTH_R:.1f}²·{CATCH_DEPTH:.0f} mm — raise CATCH_DEPTH",
        )
    )


def make_assembly() -> Assembly:
    """Standalone base-floor drive-train assembly.

    Four parts: ring gear on the column axis (z = -17..0), pinion on the
    off-axis drive axis (z = -18..-9) meshing with it, motor below the
    mount (z = -65..-25), and the mount arm bridging the two (z =
    -25..-20). The pinion ↔ ring *mesh* (the load-bearing relationship)
    is the `gear_mesh_backlash` assert_distance declared in
    `build_drive_train`; the no-interference assert here is a weaker
    guard that the cheap pitch blanks don't gross-overlap.

    Plus the bottom catch (`catch` subtree) under the motor, held clear
    of the drive train, of the fall path, and on its draw-out path. The
    rails it hangs from are not in this unit: the posts' rests, and the
    frame against the fall path and the draw-out, are the tower root's.
    """
    return (
        build_drive_train()
        .with_subassembly("catch", build_bottom_catch())
        .assert_no_interference_within([
            "ring_gear",
            "pinion",
            "motor",
            "motor_mount",
        ])
        .assert_no_interference_between(
            "catch", ["ring_gear", "pinion", "motor", "motor_mount"],
        )
        .assert_clear_of("catch", "fall_path", min_mm=FALL_CLEAR,
                         name="catch_clear_of_fall_path")
        .assert_clear_of(["ring_gear", "pinion", "motor", "motor_mount"],
                         "catch.pull_path", name="drive_clear_of_catch_pull")
    )


assembly = make_assembly()
