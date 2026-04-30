"""
M03 scanner — rev-4 turret drive coupling (slice g of the 4-station turret).

Standalone NEMA17 stepper + GT2 belt drive that turns the slice (b) shaft
in 90° increments. Driven 60T-equivalent pulley keyed to the turret
shaft sits in the 29 mm gap between the slice (b) housing top
(y = -110) and the slice (f1) cam disc bottom (y = -81); driver
20T-equivalent pulley keyed to the motor shaft. 3:1 reduction; standard
200 mm GT2 closed-loop belt at C = 58.6 mm center-to-center distance.
Motor placed on the +X side of the hub (opposite the kicker on -X)
hanging from a printed L-bracket bolted to the slice (a) cross-rail's
top T-slot.

Drive direction was decided over Geneva and worm alternatives in the
2026-04-30 turret discussion (see `turret_alternatives.md` and
`turret_design_brief.md` §6.5): GT2 belt is the maker-default low-risk
path. Worm gives self-locking dwell (no holding current) but tighter
mounting constraints on aligning a 90° gearhead with the vertical
shaft; Geneva gives mechanical phase-locking with the existing dump
cam but introduces precision-fit prototype risk on the slot/pin pair.
Belt accepts ~5 W of stepper holding current and one mitigatable
failure mode (shock-induced belt skip under kicker reaction torque,
addressable with proper tension or an idler in a follow-up).

Members (4 functional + 2 stubs):
  * drive_pulley_shaft  — stock 60T GT2 metal pulley keyed to the
                          turret shaft. Modeled as a simple cylinder
                          at the GT2-60T pitch diameter; the real part
                          has helical GT2 teeth on the OD and an M5
                          set-screw on a flat. Bore Ø8.4 (clearance
                          fit; the metal stock part presses on at
                          slight interference, modeled as clearance to
                          satisfy assert_no_interference).
  * drive_pulley_motor  — stock 20T GT2 metal pulley keyed to the
                          NEMA17 shaft. Same idiom as the driven
                          pulley.
  * drive_motor         — NEMA17 stepper stub (42.3 × 40 × 42.3 body,
                          Ø5 × 22 shaft, 4× M3 mounting holes on a
                          31 × 31 mm pattern, Ø22 centering boss).
                          Hangs below the bracket plate with shaft
                          pointing +Y up.
  * drive_bracket       — printed L-bracket: foot bolts into the
                          cross-rail's top T-slot at z = 0; vertical
                          wall rises outboard of the motor body in +X
                          direction; horizontal cantilever plate
                          extends inboard over the motor and hosts the
                          4× M3 motor mounting holes plus a Ø22
                          centering-boss pass-through.

For-standalone-only stubs (1 each, duplicated rather than sibling-
imported per `cad/CLAUDE.md` principle 2):
  * hub_shaft_stub      — slice (b) Ø8 turret shaft, used here for the
                          shaft-to-pulley bore clearance check.
  * cross_rail_stub     — slice (a) cross-rail under the bracket, for
                          the bolt-line + foot tangent check.

Coordinate frame = turret-local frame (matches `../params.py`):
    origin = hub axis at the cross-rail's top face Y plane
    +X     = operator's right            (parallel to world +X)
    +Y     = up                          (parallel to world +Y)
    +Z     = away from operator          (parallel to world +Z)

Belt:
The GT2 closed-loop belt at 200 mm length is a stock part and not
modeled as a solid in this assembly — its physical envelope (6 mm
height, ~1 mm thickness, routed at the pulleys' pitch radii in the X-Z
plane at y = -86) lies in clear airspace by construction. The straight
runs span x ∈ [+19.1, +52.2] approx at z = ±19.1 (driven-side wrap
tangent) connecting to the driver pulley at x = +58.6. Nothing else
sits in this y-z band.

Homing:
A photo-interrupter sensor + flag for shaft-position homing is deferred
to a follow-up slice. The flag would key to the shaft below the driven
pulley (y ∈ [-110, -90] is clear airspace for it) and the sensor would
mount on a small printed bracket next to the cross-rail. Not gating
this slice.

Print orientation (bracket): bottom face on the build plate, +Y up.
All bores are vertical (M5 foot bolts, Ø22 motor centering boss, 4× M3
motor mounting holes) and print as clean through-holes. The cantilever
plate's underside (-Y face from x = BRACKET_PLATE_X_MIN out to the
wall at BRACKET_WALL_X_MIN) is a horizontal overhang at 90° from
vertical and requires print-time supports — the slicer auto-generates
these for the ~50 mm × 50 mm patch under the cantilever. We accept the
90° angle in the inspect threshold (same precedent as the slice (f1)
cam, slice (f2) kicker, and platform_hinge_bearing for their horizontal
bores). A printable-without-supports redesign — adding a 45° gusset
between the wall and plate's underside — was rejected because the
gusset would crash the motor body in X (motor body inboard face at
x ≈ +37.65, gusset would need to extend from the wall at x = +85
inward through the motor's X range). Rotating the print to +Z up moves
the cantilever overhang to the side faces (good) but converts the Ø22
boss hole into a horizontal bore (bad: 22 mm cannot bridge cleanly).
Pulleys are stock parts (not printed), so no inspect call.
"""

import math
import sys
from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from turret.params import (  # noqa: E402
    TURRET_DUMP_CAM_BOTTOM_Y,
    TURRET_FRAME_CROSS_Z,
    TURRET_FRAME_FLOOR_Y,
    TURRET_HUB_X,
    TURRET_HUB_Z,
    TURRET_SHAFT_D,
    TURRET_SHAFT_TOP_Y,
    WALL_MIN,
)


# --- Cross-rail interface (private mirror of slice a) ------------------

MEMBER = 20.0                                              # 2020 extrusion
RAIL_TOP_Y = TURRET_FRAME_FLOOR_Y + MEMBER / 2             # = -210

# Slice (b) housing top (private mirror; bound assertions reference it).
HOUSING_TOP_Y = -110.0


# --- GT2 pulley sizing (60T driven, 20T driver) ------------------------

# GT2 belt: 2 mm tooth pitch. Pitch diameter D_p = (N · pitch) / π.
GT2_PITCH = 2.0
DRIVEN_TEETH = 60
DRIVER_TEETH = 20
DRIVE_RATIO = DRIVEN_TEETH / DRIVER_TEETH                  # = 3.0

DRIVEN_PITCH_D = DRIVEN_TEETH * GT2_PITCH / math.pi        # ≈ 38.197
DRIVER_PITCH_D = DRIVER_TEETH * GT2_PITCH / math.pi        # ≈ 12.732

# Modeled OD: pitch diameter (real GT2 teeth are very shallow; the
# pitch circle approximates the belt-engagement envelope to within
# < 0.5 mm, which is below all our clearance thresholds). The flange
# OD on a real metal pulley is typically pitch_d + ~2 mm; we ignore
# that here.
DRIVEN_PULLEY_OD = DRIVEN_PITCH_D                          # ≈ 38.20
DRIVER_PULLEY_OD = DRIVER_PITCH_D                          # ≈ 12.73

# Pulley height (Y direction). GT2-6 belt is 6 mm tall; with 1 mm of
# flange margin per side, 8 mm pulleys are common stock.
PULLEY_HEIGHT = 8.0

# Y center: midway through the 29 mm gap between housing top (-110)
# and cam bottom (-81). 1 mm clear of cam at the top, 19 mm clear of
# housing at the bottom.
PULLEY_Y_CENTER = -86.0
_PULLEY_TOP_Y = PULLEY_Y_CENTER + PULLEY_HEIGHT / 2        # = -82
_PULLEY_BOTTOM_Y = PULLEY_Y_CENTER - PULLEY_HEIGHT / 2     # = -90


# --- Center-to-center distance (200 mm GT2 closed-loop belt) -----------

# Stock GT2 closed-loop belt lengths near our target: 200 mm = 100
# teeth at 2 mm pitch. For two pulleys of pitch radii R, r at distance
# C, the closed-loop belt length is:
#     L = 2C·cos(α) + R·(π + 2α) + r·(π - 2α)  with α = arcsin((R-r)/C)
# Solving numerically at R = 19.099, r = 6.366, L = 200 → C ≈ 58.6.
DRIVE_CENTER_DIST = 58.6
DRIVE_BELT_LENGTH = 200.0


def _belt_length(R: float, r: float, C: float) -> float:
    alpha = math.asin((R - r) / C)
    return (
        2 * C * math.cos(alpha)
        + R * (math.pi + 2 * alpha)
        + r * (math.pi - 2 * alpha)
    )


_BELT_L_DERIVED = _belt_length(
    DRIVEN_PULLEY_OD / 2, DRIVER_PULLEY_OD / 2, DRIVE_CENTER_DIST
)


# --- Pulley bores ------------------------------------------------------

# Ø8 turret shaft + 0.4 mm clearance (matches slice f1 cam idiom).
PULLEY_SHAFT_BORE_D = TURRET_SHAFT_D + 0.4                 # = 8.4

# Ø5 NEMA17 shaft + 0.4 mm clearance.
MOTOR_SHAFT_D = 5.0
PULLEY_MOTOR_BORE_D = MOTOR_SHAFT_D + 0.4                  # = 5.4


# --- NEMA17 stub -------------------------------------------------------

# Body 42.3 × 40 × 42.3 (X × Y × Z), centered on the motor axis. Shaft
# extends MOTOR_SHAFT_LENGTH in +Y from the body's +Y face (= flange
# face). Motor flange has 4× M3 holes on a 31 × 31 mm pattern centered
# on the shaft, plus a Ø22 round centering boss (matched by the bracket
# plate's pass-through hole).
MOTOR_BODY_X = 42.3
MOTOR_BODY_Y = 40.0
MOTOR_BODY_Z = 42.3
MOTOR_SHAFT_LENGTH = 22.0
MOTOR_BOLT_PATTERN = 31.0
MOTOR_BOLT_HOLE_D = 3.5                                    # M3 clearance
MOTOR_BOSS_D = 22.0                                        # centering boss

# Motor X position. Center of driver pulley = motor shaft axis. Sits
# on the +X side of the hub (opposite the kicker at x = -110 on -X).
MOTOR_X = DRIVE_CENTER_DIST                                # = +58.6
MOTOR_Z = 0.0                                              # on cross-rail


# --- Bracket geometry --------------------------------------------------

# Plate (horizontal, at the motor's flange face). Top face under the
# driven pulley with a 1 mm air gap so the (real) pulley's grub-screw
# wrench can swing in.
BRACKET_PLATE_T = 8.0
BRACKET_PLATE_AIRGAP_BELOW_PULLEY = 1.0
_BRACKET_PLATE_TOP_Y = (
    _PULLEY_BOTTOM_Y - BRACKET_PLATE_AIRGAP_BELOW_PULLEY
)                                                          # = -91
_BRACKET_PLATE_BOTTOM_Y = _BRACKET_PLATE_TOP_Y - BRACKET_PLATE_T  # = -99
_BRACKET_PLATE_CENTER_Y = (_BRACKET_PLATE_TOP_Y + _BRACKET_PLATE_BOTTOM_Y) / 2

# Motor flange Y = bracket plate bottom face (motor bolted up against
# plate). Body extends MOTOR_BODY_Y further in -Y.
_MOTOR_FLANGE_Y = _BRACKET_PLATE_BOTTOM_Y                  # = -99
_MOTOR_BODY_BOTTOM_Y = _MOTOR_FLANGE_Y - MOTOR_BODY_Y      # = -139
_MOTOR_BODY_CENTER_Y = (_MOTOR_FLANGE_Y + _MOTOR_BODY_BOTTOM_Y) / 2  # = -119

# Wall (vertical, outboard of the motor body in +X). Inner face clears
# the motor body by BRACKET_WALL_AIRGAP_X.
BRACKET_WALL_AIRGAP_X = 5.0
_MOTOR_BODY_OUTBOARD_X = MOTOR_X + MOTOR_BODY_X / 2        # ≈ +79.65
BRACKET_WALL_T = 15.0                                      # X thickness
BRACKET_WALL_X_MIN = _MOTOR_BODY_OUTBOARD_X + BRACKET_WALL_AIRGAP_X
BRACKET_WALL_X_MAX = BRACKET_WALL_X_MIN + BRACKET_WALL_T
_BRACKET_WALL_CENTER_X = (BRACKET_WALL_X_MIN + BRACKET_WALL_X_MAX) / 2
BRACKET_WALL_Z = 50.0
_BRACKET_WALL_BOTTOM_Y = RAIL_TOP_Y + 8.0                  # foot top, = -202
_BRACKET_WALL_TOP_Y = _BRACKET_PLATE_BOTTOM_Y              # = -99
_BRACKET_WALL_HEIGHT = _BRACKET_WALL_TOP_Y - _BRACKET_WALL_BOTTOM_Y  # = 103
_BRACKET_WALL_CENTER_Y = (_BRACKET_WALL_BOTTOM_Y + _BRACKET_WALL_TOP_Y) / 2

# Plate spans from inboard edge (clears M3 holes at MOTOR_X - 15.5)
# outboard to wall outboard face.
_PLATE_X_MIN_TIGHT = (
    MOTOR_X - MOTOR_BOLT_PATTERN / 2 - MOTOR_BOLT_HOLE_D / 2 - WALL_MIN
)
BRACKET_PLATE_X_MIN = round(_PLATE_X_MIN_TIGHT - 5.0, 1)   # ≈ +35
BRACKET_PLATE_X_MAX = BRACKET_WALL_X_MAX
_BRACKET_PLATE_CENTER_X = (BRACKET_PLATE_X_MIN + BRACKET_PLATE_X_MAX) / 2
BRACKET_PLATE_X_LEN = BRACKET_PLATE_X_MAX - BRACKET_PLATE_X_MIN
BRACKET_PLATE_Z = BRACKET_WALL_Z

# Foot. Spans wall outboard ± 30 mm in X to give two M5 bolt holes
# spaced 35 mm along the rail T-slot.
BRACKET_FOOT_X = 60.0
BRACKET_FOOT_Y = 8.0
BRACKET_FOOT_Z = BRACKET_WALL_Z
_BRACKET_FOOT_CENTER_X = _BRACKET_WALL_CENTER_X
_BRACKET_FOOT_X_MIN = _BRACKET_FOOT_CENTER_X - BRACKET_FOOT_X / 2
_BRACKET_FOOT_X_MAX = _BRACKET_FOOT_CENTER_X + BRACKET_FOOT_X / 2
_BRACKET_FOOT_BOTTOM_Y = RAIL_TOP_Y                        # = -210
_BRACKET_FOOT_TOP_Y = _BRACKET_FOOT_BOTTOM_Y + BRACKET_FOOT_Y  # = -202
_BRACKET_FOOT_CENTER_Y = (_BRACKET_FOOT_BOTTOM_Y + _BRACKET_FOOT_TOP_Y) / 2

BRACKET_BOLT_HOLE_D = 5.5                                  # M5 clearance
BRACKET_BOLT_SPACING_X = 35.0
_BRACKET_BOLT_HALF_X = BRACKET_BOLT_SPACING_X / 2


# --- Sanity asserts ----------------------------------------------------

# Belt length at chosen C is within tolerance of stock value.
_belt_err = abs(_BELT_L_DERIVED - DRIVE_BELT_LENGTH)
assert _belt_err < 0.5, (
    f"belt length at C = {DRIVE_CENTER_DIST} mm derives to "
    f"{_BELT_L_DERIVED:.2f} mm; expected close to {DRIVE_BELT_LENGTH} mm "
    f"(stock). Adjust DRIVE_CENTER_DIST or pick a different stock length."
)

# Pulley Y span clears housing top + cam bottom by ≥ 1 mm.
_pulley_to_housing = _PULLEY_BOTTOM_Y - HOUSING_TOP_Y
assert _pulley_to_housing >= 1.0, (
    f"driven pulley bottom ({_PULLEY_BOTTOM_Y}) too close to housing "
    f"top ({HOUSING_TOP_Y}); gap {_pulley_to_housing:.2f} mm < 1 mm"
)
_pulley_to_cam = TURRET_DUMP_CAM_BOTTOM_Y - _PULLEY_TOP_Y
assert _pulley_to_cam >= 1.0, (
    f"driven pulley top ({_PULLEY_TOP_Y}) too close to cam bottom "
    f"({TURRET_DUMP_CAM_BOTTOM_Y}); gap {_pulley_to_cam:.2f} mm < 1 mm"
)

# Driven pulley OD doesn't crash motor body in X. Y separation also
# guarantees no crash (pulley y < -82, motor body y > -99) but the X
# gap is the more useful margin to monitor.
_pulley_to_motor_body_x = (
    (MOTOR_X - MOTOR_BODY_X / 2) - DRIVEN_PULLEY_OD / 2
)
assert _pulley_to_motor_body_x >= WALL_MIN, (
    f"driven pulley OD reaches +{DRIVEN_PULLEY_OD/2:.2f} mm in X; motor "
    f"body inboard face at +{MOTOR_X - MOTOR_BODY_X/2:.2f}; X gap "
    f"{_pulley_to_motor_body_x:.2f} mm < WALL_MIN ({WALL_MIN})"
)

# Driver pulley OD doesn't crash the bracket plate's M3 holes. M3 holes
# at radius sqrt(15.5² + 15.5²) ≈ 21.92 from shaft (= driver OD axis).
# Driver OD radius ≈ 6.37. Gap between hole edge and OD edge:
_m3_hole_radius = math.sqrt(2) * MOTOR_BOLT_PATTERN / 2
_driver_to_m3 = _m3_hole_radius - MOTOR_BOLT_HOLE_D / 2 - DRIVER_PULLEY_OD / 2
assert _driver_to_m3 >= WALL_MIN, (
    f"driver pulley OD edge {DRIVER_PULLEY_OD/2:.2f} too close to M3 "
    f"hole edge at radius {_m3_hole_radius - MOTOR_BOLT_HOLE_D/2:.2f}; "
    f"gap {_driver_to_m3:.2f} mm < WALL_MIN ({WALL_MIN})"
)

# Bracket plate inboard edge has wall to inboard M3 hole.
_plate_inboard_to_hole_edge = (
    (MOTOR_X - MOTOR_BOLT_PATTERN / 2 - MOTOR_BOLT_HOLE_D / 2)
    - BRACKET_PLATE_X_MIN
)
assert _plate_inboard_to_hole_edge >= WALL_MIN, (
    f"bracket plate inboard edge to inboard M3 hole edge wall "
    f"{_plate_inboard_to_hole_edge:.2f} mm < WALL_MIN ({WALL_MIN})"
)

# Bracket plate Z edge has wall to M3 holes (M3 at z = ±15.5).
_plate_z_to_hole_edge = (
    BRACKET_PLATE_Z / 2 - MOTOR_BOLT_PATTERN / 2 - MOTOR_BOLT_HOLE_D / 2
)
assert _plate_z_to_hole_edge >= WALL_MIN, (
    f"bracket plate Z edge to M3 hole edge wall "
    f"{_plate_z_to_hole_edge:.2f} mm < WALL_MIN ({WALL_MIN}); raise "
    f"BRACKET_PLATE_Z"
)

# Boss hole walls — Ø22 boss in plate. Plate Z extent ±25; Z-edge wall
# = 25 - 11 = 14 mm.
_boss_to_z_edge = BRACKET_PLATE_Z / 2 - MOTOR_BOSS_D / 2
assert _boss_to_z_edge >= WALL_MIN, (
    f"boss hole to plate Z edge wall {_boss_to_z_edge:.2f} mm < "
    f"WALL_MIN ({WALL_MIN})"
)

# Bracket wall inboard face clears motor body in X by ≥ WALL_MIN.
_wall_to_motor = BRACKET_WALL_X_MIN - _MOTOR_BODY_OUTBOARD_X
assert _wall_to_motor >= WALL_MIN, (
    f"bracket wall inboard face {BRACKET_WALL_X_MIN:.2f} too close to "
    f"motor body outboard face {_MOTOR_BODY_OUTBOARD_X:.2f}; gap "
    f"{_wall_to_motor:.2f} mm < WALL_MIN ({WALL_MIN})"
)

# Foot bolt pattern fits inside foot footprint (X-direction).
_foot_bolt_outer_x = (
    _BRACKET_FOOT_CENTER_X + _BRACKET_BOLT_HALF_X + BRACKET_BOLT_HOLE_D / 2
)
assert _foot_bolt_outer_x + WALL_MIN <= _BRACKET_FOOT_X_MAX, (
    f"foot bolt outer edge {_foot_bolt_outer_x:.2f} too close to foot "
    f"X+ end face {_BRACKET_FOOT_X_MAX:.2f}; raise BRACKET_FOOT_X or "
    f"shrink BRACKET_BOLT_SPACING_X"
)

# Foot bolt inner edge clears the wall foot-print (so bolt heads can
# drop in from above without the wall in the way). Inner bolt at x =
# _BRACKET_FOOT_CENTER_X - 17.5; wall at x ∈ [WALL_X_MIN, WALL_X_MAX].
# We want the inner bolt to be OUTSIDE the wall in X.
_inner_bolt_x = _BRACKET_FOOT_CENTER_X - _BRACKET_BOLT_HALF_X
assert _inner_bolt_x < BRACKET_WALL_X_MIN - BRACKET_BOLT_HOLE_D / 2, (
    f"inner bolt at x = {_inner_bolt_x:.2f} sits inside the wall "
    f"footprint [{BRACKET_WALL_X_MIN:.2f}, {BRACKET_WALL_X_MAX:.2f}]; "
    f"bolt head won't have clearance from above"
)


# --- Part functions ----------------------------------------------------

def driven_pulley(
    od: float = DRIVEN_PULLEY_OD,
    height: float = PULLEY_HEIGHT,
    bore_d: float = PULLEY_SHAFT_BORE_D,
):
    """Stock 60T GT2 metal pulley keyed to the turret shaft. Modeled
    as a simple cylinder at pitch diameter with a Ø8.4 central bore.
    Real part has helical GT2 teeth on the OD and an M5 set-screw
    against a flat on the shaft."""
    return _pulley_body(od, height, bore_d)


def driver_pulley(
    od: float = DRIVER_PULLEY_OD,
    height: float = PULLEY_HEIGHT,
    bore_d: float = PULLEY_MOTOR_BORE_D,
):
    """Stock 20T GT2 metal pulley keyed to the NEMA17 shaft."""
    return _pulley_body(od, height, bore_d)


def _pulley_body(od: float, height: float, bore_d: float):
    body = Rot(90, 0, 0) * Cylinder(od / 2, height)
    body -= Rot(90, 0, 0) * Cylinder(bore_d / 2, height * 3)
    return body


def motor():
    """NEMA17 stub. Body centered on origin; shaft protrudes in +Y from
    the body's +Y face (= flange face). Caller positions the body
    center at (MOTOR_X, _MOTOR_BODY_CENTER_Y, MOTOR_Z)."""
    body = Box(MOTOR_BODY_X, MOTOR_BODY_Y, MOTOR_BODY_Z)
    shaft = (
        Pos(0, MOTOR_BODY_Y / 2 + MOTOR_SHAFT_LENGTH / 2, 0)
        * Rot(90, 0, 0)
        * Cylinder(MOTOR_SHAFT_D / 2, MOTOR_SHAFT_LENGTH)
    )
    return body + shaft


def bracket():
    """Printed L-bracket: foot + outboard wall + horizontal cantilever
    plate. Foot bolts vertically into the cross-rail's top T-slot at
    z = 0. Plate hosts the motor's 4× M3 mounting holes plus the Ø22
    centering-boss pass-through. Returned in turret-local coords (no
    `base` shift — caller multiplies the placement)."""
    foot = (
        Pos(_BRACKET_FOOT_CENTER_X, _BRACKET_FOOT_CENTER_Y, 0)
        * Box(BRACKET_FOOT_X, BRACKET_FOOT_Y, BRACKET_FOOT_Z)
    )
    for sx in (-1, 1):
        foot -= (
            Pos(
                _BRACKET_FOOT_CENTER_X + sx * _BRACKET_BOLT_HALF_X,
                _BRACKET_FOOT_CENTER_Y,
                0,
            )
            * Rot(90, 0, 0)
            * Cylinder(BRACKET_BOLT_HOLE_D / 2, BRACKET_FOOT_Y * 3)
        )

    wall = (
        Pos(_BRACKET_WALL_CENTER_X, _BRACKET_WALL_CENTER_Y, 0)
        * Box(BRACKET_WALL_T, _BRACKET_WALL_HEIGHT, BRACKET_WALL_Z)
    )

    plate = (
        Pos(_BRACKET_PLATE_CENTER_X, _BRACKET_PLATE_CENTER_Y, 0)
        * Box(BRACKET_PLATE_X_LEN, BRACKET_PLATE_T, BRACKET_PLATE_Z)
    )
    # Ø22 centering-boss pass-through.
    plate -= (
        Pos(MOTOR_X, _BRACKET_PLATE_CENTER_Y, 0)
        * Rot(90, 0, 0)
        * Cylinder(MOTOR_BOSS_D / 2, BRACKET_PLATE_T * 3)
    )
    # 4× M3 mounting clearance holes on a 31 × 31 mm pattern centered
    # on the motor shaft.
    for sx in (-1, 1):
        for sz in (-1, 1):
            plate -= (
                Pos(
                    MOTOR_X + sx * MOTOR_BOLT_PATTERN / 2,
                    _BRACKET_PLATE_CENTER_Y,
                    sz * MOTOR_BOLT_PATTERN / 2,
                )
                * Rot(90, 0, 0)
                * Cylinder(MOTOR_BOLT_HOLE_D / 2, BRACKET_PLATE_T * 3)
            )

    return foot + wall + plate


# --- Stubs for the standalone (duplicated, not imported) ---------------

def _shaft_stub(length: float):
    """Slice (b) Ø8 turret shaft mirror. Caller positions the base at
    the rail top; this function returns a cylinder centered on its
    own origin in X / Z, spanning Y ∈ [0, length]."""
    return (
        Pos(0, length / 2, 0)
        * Rot(90, 0, 0)
        * Cylinder(TURRET_SHAFT_D / 2, length)
    )


def _cross_rail_stub(length: float = BRACKET_FOOT_X + 40.0):
    """Slice (a) cross-rail stub centered under the bracket foot.
    Length spans the foot footprint with a 20 mm margin each side so
    the bolt-line + foot-tangent contact is exercised."""
    return (
        Pos(
            _BRACKET_FOOT_CENTER_X,
            TURRET_FRAME_FLOOR_Y,
            TURRET_FRAME_CROSS_Z,
        )
        * Box(length, MEMBER, MEMBER)
    )


# --- Assembly builder --------------------------------------------------

DRIVE_PART_NAMES = (
    "drive_pulley_shaft",
    "drive_pulley_motor",
    "drive_motor",
    "drive_bracket",
)


def add_turret_drive(a: Assembly, base=Pos(0, 0, 0)) -> Assembly:
    """Add the four drive parts. Parts are positioned in turret-local
    coordinates; `base` (default: identity) is pre-multiplied to lift
    them into a parent frame (e.g. world)."""
    a = a.add(
        "drive_pulley_shaft",
        driven_pulley(),
        location=base * Pos(TURRET_HUB_X, PULLEY_Y_CENTER, TURRET_HUB_Z),
    )
    a = a.add(
        "drive_pulley_motor",
        driver_pulley(),
        location=base * Pos(MOTOR_X, PULLEY_Y_CENTER, MOTOR_Z),
    )
    a = a.add(
        "drive_motor",
        motor(),
        location=base * Pos(MOTOR_X, _MOTOR_BODY_CENTER_Y, MOTOR_Z),
    )
    a = a.add(
        "drive_bracket",
        bracket(),
        location=base,
    )
    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_turret_drive(Assembly())

# Stubs.
_SHAFT_LEN = TURRET_SHAFT_TOP_Y - RAIL_TOP_Y               # = 200
assembly = assembly.add(
    "hub_shaft_stub",
    _shaft_stub(length=_SHAFT_LEN),
    location=Pos(TURRET_HUB_X, RAIL_TOP_Y, TURRET_HUB_Z),
)
assembly = assembly.add(
    "cross_rail_stub",
    _cross_rail_stub(),
)

# Pairwise no-interference. Pulley-onto-shaft is tangent contact (zero
# overlap; passes assert_no_interference). Bracket foot tangent on
# cross-rail top, bracket plate tangent on motor flange — also zero-
# overlap. Pulleys vs each other are clear in X (center distance 58.6,
# OD radii 19.1 + 6.4 = 25.5, gap 33.1 mm).
_ALL = list(DRIVE_PART_NAMES) + ["hub_shaft_stub", "cross_rail_stub"]
for _i, _a in enumerate(_ALL):
    for _b in _ALL[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: bottom face on the build plate (+Y up). All
    # bores are vertical and clean. The cantilever plate's underside
    # is a 90° overhang requiring print-time supports — accepted at
    # the 90° threshold per the slice docstring (same precedent as
    # cam, kicker, and platform_hinge_bearing's horizontal bores).
    inspect(
        bracket(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=90.0
        ),
        out="outputs",
        name="drive_bracket",
    )
