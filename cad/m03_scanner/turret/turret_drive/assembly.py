"""
M03 scanner — rev-4 turret drive coupling (slice g of the 4-station turret).

Standalone NEMA17 stepper + GT2 belt drive that turns the slice (b) shaft
in 90° increments. Driven 60T-equivalent pulley keyed to the turret
shaft sits in the 29 mm gap between the slice (b) housing top
(z = -110) and the slice (f1) cam disc bottom (z = -81); driver
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
  * drive_motor         — NEMA17 stepper stub (42.3 × 42.3 × 40 body,
                          Ø5 × 22 shaft, 4× M3 mounting holes on a
                          31 × 31 mm pattern, Ø22 centering boss).
                          Hangs below the bracket plate with shaft
                          pointing +Z up.
  * drive_bracket       — printed L-bracket: foot bolts into the
                          cross-rail's top T-slot at y = 0; vertical
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
    origin = hub axis at the cross-rail's top face Z plane
    +X     = operator's right            (parallel to world +X)
    +Y     = away from operator          (parallel to world +Y)
    +Z     = up                          (parallel to world +Z)

Belt:
The GT2 closed-loop belt at 200 mm length is a stock part and not
modeled as a solid in this assembly — its physical envelope (6 mm
height, ~1 mm thickness, routed at the pulleys' pitch radii in the X-Y
plane at z = -86) lies in clear airspace by construction. The straight
runs span x ∈ [+19.1, +52.2] approx at y = ±19.1 (driven-side wrap
tangent) connecting to the driver pulley at x = +58.6. Nothing else
sits in this y-z band.

Homing:
A photo-interrupter sensor + flag for shaft-position homing is deferred
to a follow-up slice. The flag would key to the shaft below the driven
pulley (z ∈ [-110, -90] is clear airspace for it) and the sensor would
mount on a small printed bracket next to the cross-rail. Not gating
this slice.

Print orientation (bracket): bottom face on the build plate, +Z up
(build123d default). All bores are vertical (M5 foot bolts, Ø22 motor
centering boss, 4× M3 motor mounting holes) and print as clean
through-holes. The cantilever plate's underside (-Z face from x =
BRACKET_PLATE_X_MIN out to the wall at BRACKET_WALL_X_MIN) is a
horizontal overhang at 90° from vertical and requires print-time
supports — the slicer auto-generates these for the ~50 mm × 50 mm
patch under the cantilever. We accept the 90° angle in the inspect
threshold (same precedent as the slice (f1) cam, slice (f2) kicker,
and platform_hinge_bearing for their horizontal bores). A
printable-without-supports redesign — adding a 45° gusset between the
wall and plate's underside — was rejected because the gusset would
crash the motor body in X (motor body inboard face at x ≈ +37.65,
gusset would need to extend from the wall at x = +85 inward through
the motor's X range). Pulleys are stock parts (not printed), so no
inspect call.
"""

import math
import sys
from pathlib import Path

from build123d import Box, Cylinder, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from turret.params import (  # noqa: E402
    TURRET_DUMP_CAM_BOTTOM_Z,
    TURRET_FRAME_CROSS_Y,
    TURRET_FRAME_FLOOR_Z,
    TURRET_HUB_X,
    TURRET_HUB_Y,
    TURRET_SHAFT_D,
    TURRET_SHAFT_TOP_Z,
    WALL_MIN,
)
from design_systems import color_of  # noqa: E402
from materials import material_of  # noqa: E402


# --- Cross-rail interface (private mirror of slice a) ------------------

MEMBER = 20.0                                              # 2020 extrusion
RAIL_TOP_Z = TURRET_FRAME_FLOOR_Z + MEMBER / 2             # = -170

# Slice (b) housing top (private mirror; bound assertions reference it).
# Slice c5 (2026-05-05) raised this 30 mm (-110 → -80) along with the
# floor; housing height itself unchanged (60 mm). Pulley raised
# 11 mm in lockstep (-86 → -75 center) so housing-top → pulley-bottom
# closes from 20 mm to 1 mm and pulley-top → cam-bottom stays at 1 mm.
HOUSING_TOP_Z = -80.0


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

# Pulley height (Z direction = vertical). GT2-6 belt is 6 mm tall;
# with 1 mm of flange margin per side, 8 mm pulleys are common stock.
PULLEY_HEIGHT = 8.0

# Z center: 1 mm above the c5 housing top (-80) and 1 mm below the
# c5 cam bottom (-70). Slice c5 (2026-05-05) raised this 11 mm
# (-86 → -75) so the housing-top → pulley-bottom gap closes to 1 mm
# while the pulley-top → cam-bottom gap stays at 1 mm.
PULLEY_Z_CENTER = -75.0
_PULLEY_TOP_Z = PULLEY_Z_CENTER + PULLEY_HEIGHT / 2        # = -82
_PULLEY_BOTTOM_Z = PULLEY_Z_CENTER - PULLEY_HEIGHT / 2     # = -90


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

# Body 42.3 × 42.3 × 40 (X × Y × Z), centered on the motor axis. Shaft
# extends MOTOR_SHAFT_LENGTH in +Z from the body's +Z face (= flange
# face). Motor flange has 4× M3 holes on a 31 × 31 mm pattern centered
# on the shaft, plus a Ø22 round centering boss (matched by the bracket
# plate's pass-through hole).
MOTOR_BODY_X = 42.3
MOTOR_BODY_Y = 42.3
MOTOR_BODY_Z = 40.0
MOTOR_SHAFT_LENGTH = 22.0
MOTOR_BOLT_PATTERN = 31.0
MOTOR_BOLT_HOLE_D = 3.5                                    # M3 clearance
MOTOR_BOSS_D = 22.0                                        # centering boss

# Motor X position. Center of driver pulley = motor shaft axis. Sits
# on the +X side of the hub (opposite the kicker at x = -110 on -X).
MOTOR_X = DRIVE_CENTER_DIST                                # = +58.6
MOTOR_Y = 0.0                                              # on cross-rail (depth)


# --- Bracket geometry --------------------------------------------------

# Plate (horizontal, at the motor's flange face). Top face under the
# driven pulley with a 1 mm air gap so the (real) pulley's grub-screw
# wrench can swing in.
BRACKET_PLATE_T = 8.0
BRACKET_PLATE_AIRGAP_BELOW_PULLEY = 1.0
_BRACKET_PLATE_TOP_Z = (
    _PULLEY_BOTTOM_Z - BRACKET_PLATE_AIRGAP_BELOW_PULLEY
)                                                          # = -91
_BRACKET_PLATE_BOTTOM_Z = _BRACKET_PLATE_TOP_Z - BRACKET_PLATE_T  # = -99
_BRACKET_PLATE_CENTER_Z = (_BRACKET_PLATE_TOP_Z + _BRACKET_PLATE_BOTTOM_Z) / 2

# Motor flange Z = bracket plate bottom face (motor bolted up against
# plate). Body extends MOTOR_BODY_Z further in -Z.
_MOTOR_FLANGE_Z = _BRACKET_PLATE_BOTTOM_Z                  # = -99
_MOTOR_BODY_BOTTOM_Z = _MOTOR_FLANGE_Z - MOTOR_BODY_Z      # = -139
_MOTOR_BODY_CENTER_Z = (_MOTOR_FLANGE_Z + _MOTOR_BODY_BOTTOM_Z) / 2  # = -119

# Wall (vertical, outboard of the motor body in +X). Inner face clears
# the motor body by BRACKET_WALL_AIRGAP_X.
BRACKET_WALL_AIRGAP_X = 5.0
_MOTOR_BODY_OUTBOARD_X = MOTOR_X + MOTOR_BODY_X / 2        # ≈ +79.65
BRACKET_WALL_T = 15.0                                      # X thickness
BRACKET_WALL_X_MIN = _MOTOR_BODY_OUTBOARD_X + BRACKET_WALL_AIRGAP_X
BRACKET_WALL_X_MAX = BRACKET_WALL_X_MIN + BRACKET_WALL_T
_BRACKET_WALL_CENTER_X = (BRACKET_WALL_X_MIN + BRACKET_WALL_X_MAX) / 2
BRACKET_WALL_Y = 50.0                                      # depth
_BRACKET_WALL_BOTTOM_Z = RAIL_TOP_Z + 8.0                  # foot top, = -162
_BRACKET_WALL_TOP_Z = _BRACKET_PLATE_BOTTOM_Z              # = -99
_BRACKET_WALL_HEIGHT = _BRACKET_WALL_TOP_Z - _BRACKET_WALL_BOTTOM_Z  # = 63
_BRACKET_WALL_CENTER_Z = (_BRACKET_WALL_BOTTOM_Z + _BRACKET_WALL_TOP_Z) / 2

# Plate spans from inboard edge (clears M3 holes at MOTOR_X - 15.5)
# outboard to wall outboard face.
_PLATE_X_MIN_TIGHT = (
    MOTOR_X - MOTOR_BOLT_PATTERN / 2 - MOTOR_BOLT_HOLE_D / 2 - WALL_MIN
)
BRACKET_PLATE_X_MIN = round(_PLATE_X_MIN_TIGHT - 5.0, 1)   # ≈ +35
BRACKET_PLATE_X_MAX = BRACKET_WALL_X_MAX
_BRACKET_PLATE_CENTER_X = (BRACKET_PLATE_X_MIN + BRACKET_PLATE_X_MAX) / 2
BRACKET_PLATE_X_LEN = BRACKET_PLATE_X_MAX - BRACKET_PLATE_X_MIN
BRACKET_PLATE_Y = BRACKET_WALL_Y                           # depth

# Foot. Spans wall outboard ± 30 mm in X to give two M5 bolt holes
# spaced 35 mm along the rail T-slot.
BRACKET_FOOT_X = 60.0
BRACKET_FOOT_Y = BRACKET_WALL_Y                            # depth = 50
BRACKET_FOOT_Z = 8.0                                       # vertical
_BRACKET_FOOT_CENTER_X = _BRACKET_WALL_CENTER_X
_BRACKET_FOOT_X_MIN = _BRACKET_FOOT_CENTER_X - BRACKET_FOOT_X / 2
_BRACKET_FOOT_X_MAX = _BRACKET_FOOT_CENTER_X + BRACKET_FOOT_X / 2
_BRACKET_FOOT_BOTTOM_Z = RAIL_TOP_Z                        # = -170
_BRACKET_FOOT_TOP_Z = _BRACKET_FOOT_BOTTOM_Z + BRACKET_FOOT_Z  # = -162
_BRACKET_FOOT_CENTER_Z = (_BRACKET_FOOT_BOTTOM_Z + _BRACKET_FOOT_TOP_Z) / 2

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

# Pulley Z span clears housing top + cam bottom by ≥ 1 mm.
_pulley_to_housing = _PULLEY_BOTTOM_Z - HOUSING_TOP_Z
assert _pulley_to_housing >= 1.0, (
    f"driven pulley bottom ({_PULLEY_BOTTOM_Z}) too close to housing "
    f"top ({HOUSING_TOP_Z}); gap {_pulley_to_housing:.2f} mm < 1 mm"
)
_pulley_to_cam = TURRET_DUMP_CAM_BOTTOM_Z - _PULLEY_TOP_Z
assert _pulley_to_cam >= 1.0, (
    f"driven pulley top ({_PULLEY_TOP_Z}) too close to cam bottom "
    f"({TURRET_DUMP_CAM_BOTTOM_Z}); gap {_pulley_to_cam:.2f} mm < 1 mm"
)

# Driven pulley OD doesn't crash motor body in X.
_pulley_to_motor_body_x = (
    (MOTOR_X - MOTOR_BODY_X / 2) - DRIVEN_PULLEY_OD / 2
)
assert _pulley_to_motor_body_x >= WALL_MIN, (
    f"driven pulley OD reaches +{DRIVEN_PULLEY_OD/2:.2f} mm in X; motor "
    f"body inboard face at +{MOTOR_X - MOTOR_BODY_X/2:.2f}; X gap "
    f"{_pulley_to_motor_body_x:.2f} mm < WALL_MIN ({WALL_MIN})"
)

# Driver pulley OD doesn't crash the bracket plate's M3 holes.
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

# Bracket plate Y edge has wall to M3 holes (M3 at y = ±15.5).
_plate_y_to_hole_edge = (
    BRACKET_PLATE_Y / 2 - MOTOR_BOLT_PATTERN / 2 - MOTOR_BOLT_HOLE_D / 2
)
assert _plate_y_to_hole_edge >= WALL_MIN, (
    f"bracket plate Y edge to M3 hole edge wall "
    f"{_plate_y_to_hole_edge:.2f} mm < WALL_MIN ({WALL_MIN}); raise "
    f"BRACKET_PLATE_Y"
)

# Boss hole walls — Ø22 boss in plate. Plate Y extent ±25; Y-edge wall
# = 25 - 11 = 14 mm.
_boss_to_y_edge = BRACKET_PLATE_Y / 2 - MOTOR_BOSS_D / 2
assert _boss_to_y_edge >= WALL_MIN, (
    f"boss hole to plate Y edge wall {_boss_to_y_edge:.2f} mm < "
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
# drop in from above without the wall in the way).
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
    # Default Cylinder axis +Z = vertical = correct under Z-up. No
    # rotation needed.
    body = Cylinder(od / 2, height)
    body -= Cylinder(bore_d / 2, height * 3)
    return body


def motor():
    """NEMA17 stub. Body centered on origin; shaft protrudes in +Z from
    the body's +Z face (= flange face). Caller positions the body
    center at (MOTOR_X, MOTOR_Y, _MOTOR_BODY_CENTER_Z)."""
    body = Box(MOTOR_BODY_X, MOTOR_BODY_Y, MOTOR_BODY_Z)
    shaft = (
        Pos(0, 0, MOTOR_BODY_Z / 2 + MOTOR_SHAFT_LENGTH / 2)
        * Cylinder(MOTOR_SHAFT_D / 2, MOTOR_SHAFT_LENGTH)
    )
    return body + shaft


def bracket():
    """Printed L-bracket: foot + outboard wall + horizontal cantilever
    plate. Foot bolts vertically into the cross-rail's top T-slot at
    y = 0. Plate hosts the motor's 4× M3 mounting holes plus the Ø22
    centering-boss pass-through. Returned in turret-local coords (no
    `base` shift — caller multiplies the placement)."""
    foot = (
        Pos(_BRACKET_FOOT_CENTER_X, 0, _BRACKET_FOOT_CENTER_Z)
        * Box(BRACKET_FOOT_X, BRACKET_FOOT_Y, BRACKET_FOOT_Z)
    )
    for sx in (-1, 1):
        foot -= (
            Pos(
                _BRACKET_FOOT_CENTER_X + sx * _BRACKET_BOLT_HALF_X,
                0,
                _BRACKET_FOOT_CENTER_Z,
            )
            * Cylinder(BRACKET_BOLT_HOLE_D / 2, BRACKET_FOOT_Z * 3)
        )

    wall = (
        Pos(_BRACKET_WALL_CENTER_X, 0, _BRACKET_WALL_CENTER_Z)
        * Box(BRACKET_WALL_T, BRACKET_WALL_Y, _BRACKET_WALL_HEIGHT)
    )

    plate = (
        Pos(_BRACKET_PLATE_CENTER_X, 0, _BRACKET_PLATE_CENTER_Z)
        * Box(BRACKET_PLATE_X_LEN, BRACKET_PLATE_Y, BRACKET_PLATE_T)
    )
    # Ø22 centering-boss pass-through (axis ±Z = build123d default).
    plate -= (
        Pos(MOTOR_X, 0, _BRACKET_PLATE_CENTER_Z)
        * Cylinder(MOTOR_BOSS_D / 2, BRACKET_PLATE_T * 3)
    )
    # 4× M3 mounting clearance holes on a 31 × 31 mm pattern centered
    # on the motor shaft. Axis ±Z (vertical).
    for sx in (-1, 1):
        for sy in (-1, 1):
            plate -= (
                Pos(
                    MOTOR_X + sx * MOTOR_BOLT_PATTERN / 2,
                    sy * MOTOR_BOLT_PATTERN / 2,
                    _BRACKET_PLATE_CENTER_Z,
                )
                * Cylinder(MOTOR_BOLT_HOLE_D / 2, BRACKET_PLATE_T * 3)
            )

    return foot + wall + plate


# --- Stubs for the standalone (duplicated, not imported) ---------------

def _shaft_stub(length: float):
    """Slice (b) Ø8 turret shaft mirror. Caller positions the base at
    the rail top; this function returns a cylinder centered on its
    own origin in X / Y, spanning Z ∈ [0, length]."""
    return Pos(0, 0, length / 2) * Cylinder(TURRET_SHAFT_D / 2, length)


def _cross_rail_stub(length: float = BRACKET_FOOT_X + 40.0):
    """Slice (a) cross-rail stub centered under the bracket foot.
    Length spans the foot footprint with a 20 mm margin each side so
    the bolt-line + foot-tangent contact is exercised."""
    return (
        Pos(
            _BRACKET_FOOT_CENTER_X,
            TURRET_FRAME_CROSS_Y,
            TURRET_FRAME_FLOOR_Z,
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
        location=base * Pos(TURRET_HUB_X, TURRET_HUB_Y, PULLEY_Z_CENTER),
        material=material_of("drive_pulley_shaft"),
    )
    a = a.add(
        "drive_pulley_motor",
        driver_pulley(),
        location=base * Pos(MOTOR_X, MOTOR_Y, PULLEY_Z_CENTER),
        material=material_of("drive_pulley_motor"),
    )
    a = a.add(
        "drive_motor",
        motor(),
        location=base * Pos(MOTOR_X, MOTOR_Y, _MOTOR_BODY_CENTER_Z),
        material=material_of("drive_motor"),
    )
    a = a.add(
        "drive_bracket",
        bracket(),
        location=base,
        material=material_of("drive_bracket"),
        color=color_of("drive_bracket"),
    )
    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_turret_drive(Assembly())

# Stubs.
_SHAFT_LEN = TURRET_SHAFT_TOP_Z - RAIL_TOP_Z               # = 160
assembly = assembly.add(
    "hub_shaft_stub",
    _shaft_stub(length=_SHAFT_LEN),
    location=Pos(TURRET_HUB_X, TURRET_HUB_Y, RAIL_TOP_Z),
    material="steel",
)
assembly = assembly.add(
    "cross_rail_stub",
    _cross_rail_stub(),
    material="aluminium_anodized",
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
    # Print orientation: bottom face on the build plate, +Z up
    # (build123d default). All bores are vertical and clean. The
    # cantilever plate's underside is a 90° overhang requiring print-
    # time supports — accepted at the 90° threshold per the slice
    # docstring.
    inspect(
        bracket(),
        method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
        out="outputs",
        name="drive_bracket",
    )
