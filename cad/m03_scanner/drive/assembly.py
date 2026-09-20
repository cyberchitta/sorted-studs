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
2026-04-30 turret discussion (see `_notes/archive/m03-turret-alternatives.md` and
`_notes/decisions/m03-turret.md` §6.5): GT2 belt is the maker-default low-risk
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
                          set-screw on a flat. Bore Ø7.98 — the true
                          light press fit on the Ø8 shaft, modeled at
                          its interference and declared with
                          `assert_allowed_contact`.
  * drive_pulley_motor  — stock 20T GT2 metal pulley keyed to the
                          NEMA17 shaft. Same idiom as the driven
                          pulley.
  * drive_motor         — NEMA17 stepper stub (42.3 × 42.3 × 40 body,
                          Ø5 × 22 shaft, 4× M3 mounting holes on a
                          31 × 31 mm pattern, Ø22 centering boss).
                          Hangs below the bracket plate with shaft
                          pointing +Z up.
  * drive_bracket       — printed Z-bracket: foot bolts into the
                          cross-rail's top T-slot at y = 0 with both
                          M5 heads outboard of the wall (key access
                          from above with the motor mounted); vertical
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

from build123d import Box, Cylinder, Pos

from cad_khana.mechanism.assembly import Assembly


from ..params import (  # noqa: E402
    TURRET_FRAME_CROSS_Y,
    TURRET_FRAME_FLOOR_Z,
    TURRET_HUB_X,
    TURRET_HUB_Y,
    TURRET_SHAFT_D,
    TURRET_SHAFT_TOP_Z,
    WALL_MIN,
)

from ..materials import material_of  # noqa: E402


# --- Cross-rail interface (private mirror of slice a) ------------------

MEMBER = 20.0                                              # 2020 extrusion
RAIL_TOP_Z = TURRET_FRAME_FLOOR_Z + MEMBER / 2             # = -120 after c7

# The pulley's Z corridor between the slice (b) housing top and the
# slice (c) spider bottom is asserted at the m03 root
# (`pulley_housing_gap` / `pulley_spider_gap` in `build_turret`),
# measured on the real neighbours — no private mirrors of their Z
# extents live here anymore.


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

# Z center: 1 mm above the c7 housing top (-60) and 1 mm below the
# c7 cam bottom (-50). Slice c5 (2026-05-05) raised this 11 mm
# (-86 → -75) so the housing-top → pulley-bottom gap closes to 1 mm
# while the pulley-top → cam-bottom gap stays at 1 mm. Slice c7
# (2026-05-05) raised it another 20 mm (-75 → -55) in lockstep with
# the housing rise; both gaps stay at 1 mm.
PULLEY_Z_CENTER = -55.0
_PULLEY_BOTTOM_Z = PULLEY_Z_CENTER - PULLEY_HEIGHT / 2     # = -59


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

# Driven pulley: light press fit on the Ø8 turret shaft. 0.02 mm
# diametral interference sits mid-band for ISO H7/p6 at Ø8 — the pulley
# drives on under an arbor press and the M5 set-screw against the shaft
# flat is the anti-rotation backup, not the primary grip. Modeled at
# the true interference and declared with `assert_allowed_contact`; the
# previous Ø8.4 "clearance" bore existed only to keep
# assert_no_interference quiet, which left the model lying about the fit.
PULLEY_SHAFT_PRESS_FIT = 0.02                              # diametral
PULLEY_SHAFT_BORE_D = TURRET_SHAFT_D - PULLEY_SHAFT_PRESS_FIT   # = 7.98

# Driver pulley: Ø5 NEMA17 shaft + 0.4 mm clearance — a slip fit by
# design (the pulley comes off the stepper shaft routinely; the set
# screw on the shaft flat carries the torque).
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
)                                                          # = -60
_BRACKET_PLATE_BOTTOM_Z = _BRACKET_PLATE_TOP_Z - BRACKET_PLATE_T  # = -68
_BRACKET_PLATE_CENTER_Z = (_BRACKET_PLATE_TOP_Z + _BRACKET_PLATE_BOTTOM_Z) / 2

# Motor flange Z = bracket plate bottom face (motor bolted up against
# plate). Body extends MOTOR_BODY_Z further in -Z.
_MOTOR_FLANGE_Z = _BRACKET_PLATE_BOTTOM_Z                  # = -68
_MOTOR_BODY_BOTTOM_Z = _MOTOR_FLANGE_Z - MOTOR_BODY_Z      # = -108
_MOTOR_BODY_CENTER_Z = (_MOTOR_FLANGE_Z + _MOTOR_BODY_BOTTOM_Z) / 2  # = -88

# Wall (vertical, outboard of the motor body in +X). Inner face clears
# the motor body by BRACKET_WALL_AIRGAP_X.
BRACKET_WALL_AIRGAP_X = 5.0
_MOTOR_BODY_OUTBOARD_X = MOTOR_X + MOTOR_BODY_X / 2        # ≈ +79.65
BRACKET_WALL_T = 15.0                                      # X thickness
BRACKET_WALL_X_MIN = _MOTOR_BODY_OUTBOARD_X + BRACKET_WALL_AIRGAP_X
BRACKET_WALL_X_MAX = BRACKET_WALL_X_MIN + BRACKET_WALL_T
_BRACKET_WALL_CENTER_X = (BRACKET_WALL_X_MIN + BRACKET_WALL_X_MAX) / 2
BRACKET_WALL_Y = 50.0                                      # depth
_BRACKET_WALL_BOTTOM_Z = RAIL_TOP_Z + 8.0                  # foot top, = -112
_BRACKET_WALL_TOP_Z = _BRACKET_PLATE_BOTTOM_Z              # = -68
_BRACKET_WALL_HEIGHT = _BRACKET_WALL_TOP_Z - _BRACKET_WALL_BOTTOM_Z  # = 44
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

# Foot. A short toe inboard of the wall, then the foot runs outboard
# past two M5 bolt holes along the rail's T-slot — both bolts OUTBOARD
# of the wall. The original layout straddled the wall (one bolt each
# side, 35 mm apart); the inboard bolt then sat in the 5 mm strip
# between the motor body and the wall, where an M5 head (Ø8.7 × 5)
# neither fits nor can be driven with the motor mounted. Found red the
# first time the detail tier was checked (2026-09-16); moved outboard
# the same day. With both heads outboard, a hex key comes straight
# down past the wall's outboard face with the motor in place.
#
# Loads are small: the belt pull at z ≈ -86 tips the bracket inboard
# about the toe's inner edge, and the two outboard bolts hold it in
# tension at 22 / 40 mm lever arms — tens of newtons on an M5.
BRACKET_FOOT_TOE_X = 10.0                                  # inboard of the wall
BRACKET_FOOT_Y = BRACKET_WALL_Y                            # depth = 50
BRACKET_FOOT_Z = 8.0                                       # vertical
_BRACKET_FOOT_BOTTOM_Z = RAIL_TOP_Z                        # = -120
_BRACKET_FOOT_TOP_Z = _BRACKET_FOOT_BOTTOM_Z + BRACKET_FOOT_Z  # = -112
_BRACKET_FOOT_CENTER_Z = (_BRACKET_FOOT_BOTTOM_Z + _BRACKET_FOOT_TOP_Z) / 2

BRACKET_BOLT_HOLE_D = 5.5                                  # M5 clearance
BRACKET_BOLT_HEAD_D = 8.7                                  # ISO 4762 M5 head (bd_warehouse)
BRACKET_BOLT_KEY_CLEARANCE_X = 2.0                         # head edge to wall face
BRACKET_BOLT_SPACING_X = 18.0
BRACKET_BOLT_INNER_X = (
    BRACKET_WALL_X_MAX + BRACKET_BOLT_HEAD_D / 2 + BRACKET_BOLT_KEY_CLEARANCE_X
)                                                          # ≈ +106.1
BRACKET_BOLT_OUTER_X = BRACKET_BOLT_INNER_X + BRACKET_BOLT_SPACING_X  # ≈ +124.1
BRACKET_BOLT_XS = (BRACKET_BOLT_INNER_X, BRACKET_BOLT_OUTER_X)  # inner, outer
BRACKET_FOOT_END_MARGIN_X = 5.0                            # hole edge to foot end
_BRACKET_FOOT_X_MIN = BRACKET_WALL_X_MIN - BRACKET_FOOT_TOE_X          # ≈ +74.75
_BRACKET_FOOT_X_MAX = (
    BRACKET_BOLT_OUTER_X + BRACKET_BOLT_HOLE_D / 2 + BRACKET_FOOT_END_MARGIN_X
)                                                          # ≈ +131.85
BRACKET_FOOT_X = _BRACKET_FOOT_X_MAX - _BRACKET_FOOT_X_MIN
_BRACKET_FOOT_CENTER_X = (_BRACKET_FOOT_X_MIN + _BRACKET_FOOT_X_MAX) / 2


# --- Part functions ----------------------------------------------------

def driven_pulley(
    od: float = DRIVEN_PULLEY_OD,
    height: float = PULLEY_HEIGHT,
    bore_d: float = PULLEY_SHAFT_BORE_D,
):
    """Stock 60T GT2 metal pulley keyed to the turret shaft. Modeled
    as a simple cylinder at pitch diameter with a Ø7.98 central bore —
    the press fit's true interference on the Ø8 shaft, not a fudged
    clearance. Real part has helical GT2 teeth on the OD and an M5
    set-screw against a flat on the shaft."""
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
    # Hole-edge and plate-edge wall sanity for this body. Every one is
    # WALL_MIN arithmetic over subtracted negative space (M3 pattern,
    # Ø22 boss, foot bolt pattern), so none is expressible as a
    # part-pair assertion — bare, and fired when the bracket is cut.
    _m3_hole_radius = math.sqrt(2) * MOTOR_BOLT_PATTERN / 2
    _driver_to_m3 = _m3_hole_radius - MOTOR_BOLT_HOLE_D / 2 - DRIVER_PULLEY_OD / 2
    assert _driver_to_m3 >= WALL_MIN, (
        f"driver pulley OD edge {DRIVER_PULLEY_OD/2:.2f} too close to M3 "
        f"hole edge at radius {_m3_hole_radius - MOTOR_BOLT_HOLE_D/2:.2f}; "
        f"gap {_driver_to_m3:.2f} mm < WALL_MIN ({WALL_MIN})"
    )
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
    # Boss hole walls — Ø22 boss in plate. Plate Y extent ±25; Y-edge
    # wall = 25 - 11 = 14 mm.
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
    # Both bolt heads stand outboard of the wall with hex-key clearance
    # from above (by construction; guard the derivation).
    _inner_head_edge_x = BRACKET_BOLT_INNER_X - BRACKET_BOLT_HEAD_D / 2
    assert _inner_head_edge_x >= BRACKET_WALL_X_MAX + BRACKET_BOLT_KEY_CLEARANCE_X - 1e-9, (
        f"inner bolt head edge {_inner_head_edge_x:.2f} closer than "
        f"{BRACKET_BOLT_KEY_CLEARANCE_X} mm to the wall outboard face "
        f"{BRACKET_WALL_X_MAX:.2f}"
    )
    # Two heads don't touch each other along the slot.
    assert BRACKET_BOLT_SPACING_X >= BRACKET_BOLT_HEAD_D + WALL_MIN, (
        f"bolt spacing {BRACKET_BOLT_SPACING_X} leaves no gap between "
        f"Ø{BRACKET_BOLT_HEAD_D} heads"
    )
    # The foot's inner edge is the tipping fulcrum; keep a real toe.
    assert BRACKET_FOOT_TOE_X >= WALL_MIN

    foot = (
        Pos(_BRACKET_FOOT_CENTER_X, 0, _BRACKET_FOOT_CENTER_Z)
        * Box(BRACKET_FOOT_X, BRACKET_FOOT_Y, BRACKET_FOOT_Z)
    )
    for bolt_x in BRACKET_BOLT_XS:
        foot -= (
            Pos(bolt_x, 0, _BRACKET_FOOT_CENTER_Z)
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

DRIVE_STATIC_PART_NAMES = (
    "drive_pulley_motor",
    "drive_motor",
    "drive_bracket",
)
DRIVE_ROTOR_PART_NAMES = ("drive_pulley_shaft",)
DRIVE_PART_NAMES = DRIVE_ROTOR_PART_NAMES + DRIVE_STATIC_PART_NAMES

# Detail-only fasteners, added into this unit by
# `..detail_variations.apply_default_detail`. Public because the turret
# level asserts the same two bolts against the real cross-rail, and a
# second copy of the names there would be a mirror to keep in step.
DETAIL_FOOT_BOLT_NAMES = ("drive_foot_bolt_inner", "drive_foot_bolt_outer")


def build_turret_drive_static() -> Assembly:
    """Static drive sub-assembly: motor + driver pulley (on the motor
    shaft) + motor bracket, in turret-local coordinates. The motor
    pulley turns in real life, but only the driven-side pulley is
    locked to the hub shaft we animate — the motor and its pulley
    render as static for the animation."""
    # Driven pulley OD must not crash the motor body in X. WALL_MIN
    # arithmetic against the motor's placement constants — the two
    # parts live in different subtrees (rotor vs static), so there is
    # no level below the turret that could measure the pair. Bare.
    _pulley_to_motor_body_x = (
        (MOTOR_X - MOTOR_BODY_X / 2) - DRIVEN_PULLEY_OD / 2
    )
    assert _pulley_to_motor_body_x >= WALL_MIN, (
        f"driven pulley OD reaches +{DRIVEN_PULLEY_OD/2:.2f} mm in X; motor "
        f"body inboard face at +{MOTOR_X - MOTOR_BODY_X/2:.2f}; X gap "
        f"{_pulley_to_motor_body_x:.2f} mm < WALL_MIN ({WALL_MIN})"
    )

    a = Assembly()
    a = a.with_part(
        "drive_pulley_motor",
        driver_pulley(),
        location=Pos(MOTOR_X, MOTOR_Y, PULLEY_Z_CENTER),
        material=material_of("drive_pulley_motor"),
    )
    a = a.with_part(
        "drive_motor",
        motor(),
        location=Pos(MOTOR_X, MOTOR_Y, _MOTOR_BODY_CENTER_Z),
        material=material_of("drive_motor"),
    )
    a = a.with_part(
        "drive_bracket",
        bracket(),
        material=material_of("drive_bracket"),
    )
    # The motor hangs from the bracket: its flange face bolts flat
    # against the cantilever plate's underside (both at z = -68). The
    # rest must exist — a plate floating above the flange would pass
    # assert_no_interference just as happily — so declare it here,
    # where both parts live, and let it propagate into every composed
    # run.
    a = a.assert_tangent_contact("drive_bracket", "drive_motor")
    # Belt length at the chosen centre distance must land on a stock
    # length. The belt is not modelled, so there is no part pair to
    # measure — but recording the derived length beats a bare assert:
    # it lands in mechanism.json and `khana diff` reports it drifting
    # toward the tolerance before it crosses it.
    a = a.assert_scalar(
        "belt_length_error",
        abs(_BELT_L_DERIVED - DRIVE_BELT_LENGTH),
        le=0.5,
        detail=(
            f"belt length at C = {DRIVE_CENTER_DIST} mm derives to "
            f"{_BELT_L_DERIVED:.2f} mm against {DRIVE_BELT_LENGTH} mm "
            f"stock; adjust DRIVE_CENTER_DIST or pick another stock length"
        ),
    )

    # Detail-tier fastener assertions — the two M5 foot bolts against
    # the parts this unit owns. The bolts exist only once
    # `..detail_variations.apply_default_detail` adds them; in a cheap
    # run they are absent and these skip (`passed: null`). Declared
    # *here*, in the unit that owns the bolted joint, rather than in
    # the standalone fixture: the detail bundle keys the addition by
    # this unit's path, so the claims qualify and evaluate at every
    # composed root — the m03 unit and the whole machine included. In
    # the standalone they were invisible above the drive.
    # Bolt shank rides the foot's M5 clearance hole (bored out — zero
    # overlap). The motor body hangs below the plate inboard of the
    # wall; both heads stand outboard of it. Red on the original
    # straddle layout the first time it was evaluated (2026-09-16:
    # inner head 1 mm into the motor body); green since the bolts
    # moved outboard the same day.
    for bolt in DETAIL_FOOT_BOLT_NAMES:
        a = a.assert_no_interference(bolt, "drive_bracket")
        a = a.assert_no_interference(bolt, "drive_motor")
    return a


def build_turret_drive_rotor() -> Assembly:
    """Rotor drive sub-assembly: the driven pulley (keyed to the hub
    shaft), in turret-local coordinates."""
    return Assembly().with_part(
        "drive_pulley_shaft",
        driven_pulley(),
        location=Pos(TURRET_HUB_X, TURRET_HUB_Y, PULLEY_Z_CENTER),
        material=material_of("drive_pulley_shaft"),
    )


# --- Standalone assembly -----------------------------------------------

_SHAFT_LEN = TURRET_SHAFT_TOP_Z - RAIL_TOP_Z               # = 160



def build_turret_drive_standalone() -> Assembly:
    """Both drive subtrees plus the shaft and cross-rail stubs the
    press fit and the bolted foot joint are declared against."""
    a = (
        Assembly()
        .with_subassembly("static", build_turret_drive_static())
        .with_subassembly("rotor", build_turret_drive_rotor())
        .with_part(
            "hub_shaft_stub",
            _shaft_stub(length=_SHAFT_LEN),
            location=Pos(TURRET_HUB_X, TURRET_HUB_Y, RAIL_TOP_Z),
            material="steel",
        )
        .with_part(
            "cross_rail_stub",
            _cross_rail_stub(),
            material="aluminium_anodized",
        )
        # Press fit, declared at its true interference. The 0.02 mm
        # diametral band gives a Ø7.98/Ø8.0 annulus 8 mm tall ≈ 2.01 mm³
        # of overlap (measured 2.0081 mm³, 2026-07-25); the bounds
        # bracket that at ±25%, i.e. 0.015–0.025 mm of interference.
        # Below the floor the fit has drifted back to a clearance fit
        # and the set screw is carrying the turret alone; above the
        # ceiling it's a hydraulic-press job on a stock pulley.
        .assert_allowed_contact(
            "rotor.drive_pulley_shaft", "hub_shaft_stub",
            min_overlap_mm3=1.5, max_overlap_mm3=2.6,
            reason=f"press fit, {PULLEY_SHAFT_PRESS_FIT} mm diametral on Ø8",
        )
    )

    # Pairwise no-interference. The tangent rests (bracket foot on the
    # cross-rail top, bracket plate on the motor flange) have zero
    # overlap and stay in the group check; their *contact* is asserted
    # separately below / in `build_turret_drive_static`. The driven
    # pulley genuinely overlaps the shaft — it's a press fit — and the
    # group form derives that exclusion from the `assert_allowed_contact`
    # above, which is why this call comes *after* it: the skip set is
    # read at expansion time, so a same-level contact declared later
    # would not be seen. Pulleys vs each other are clear in X (center
    # distance 58.6, OD radii 19.1 + 6.4 = 25.5, gap 33.1 mm).
    all_parts = (
        [f"rotor.{n}" for n in DRIVE_ROTOR_PART_NAMES]
        + [f"static.{n}" for n in DRIVE_STATIC_PART_NAMES]
        + ["hub_shaft_stub", "cross_rail_stub"]
    )
    a = a.assert_no_interference_within(all_parts)

    # The bracket foot rests on the cross-rail's top face (both at
    # z = -120) — the M5 bolts pull the joint closed, they don't span a
    # gap. Declared here against the standalone's rail stub; the same
    # claim against the real `frame.floor_cross_x` lives at the m03
    # root, where the drive and frame subtrees meet.
    a = a.assert_tangent_contact("static.drive_bracket", "cross_rail_stub")

    return a


# Degenerate master, so `khana check <file>` resolves with no `:factory`.
assembly = build_turret_drive_standalone()
