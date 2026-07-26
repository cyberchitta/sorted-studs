"""
M03 scanner — DUMP servo (replacement for the dump_cam + dump_kicker
pair retired 2026-05-23 after the face-cam rollback).

Bench-mounted hobby servo (SG90 class) at the DUMP heading drives a
short lifter arm that lands a pad under the IMAGE-pose drop block at
the DUMP station. The MCU fires the servo on the rotor index pulse;
mechanical timing isn't coupled to the shaft. See
`_notes/decisions/m03-turret.md` §6.1.

Members (1 stock + 2 printed):
  * servo_body    — stock SG90-class hobby servo, modelled as a single
                    23 × 12 × 23 box centred on the output-shaft axis.
                    Output shaft axis along +Y; body extends in -Y
                    from the pivot face. Treated as bought hardware
                    (no printability inspect).
  * servo_bracket — printed L-bracket that bolts to the turret cross-
                    rail's top T-slot and cantilevers in -Y to suspend
                    the servo body below the rotor plane. Two M5 bolt
                    holes through the foot at the cross-rail centreline
                    (turret Y = 0).
  * servo_arm     — printed rigid arm: hub disc at the servo output
                    shaft + slanted beam + pad block at the tip. Rotates
                    around the servo shaft (axis along ±Y) lifting the
                    pad against the DUMP drop block from below. At rest
                    the pad's top face sits at z = TURRET_DUMP_SERVO_
                    LIFTER_TOP_Z = -45, 2 mm below the platform
                    extension's bottom at z = -43.

Coordinate frame = turret-local frame (matches `../params.py`):
    origin = hub axis at the cross-rail's top face Z plane
    +X     = operator's right            (parallel to world +X)
    +Y     = away from operator          (parallel to world +Y)
    +Z     = up                          (parallel to world +Z)

Why the pivot sits at turret (-80, -25, -100):
  * Lifter pad target XY = (-30, -25) is fixed by the platform
    extension's lift point at DUMP heading 270°.
  * The pivot can't share that X with the body, because every cross-
    rail-top mount around the hub region (homing bracket at
    X ∈ [-62.5, -31.5], hub housing at X ∈ [-30, +30], motor bracket
    at X ∈ [+29, +89]) is already occupied. Pivot X = -80 puts the
    servo body at X ∈ [-86, -74] — well clear of the homing bracket
    on -X and the hub housing on +X.
  * Pivot Z = -100 puts the body bottom at z = -112, flush with the
    8-mm-thick bracket foot sitting on rail top (z = -120). Lifter
    arm spans z = -100 → -48 at rest, ~52 mm vertical reach.

Print orientation:
  * Bracket: bottom face on the build plate, +Z up. Foot prints flat,
    cradle walls rise as plain vertical extrusions; no overhangs.
  * Arm: lay the arm beam on the build plate so the pad and hub end
    print as upstanding bosses (standard cantilever lever idiom).
    The standalone inspect uses the as-modelled orientation (arm at
    its turret-local pose); the printability check passes either way
    because every feature is a clean Box/Cylinder.
"""

import math

from build123d import Box, Cylinder, Pos, Rot

from cad_khana.mechanism.assembly import Assembly


from ..params import (
    TURRET_DUMP_SERVO_LIFTER_TOP_Z,
    TURRET_DUMP_SERVO_LIFTER_TURRET_X,
    TURRET_DUMP_SERVO_LIFTER_TURRET_Y,
    TURRET_DUMP_SERVO_PIVOT_X,
    TURRET_DUMP_SERVO_PIVOT_Y,
    TURRET_DUMP_SERVO_PIVOT_Z,
    TURRET_FRAME_CROSS_Y,
    TURRET_FRAME_FLOOR_Z,
    WALL_MIN,
)
from ..materials import material_of


# --- Cross-rail interface (private mirror of slice a) ------------------

MEMBER = 20.0                                              # 2020 extrusion
RAIL_TOP_Z = TURRET_FRAME_FLOOR_Z + MEMBER / 2             # = -120


# --- SG90-class servo body stub ----------------------------------------
#
# Nominal SG90 footprint: ~22.7 (length) × 12.0 (width) × 22.5 (height,
# incl. shaft hub). Modelled as a plain 23 × 12 × 23 box centred on the
# pivot in Y / Z; the body extends in -Y from the +Y face (which carries
# the output shaft). Flanges are not modelled — they bolt to the cradle
# walls and don't affect the integrate-level interference checks.

SERVO_BODY_X = 12.0
SERVO_BODY_Y = 24.0    # 22.5 nominal + 1.5 mm round-up so body bottom flushes with the 8-mm bracket plate
SERVO_BODY_Z = 23.0

# Output-shaft protrusion above the body's +Y face (SG90 reference:
# ~4–5 mm of splined output spindle). The arm's hub disc sits in this
# protruding zone, so the body's +Y face sits SHAFT_PROTRUSION_Y in -Y
# from the pivot.
SERVO_SHAFT_PROTRUSION_Y = 5.0

# Body centroid in turret-local. Body's +Y face at pivot.Y -
# SHAFT_PROTRUSION_Y; body extends in -Y from there. Body Z centred on
# the pivot Z so the output shaft axis lies on the body's geometric
# centre line (matches SG90's centred output spindle).
_SERVO_BODY_X_CENTER = TURRET_DUMP_SERVO_PIVOT_X
_SERVO_BODY_Y_CENTER = (
    TURRET_DUMP_SERVO_PIVOT_Y - SERVO_SHAFT_PROTRUSION_Y - SERVO_BODY_Y / 2
)
_SERVO_BODY_Z_CENTER = TURRET_DUMP_SERVO_PIVOT_Z

_SERVO_BODY_Y_MIN = _SERVO_BODY_Y_CENTER - SERVO_BODY_Y / 2  # = -54
_SERVO_BODY_Y_MAX = _SERVO_BODY_Y_CENTER + SERVO_BODY_Y / 2  # = -30
_SERVO_BODY_Z_MIN = _SERVO_BODY_Z_CENTER - SERVO_BODY_Z / 2  # = -112
_SERVO_BODY_Z_MAX = _SERVO_BODY_Z_CENTER + SERVO_BODY_Z / 2  # = -89
_SERVO_BODY_X_MIN = _SERVO_BODY_X_CENTER - SERVO_BODY_X / 2  # = -86
_SERVO_BODY_X_MAX = _SERVO_BODY_X_CENTER + SERVO_BODY_X / 2  # = -74


# --- Bracket geometry --------------------------------------------------

# Foot + cantilever: single horizontal plate on the cross-rail top,
# spanning from the rail's +Y side (Y = +12, 2 mm past rail face) back
# through the rail centreline (where bolts drop into the T-slot) and
# cantilevering in -Y to past the body's -Y face.
_BRACKET_PLATE_X = SERVO_BODY_X + 2 * 6.0                  # = 24 (cradle wall + clearance each side)
_BRACKET_PLATE_X_CENTER = _SERVO_BODY_X_CENTER             # = -80
_BRACKET_PLATE_Y_MIN = _SERVO_BODY_Y_MIN - 2.0             # = -49.5 (clear body -Y face)
_BRACKET_PLATE_Y_MAX = +12.0                               # = +12 (past rail +Y face)
_BRACKET_PLATE_Y_LEN = _BRACKET_PLATE_Y_MAX - _BRACKET_PLATE_Y_MIN
_BRACKET_PLATE_Y_CENTER = (_BRACKET_PLATE_Y_MIN + _BRACKET_PLATE_Y_MAX) / 2
_BRACKET_PLATE_T = 8.0
_BRACKET_PLATE_BOTTOM_Z = RAIL_TOP_Z                       # = -120
_BRACKET_PLATE_TOP_Z = _BRACKET_PLATE_BOTTOM_Z + _BRACKET_PLATE_T  # = -112
_BRACKET_PLATE_CENTER_Z = (_BRACKET_PLATE_BOTTOM_Z + _BRACKET_PLATE_TOP_Z) / 2

# Two M5 bolts at the rail centreline (Y = TURRET_FRAME_CROSS_Y = 0)
# dropping vertically into the cross-rail's top T-slot.
BRACKET_BOLT_HOLE_D = 5.5
BRACKET_BOLT_SPACING_X = 14.0
_BRACKET_BOLT_HALF_X = BRACKET_BOLT_SPACING_X / 2

# Two cradle walls capturing the body's ±X faces. Wall inner faces flush
# with body faces with WALL_AIR_GAP clearance; wall outer faces flush
# with bracket plate edges. Walls span the full body Y extent and rise
# from the plate top to the body top face.
_WALL_AIR_GAP = 0.4
_CRADLE_WALL_T = (_BRACKET_PLATE_X - SERVO_BODY_X) / 2 - _WALL_AIR_GAP   # = 5.6
_CRADLE_WALL_X_OFFSET = SERVO_BODY_X / 2 + _WALL_AIR_GAP + _CRADLE_WALL_T / 2
_CRADLE_WALL_Y_CENTER = _SERVO_BODY_Y_CENTER
_CRADLE_WALL_Y_LEN = SERVO_BODY_Y
_CRADLE_WALL_Z_BOTTOM = _BRACKET_PLATE_TOP_Z               # = -112
_CRADLE_WALL_Z_TOP = _SERVO_BODY_Z_MAX                     # = -88.5
_CRADLE_WALL_Z_LEN = _CRADLE_WALL_Z_TOP - _CRADLE_WALL_Z_BOTTOM
_CRADLE_WALL_Z_CENTER = (_CRADLE_WALL_Z_BOTTOM + _CRADLE_WALL_Z_TOP) / 2


# --- Arm geometry ------------------------------------------------------

# Pivot → pad geometry, all in turret-local.
_PAD_CENTER_X = TURRET_DUMP_SERVO_LIFTER_TURRET_X          # = -30
_PAD_CENTER_Y = TURRET_DUMP_SERVO_LIFTER_TURRET_Y          # = -25
PAD_X_LEN = 22.0      # covers drop block X ∈ [-39, -21] (18 mm) + margin
PAD_Y_LEN = 26.0      # covers drop block Y ∈ [-36, -14] (22 mm) + margin
PAD_Z_LEN = 6.0
_PAD_TOP_Z = TURRET_DUMP_SERVO_LIFTER_TOP_Z                # = -45
_PAD_CENTER_Z = _PAD_TOP_Z - PAD_Z_LEN / 2                 # = -48
_PAD_BOTTOM_Z = _PAD_TOP_Z - PAD_Z_LEN                     # = -51

# Beam vector pivot → pad in X-Z (Y locked to the pad / pivot Y plane).
_BEAM_DX = _PAD_CENTER_X - TURRET_DUMP_SERVO_PIVOT_X       # = +50
_BEAM_DZ = _PAD_CENTER_Z - TURRET_DUMP_SERVO_PIVOT_Z       # = +52
_BEAM_LEN = math.hypot(_BEAM_DX, _BEAM_DZ)                 # ≈ 72.14

# Beam orientation: angle from +X-axis in the X-Z plane. build123d's
# default Box long axis is +X; Rot(0, -θ, 0) maps +X to (cos θ, 0, +sin θ),
# which is the (+X, +Z) direction we want. θ = atan2(dz, dx).
_BEAM_ANGLE_DEG = math.degrees(math.atan2(_BEAM_DZ, _BEAM_DX))   # ≈ 46.12

# Beam cross-section (Y × perpendicular). Y stays along world ±Y after
# the Y-axis rotation; the "Z" Box dimension becomes the in-plane beam
# thickness.
BEAM_Y_LEN = 8.0
BEAM_THICK = 6.0
_BEAM_CENTER_X = (TURRET_DUMP_SERVO_PIVOT_X + _PAD_CENTER_X) / 2  # = -55
_BEAM_CENTER_Y = TURRET_DUMP_SERVO_PIVOT_Y                        # = -25
_BEAM_CENTER_Z = (TURRET_DUMP_SERVO_PIVOT_Z + _PAD_CENTER_Z) / 2  # = -74

# Hub disc: cylinder on the servo output shaft. Bore Ø4.5 over SG90's
# Ø5 splined output spindle (real part heat-sets the included horn).
HUB_DISC_OD = 16.0
HUB_DISC_Y_LEN = 8.0
HUB_BORE_D = 4.5


# --- Part functions ----------------------------------------------------

def servo_body():
    """Stock SG90-class servo body stub. Centroid placed in
    `build_turret_dump_servo_static`; returned at part-local identity
    here."""
    return Box(SERVO_BODY_X, SERVO_BODY_Y, SERVO_BODY_Z)


def servo_bracket():
    """Printed bracket: foot plate sitting on the cross-rail top with
    two M5 bolt holes at the rail centreline, plus a pair of cradle
    walls hugging the servo body in ±X. Returned in turret-local
    coords (placed at identity in the assembly)."""
    # Bolt-hole and cradle wall sanity. WALL_MIN arithmetic over
    # subtracted negative space plus a containment bound on the plate's
    # Y span, so none is a part-pair claim — bare, fired when the
    # bracket is cut.
    _bolt_to_foot_x_inner = (
        _BRACKET_PLATE_X / 2 - _BRACKET_BOLT_HALF_X - BRACKET_BOLT_HOLE_D / 2
    )
    assert _bolt_to_foot_x_inner >= WALL_MIN, (
        f"bracket bolt edge to foot X-edge wall {_bolt_to_foot_x_inner:.2f} mm "
        f"< WALL_MIN ({WALL_MIN}); raise _BRACKET_PLATE_X or shrink "
        f"BRACKET_BOLT_SPACING_X"
    )
    # Bracket Y-extent: bolt at rail centreline must sit inside the plate.
    assert _BRACKET_PLATE_Y_MIN < TURRET_FRAME_CROSS_Y < _BRACKET_PLATE_Y_MAX, (
        f"bracket plate Y range [{_BRACKET_PLATE_Y_MIN}, {_BRACKET_PLATE_Y_MAX}] "
        f"doesn't contain rail centreline Y = {TURRET_FRAME_CROSS_Y}; "
        f"M5 bolts wouldn't drop into the T-slot"
    )
    assert _CRADLE_WALL_T >= WALL_MIN, (
        f"cradle wall thickness {_CRADLE_WALL_T:.2f} mm < WALL_MIN "
        f"({WALL_MIN}); raise _BRACKET_PLATE_X or shrink _WALL_AIR_GAP"
    )

    plate = (
        Pos(_BRACKET_PLATE_X_CENTER, _BRACKET_PLATE_Y_CENTER, _BRACKET_PLATE_CENTER_Z)
        * Box(_BRACKET_PLATE_X, _BRACKET_PLATE_Y_LEN, _BRACKET_PLATE_T)
    )
    for sx in (-1, 1):
        plate -= (
            Pos(
                _BRACKET_PLATE_X_CENTER + sx * _BRACKET_BOLT_HALF_X,
                TURRET_FRAME_CROSS_Y,
                _BRACKET_PLATE_CENTER_Z,
            )
            * Cylinder(BRACKET_BOLT_HOLE_D / 2, _BRACKET_PLATE_T * 3)
        )

    for sx in (-1, 1):
        plate += (
            Pos(
                _BRACKET_PLATE_X_CENTER + sx * _CRADLE_WALL_X_OFFSET,
                _CRADLE_WALL_Y_CENTER,
                _CRADLE_WALL_Z_CENTER,
            )
            * Box(_CRADLE_WALL_T, _CRADLE_WALL_Y_LEN, _CRADLE_WALL_Z_LEN)
        )

    return plate


def servo_arm():
    """Printed lifter arm: hub disc at the servo shaft + slanted beam +
    pad block at the tip. Single rigid body; the whole part rotates
    about the servo Y-axis through the pivot."""
    # Hub bore clears the shaft with a single-wall-thick remainder.
    _hub_wall = (HUB_DISC_OD - HUB_BORE_D) / 2
    assert _hub_wall >= WALL_MIN, (
        f"hub disc wall around shaft bore {_hub_wall:.2f} mm < WALL_MIN "
        f"({WALL_MIN}); raise HUB_DISC_OD"
    )

    hub = (
        Pos(
            TURRET_DUMP_SERVO_PIVOT_X,
            TURRET_DUMP_SERVO_PIVOT_Y,
            TURRET_DUMP_SERVO_PIVOT_Z,
        )
        * Rot(90, 0, 0)
        * Cylinder(HUB_DISC_OD / 2, HUB_DISC_Y_LEN)
    )
    hub -= (
        Pos(
            TURRET_DUMP_SERVO_PIVOT_X,
            TURRET_DUMP_SERVO_PIVOT_Y,
            TURRET_DUMP_SERVO_PIVOT_Z,
        )
        * Rot(90, 0, 0)
        * Cylinder(HUB_BORE_D / 2, HUB_DISC_Y_LEN * 3)
    )

    beam = (
        Pos(_BEAM_CENTER_X, _BEAM_CENTER_Y, _BEAM_CENTER_Z)
        * Rot(0, -_BEAM_ANGLE_DEG, 0)
        * Box(_BEAM_LEN, BEAM_Y_LEN, BEAM_THICK)
    )

    pad = (
        Pos(_PAD_CENTER_X, _PAD_CENTER_Y, _PAD_CENTER_Z)
        * Box(PAD_X_LEN, PAD_Y_LEN, PAD_Z_LEN)
    )

    return hub + beam + pad


# --- Stubs for the standalone (duplicated, not imported) ---------------

def _cross_rail_stub(length: float = 60.0):
    """Slice (a) cross-rail stub under the bracket foot — duplicated
    rather than imported per cad/CLAUDE.md principle 2."""
    return (
        Pos(_BRACKET_PLATE_X_CENTER, TURRET_FRAME_CROSS_Y, TURRET_FRAME_FLOOR_Z)
        * Box(length, MEMBER, MEMBER)
    )


# --- Assembly builder --------------------------------------------------

SERVO_STATIC_PART_NAMES = ("servo_bracket", "servo_body")
SERVO_ARM_PART_NAMES = ("servo_arm",)
SERVO_PART_NAMES = SERVO_STATIC_PART_NAMES + SERVO_ARM_PART_NAMES


def build_turret_dump_servo_static() -> Assembly:
    """Static servo sub-assembly: printed bracket + bought servo body,
    in turret-local coordinates. Both bolt to the bench cross-rail;
    the arm (which pivots on the servo output shaft) is a separate
    builder so the m03 top level can wrap it in a jointed
    sub-assembly."""
    a = Assembly()
    a = a.with_part(
        "servo_bracket", servo_bracket(),
        material=material_of("servo_bracket"),
    )
    a = a.with_part(
        "servo_body",
        servo_body(),
        location=Pos(
            _SERVO_BODY_X_CENTER,
            _SERVO_BODY_Y_CENTER,
            _SERVO_BODY_Z_CENTER,
        ),
        material=material_of("servo_body"),
    )
    # The servo body seats flush on the bracket plate's top face. This
    # was a `_SERVO_BODY_Z_MIN == _BRACKET_PLATE_TOP_Z` constant
    # comparison; as a contact assertion it is measured on the placed
    # geometry, and it closes the gap the module docstring flagged —
    # a body floating 3 mm above the plate passes no-interference just
    # as happily, so nothing asserted that the seat *exists*.
    a = a.assert_tangent_contact("servo_bracket", "servo_body")
    # Pad-to-platform clearance at REST. `_PLATFORM_EXT_BOTTOM_Z` is a
    # mirror of the platform extension's bottom; the claim is scoped to
    # the platforms' rest tilt, so it stays bare for the same reason as
    # the homing drop-block bound (see `homing/assembly.py`) — a
    # directed-distance form goes red mid-dump-dwell, when the dumping
    # platform tips through the same Z band without ever touching.
    _platform_ext_bottom_z = -43.0
    _pad_to_extension = _platform_ext_bottom_z - _PAD_TOP_Z
    assert _pad_to_extension >= 2.0 - 1e-9, (
        f"pad top ({_PAD_TOP_Z}) too close to platform extension bottom "
        f"({_platform_ext_bottom_z}); REST clearance "
        f"{_pad_to_extension:.2f} mm < 2 mm"
    )
    return a


def build_turret_dump_servo_arm() -> Assembly:
    """Just the arm — the rigid body that swings on the servo shaft —
    as its own sub-assembly, so the m03 top level can attach the
    revolute joint around the pivot axis."""
    return Assembly().with_part(
        "servo_arm", servo_arm(),
        material=material_of("servo_arm"),
    )


# --- Standalone assembly -----------------------------------------------

def build_turret_dump_servo_standalone() -> Assembly:
    """Both servo subtrees plus a cross-rail stub to seat the bracket
    foot on.

    All pairwise no-interference between servo parts and the rail stub,
    plus the bracket foot's tangent rest on the rail top. (The body's
    seat on the bracket is declared in
    ``build_turret_dump_servo_static``, so it propagates into composed
    runs too.) Both rests used to be constant comparisons in this
    module's sanity block — no-interference alone accepts a floating
    part, so it never asserted the joints close.
    """
    all_parts = (
        [f"static.{n}" for n in SERVO_STATIC_PART_NAMES]
        + [f"arm.{n}" for n in SERVO_ARM_PART_NAMES]
        + ["cross_rail_stub"]
    )
    return (
        Assembly()
        .with_subassembly("static", build_turret_dump_servo_static())
        .with_subassembly("arm", build_turret_dump_servo_arm())
        .with_part(
            "cross_rail_stub", _cross_rail_stub(),
            material="aluminium_anodized",
        )
        .assert_no_interference_within(all_parts)
        .assert_tangent_contact("static.servo_bracket", "cross_rail_stub")
    )


# Degenerate master, so `khana check <file>` resolves with no `:factory`.
assembly = build_turret_dump_servo_standalone()
