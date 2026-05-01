"""
M03 scanner — rev-4 turret photo-interrupter homing (slice h of the
4-station turret).

Standalone shaft-position homing: a printed flag disc keyed to the
slice (b) shaft below the slice (g) driven pulley, plus a bought
slotted optical interrupter on a printed L-bracket. Firmware homes
once at startup (rotate until the sensor sees the detect feature) and
step-counts thereafter; without this, the stepper has no absolute
reference and a power cycle disagrees with the indexed station
positions.

Layout: flag disc + sensor live in the 20 mm of clear airspace between
the slice (b) housing top (y = -110) and the slice (g) driven pulley
bottom (y = -90). The bracket bolts to the slice (a) cross-rail's top
T-slot at -X side, in the gap between the hub housing (x_max = +30)
and the slice (f2) kicker mount foot (x_min = -80). Sensor azimuth at
heading 180° (-X direction); detect feature on the disc at the same
azimuth at home.

Members (3 functional + 2 stubs):
  * homing_flag       — printed disc (Ø32 × 3 mm thick, Ø8.4 shaft
                        clearance bore, single Ø4 detect hole at radius
                        12 from axis at heading 180°). Keyed to the
                        shaft via M3 set screw on a flat (real part);
                        modeled as clearance bore so
                        assert_no_interference reads tangent contact.
                        Y-center -100 (mid-airspace; 8.5 mm clear of
                        both housing top and pulley bottom).
  * homing_sensor     — slotted opto-interrupter stub (bought part).
                        Modeled as a body with two prongs above + below
                        the disc plane and a 3-mm-tall slot between
                        them; main block sits outboard at x ∈ [-40,
                        -28], prongs reach in to x = -10 (6 mm past
                        disc OD at radius 16). Beam axis (not modeled)
                        crosses the slot at (-12, *, 0) — aligned with
                        the detect hole at home.
  * homing_bracket    — printed L-bracket: foot bolts to cross-rail's
                        top T-slot at z = 0 with two M5 bolts spaced
                        22.5 mm along the rail; vertical post rises to
                        sensor body Y. Sensor mounts to post's +X face
                        (toward axis). No horizontal cantilever — post
                        ends at sensor top face, sensor hangs from
                        outboard side.

For-standalone-only stubs (1 each, duplicated rather than sibling-
imported per `cad/CLAUDE.md` principle 2):
  * hub_shaft_stub    — slice (b) Ø8 turret shaft, used for the
                        flag-bore clearance check.
  * cross_rail_stub   — slice (a) cross-rail under the bracket foot,
                        for the bolt-line + foot tangent check.

Coordinate frame = turret-local frame (matches `../params.py`):
    origin = hub axis at the cross-rail's top face Y plane
    +X     = operator's right            (parallel to world +X)
    +Y     = up                          (parallel to world +Y)
    +Z     = away from operator          (parallel to world +Z)

Detect feature design: Ø4 hole at radius 12 from axis at heading 180°
(the home azimuth). The beam (LED in top prong → photodiode in bottom
prong, beam axis vertical at (-12, *, 0)) is blocked by disc material
at all azimuths except home, where the hole aligns with the beam and
photodiode sees light. The hole rotates with the disc; firmware homes
once at startup, then step-counts. The disc is set-screwed to a flat
on the shaft, so the rotational offset between disc-home and arm-
indexed-position can be set during physical assembly; this slice ships
the disc at its home rotation (hole at heading 180°).

Print orientation (flag): bottom face on build plate, +Y up. All
features (OD, bore, detect hole) are vertical bores — clean print.
Print orientation (bracket): bottom face on build plate, +Y up. Foot
prints flat, post is a vertical extrusion. No horizontal overhangs;
default 45° threshold suffices (no need to lift to 90° as drive_bracket
did, since this bracket has no cantilever plate).
"""

import sys
from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from turret.params import (  # noqa: E402
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


# --- Adjacent-slice mirrors (private; bound assertions reference these) -

# Slice (b) housing top face Y. Flag must clear this with ≥ 1 mm gap.
HOUSING_TOP_Y = -110.0

# Slice (b) housing X half-extent. Bracket foot inner edge must clear
# this with ≥ WALL_MIN gap.
HOUSING_HALF_X = 30.0

# Slice (g) driven pulley bottom face Y. Flag + sensor must clear this
# with ≥ 1 mm gap.
PULLEY_BOTTOM_Y = -90.0

# Slice (f2) kicker mount foot +X face (most-toward-axis face on -X
# side). Bracket foot outer edge must clear this with ≥ WALL_MIN gap.
KICKER_FOOT_X_INNER = -80.0


# --- Flag disc geometry ------------------------------------------------

FLAG_OD = 32.0                                             # disc diameter
FLAG_T = 3.0                                               # disc thickness
FLAG_BORE_D = TURRET_SHAFT_D + 0.4                         # = 8.4

# Detect feature: Ø4 hole at radius 12 from axis at heading 180°.
FLAG_DETECT_HOLE_D = 4.0
FLAG_DETECT_HOLE_R = 12.0

# Y-center: mid-airspace between housing top (-110) and pulley bottom
# (-90). 8.5 mm clear of housing top, 8.5 mm clear of pulley bottom.
FLAG_Y_CENTER = -100.0
_FLAG_TOP_Y = FLAG_Y_CENTER + FLAG_T / 2                   # = -98.5
_FLAG_BOTTOM_Y = FLAG_Y_CENTER - FLAG_T / 2                # = -101.5


# --- Sensor body (stub for bought slotted opto-interrupter) ------------

# Main block: solid box behind the slot. Prongs cantilever forward
# (toward axis) from the main block, with a slot Y-band between them.
# Slot Y matches disc plane Y range exactly (tangent contact at top +
# bottom, modeled as zero-overlap which assert_no_interference accepts).
SENSOR_MAIN_X = 12.0                                       # body X length
SENSOR_BODY_Y = 8.0                                        # body Y total
SENSOR_BODY_Z = 14.0                                       # body Z width
SENSOR_PRONG_X = 18.0                                      # prong X reach

# Sensor outer +X face mounts to bracket post's +X face (on -X side of
# axis, "+X face" = face toward axis = least-negative X).
SENSOR_OUTER_X = -40.0
_SENSOR_MAIN_X_MAX = SENSOR_OUTER_X + SENSOR_MAIN_X        # = -28
_SENSOR_PRONG_TIP_X = _SENSOR_MAIN_X_MAX + SENSOR_PRONG_X  # = -10
_SENSOR_MAIN_CENTER_X = (SENSOR_OUTER_X + _SENSOR_MAIN_X_MAX) / 2
_SENSOR_PRONG_CENTER_X = (_SENSOR_MAIN_X_MAX + _SENSOR_PRONG_TIP_X) / 2

# Slot Y = disc plane Y. Top prong above slot, bottom prong below.
_SLOT_Y_TOP = _FLAG_TOP_Y                                  # = -98.5
_SLOT_Y_BOT = _FLAG_BOTTOM_Y                               # = -101.5
_SENSOR_TOP_Y = _SLOT_Y_TOP + (SENSOR_BODY_Y - FLAG_T) / 2  # = -96
_SENSOR_BOT_Y = _SLOT_Y_BOT - (SENSOR_BODY_Y - FLAG_T) / 2  # = -104
_SENSOR_BODY_CENTER_Y = (_SENSOR_TOP_Y + _SENSOR_BOT_Y) / 2  # = -100
_TOP_PRONG_CENTER_Y = (_SENSOR_TOP_Y + _SLOT_Y_TOP) / 2    # = -97.25
_BOT_PRONG_CENTER_Y = (_SENSOR_BOT_Y + _SLOT_Y_BOT) / 2    # = -102.75
_PRONG_T = (SENSOR_BODY_Y - FLAG_T) / 2                    # = 2.5


# --- Bracket geometry --------------------------------------------------

# Foot: 2 M5 bolts dropping into cross-rail's top T-slot at z = 0.
# Centered at -X side, between the hub housing (x_max = +30) and the
# kicker mount foot (x_min = -80). With a 31 mm wide foot at center
# x = -47, the foot's inner edge sits 1.5 mm clear of the housing
# (-X face -30) and the outer edge sits ≫ WALL_MIN clear of the kicker.
BRACKET_FOOT_X = 31.0
BRACKET_FOOT_Y = 8.0
BRACKET_FOOT_Z = 14.0
_BRACKET_FOOT_CENTER_X = -47.0
_BRACKET_FOOT_X_MIN = _BRACKET_FOOT_CENTER_X - BRACKET_FOOT_X / 2  # = -62.5
_BRACKET_FOOT_X_MAX = _BRACKET_FOOT_CENTER_X + BRACKET_FOOT_X / 2  # = -31.5
_BRACKET_FOOT_BOTTOM_Y = RAIL_TOP_Y                        # = -210
_BRACKET_FOOT_TOP_Y = _BRACKET_FOOT_BOTTOM_Y + BRACKET_FOOT_Y  # = -202
_BRACKET_FOOT_CENTER_Y = (_BRACKET_FOOT_BOTTOM_Y + _BRACKET_FOOT_TOP_Y) / 2

BRACKET_BOLT_HOLE_D = 5.5                                  # M5 clearance
BRACKET_BOLT_SPACING_X = 22.5
_BRACKET_BOLT_HALF_X = BRACKET_BOLT_SPACING_X / 2

# Post: vertical wall hosting the sensor on its +X face (axis-side
# face). Centered on the foot center X; 14 mm wide so it threads
# between the two M5 bolts with WALL_MIN to each. Rises from foot top
# to sensor body top.
BRACKET_POST_X = 14.0
BRACKET_POST_Z = BRACKET_FOOT_Z                            # = 14
_BRACKET_POST_CENTER_X = _BRACKET_FOOT_CENTER_X            # = -47
_BRACKET_POST_X_MIN = _BRACKET_POST_CENTER_X - BRACKET_POST_X / 2  # = -54
_BRACKET_POST_X_MAX = _BRACKET_POST_CENTER_X + BRACKET_POST_X / 2  # = -40
_BRACKET_POST_BOTTOM_Y = _BRACKET_FOOT_TOP_Y               # = -202
_BRACKET_POST_TOP_Y = _SENSOR_TOP_Y                        # = -96
BRACKET_POST_Y = _BRACKET_POST_TOP_Y - _BRACKET_POST_BOTTOM_Y  # = 106
_BRACKET_POST_CENTER_Y = (_BRACKET_POST_BOTTOM_Y + _BRACKET_POST_TOP_Y) / 2


# --- Sanity asserts ----------------------------------------------------

# Flag clears housing top + pulley bottom by ≥ 1 mm.
_flag_to_housing = _FLAG_BOTTOM_Y - HOUSING_TOP_Y
assert _flag_to_housing >= 1.0, (
    f"flag bottom ({_FLAG_BOTTOM_Y}) too close to housing top "
    f"({HOUSING_TOP_Y}); gap {_flag_to_housing:.2f} mm < 1 mm"
)
_flag_to_pulley = PULLEY_BOTTOM_Y - _FLAG_TOP_Y
assert _flag_to_pulley >= 1.0, (
    f"flag top ({_FLAG_TOP_Y}) too close to pulley bottom "
    f"({PULLEY_BOTTOM_Y}); gap {_flag_to_pulley:.2f} mm < 1 mm"
)

# Sensor clears housing top + pulley bottom by ≥ 1 mm.
_sensor_to_housing = _SENSOR_BOT_Y - HOUSING_TOP_Y
assert _sensor_to_housing >= 1.0, (
    f"sensor bottom ({_SENSOR_BOT_Y}) too close to housing top "
    f"({HOUSING_TOP_Y}); gap {_sensor_to_housing:.2f} mm < 1 mm"
)
_sensor_to_pulley = PULLEY_BOTTOM_Y - _SENSOR_TOP_Y
assert _sensor_to_pulley >= 1.0, (
    f"sensor top ({_SENSOR_TOP_Y}) too close to pulley bottom "
    f"({PULLEY_BOTTOM_Y}); gap {_sensor_to_pulley:.2f} mm < 1 mm"
)

# Detect hole sits inside the slot's X range (so the beam aligns with
# the hole at home).
assert SENSOR_OUTER_X + SENSOR_MAIN_X <= -FLAG_DETECT_HOLE_R <= _SENSOR_PRONG_TIP_X, (
    f"detect hole at x = -{FLAG_DETECT_HOLE_R} (azimuth 180°) outside "
    f"slot X range [{_SENSOR_MAIN_X_MAX}, {_SENSOR_PRONG_TIP_X}]; "
    f"adjust FLAG_DETECT_HOLE_R or SENSOR_PRONG_X"
)

# Detect hole edge clears the bore (so the hole doesn't break the
# bore-side wall).
_hole_to_bore = (
    FLAG_DETECT_HOLE_R - FLAG_DETECT_HOLE_D / 2 - FLAG_BORE_D / 2
)
assert _hole_to_bore >= WALL_MIN, (
    f"detect hole edge to bore edge wall {_hole_to_bore:.2f} mm < "
    f"WALL_MIN ({WALL_MIN}); raise FLAG_DETECT_HOLE_R or shrink hole"
)

# Detect hole edge clears the disc OD (so the hole doesn't break the
# rim).
_hole_to_od = (
    FLAG_OD / 2 - FLAG_DETECT_HOLE_R - FLAG_DETECT_HOLE_D / 2
)
assert _hole_to_od >= WALL_MIN, (
    f"detect hole edge to disc OD wall {_hole_to_od:.2f} mm < "
    f"WALL_MIN ({WALL_MIN}); raise FLAG_OD or lower FLAG_DETECT_HOLE_R"
)

# Sensor prong tip clears the shaft.
_prong_tip_to_shaft = abs(_SENSOR_PRONG_TIP_X) - TURRET_SHAFT_D / 2
assert _prong_tip_to_shaft >= WALL_MIN, (
    f"sensor prong tip ({_SENSOR_PRONG_TIP_X}) too close to shaft "
    f"OD (Ø{TURRET_SHAFT_D}); gap {_prong_tip_to_shaft:.2f} mm < "
    f"WALL_MIN ({WALL_MIN})"
)

# Bracket foot inner edge clears housing X-half.
_foot_to_housing = abs(_BRACKET_FOOT_X_MAX) - HOUSING_HALF_X
assert _foot_to_housing >= WALL_MIN, (
    f"bracket foot inner edge ({_BRACKET_FOOT_X_MAX}) too close to "
    f"housing -X face (-{HOUSING_HALF_X}); gap {_foot_to_housing:.2f} "
    f"mm < WALL_MIN ({WALL_MIN}); shift bracket further -X or shrink "
    f"BRACKET_FOOT_X"
)

# Bracket foot outer edge clears kicker mount foot inner edge. Kicker
# foot inner edge sits further -X (more negative) than bracket foot
# outer edge, so the gap is (bracket outer) - (kicker inner).
_foot_to_kicker = _BRACKET_FOOT_X_MIN - KICKER_FOOT_X_INNER
assert _foot_to_kicker >= WALL_MIN, (
    f"bracket foot outer edge ({_BRACKET_FOOT_X_MIN}) too close to "
    f"kicker foot inner edge ({KICKER_FOOT_X_INNER}); gap "
    f"{_foot_to_kicker:.2f} mm < WALL_MIN ({WALL_MIN})"
)

# Bolt holes fit inside the foot with WALL_MIN walls on each side, and
# don't pass through the post (so the post sits on solid foot material
# between the two bolts).
_inner_bolt_x = _BRACKET_FOOT_CENTER_X + _BRACKET_BOLT_HALF_X
_outer_bolt_x = _BRACKET_FOOT_CENTER_X - _BRACKET_BOLT_HALF_X
_bolt_to_foot_inner = _BRACKET_FOOT_X_MAX - (_inner_bolt_x + BRACKET_BOLT_HOLE_D / 2)
assert _bolt_to_foot_inner >= WALL_MIN, (
    f"inner bolt ({_inner_bolt_x}) edge too close to foot inner edge "
    f"({_BRACKET_FOOT_X_MAX}); gap {_bolt_to_foot_inner:.2f} mm < "
    f"WALL_MIN ({WALL_MIN})"
)
_bolt_to_foot_outer = (_outer_bolt_x - BRACKET_BOLT_HOLE_D / 2) - _BRACKET_FOOT_X_MIN
assert _bolt_to_foot_outer >= WALL_MIN, (
    f"outer bolt ({_outer_bolt_x}) edge too close to foot outer edge "
    f"({_BRACKET_FOOT_X_MIN}); gap {_bolt_to_foot_outer:.2f} mm < "
    f"WALL_MIN ({WALL_MIN})"
)
# Inner bolt sits +X of post (toward axis). Bolt's most-outboard edge
# (least +X face of the hole) must clear post's +X face.
_inner_bolt_to_post = (_inner_bolt_x - BRACKET_BOLT_HOLE_D / 2) - _BRACKET_POST_X_MAX
assert _inner_bolt_to_post >= WALL_MIN, (
    f"inner bolt ({_inner_bolt_x}) outboard edge too close to post "
    f"+X face ({_BRACKET_POST_X_MAX}); gap {_inner_bolt_to_post:.2f} "
    f"mm < WALL_MIN ({WALL_MIN}); widen BRACKET_BOLT_SPACING_X or "
    f"shrink BRACKET_POST_X"
)
# Outer bolt sits -X of post (away from axis). Bolt's most-inboard
# edge (most-+X face of the hole) must clear post's -X face.
_outer_bolt_to_post = _BRACKET_POST_X_MIN - (_outer_bolt_x + BRACKET_BOLT_HOLE_D / 2)
assert _outer_bolt_to_post >= WALL_MIN, (
    f"outer bolt ({_outer_bolt_x}) inboard edge too close to post "
    f"-X face ({_BRACKET_POST_X_MIN}); gap {_outer_bolt_to_post:.2f} "
    f"mm < WALL_MIN ({WALL_MIN})"
)


# --- Part functions ----------------------------------------------------

def flag(
    od: float = FLAG_OD,
    thickness: float = FLAG_T,
    bore_d: float = FLAG_BORE_D,
    detect_hole_d: float = FLAG_DETECT_HOLE_D,
    detect_hole_r: float = FLAG_DETECT_HOLE_R,
):
    """Printed disc keyed to the turret shaft below the driven pulley.
    Centered on placement origin (axis along ±Y); bore is a Ø8.4
    clearance hole (real part has an M3 set screw on a flat). One
    Ø4 detect hole at radius `detect_hole_r` from axis at heading
    180° (the -X side, where the sensor sits)."""
    body = Rot(90, 0, 0) * Cylinder(od / 2, thickness)
    body -= Rot(90, 0, 0) * Cylinder(bore_d / 2, thickness * 3)
    body -= (
        Pos(-detect_hole_r, 0, 0)
        * Rot(90, 0, 0)
        * Cylinder(detect_hole_d / 2, thickness * 3)
    )
    return body


def sensor():
    """Slotted opto-interrupter stub. Bought part — no inspect call.
    Modeled as a body with a 3-mm-tall slot in Y between two prongs.
    Returned in turret-local coordinates (no `base` shift); caller
    multiplies the placement."""
    main_block = (
        Pos(_SENSOR_MAIN_CENTER_X, _SENSOR_BODY_CENTER_Y, 0)
        * Box(SENSOR_MAIN_X, SENSOR_BODY_Y, SENSOR_BODY_Z)
    )
    top_prong = (
        Pos(_SENSOR_PRONG_CENTER_X, _TOP_PRONG_CENTER_Y, 0)
        * Box(SENSOR_PRONG_X, _PRONG_T, SENSOR_BODY_Z)
    )
    bot_prong = (
        Pos(_SENSOR_PRONG_CENTER_X, _BOT_PRONG_CENTER_Y, 0)
        * Box(SENSOR_PRONG_X, _PRONG_T, SENSOR_BODY_Z)
    )
    return main_block + top_prong + bot_prong


def bracket():
    """Printed L-bracket: foot bolts to cross-rail's top T-slot;
    vertical post rises to sensor body top. Sensor mounts to post's
    +X face (toward axis). Returned in turret-local coords."""
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

    post = (
        Pos(_BRACKET_POST_CENTER_X, _BRACKET_POST_CENTER_Y, 0)
        * Box(BRACKET_POST_X, BRACKET_POST_Y, BRACKET_POST_Z)
    )

    return foot + post


# --- Stubs for the standalone (duplicated, not imported) ---------------

def _shaft_stub(length: float):
    """Slice (b) Ø8 turret shaft mirror. Caller positions the base at
    the rail top; this function returns a cylinder centered on its own
    origin in X / Z, spanning Y ∈ [0, length]."""
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

HOMING_PART_NAMES = (
    "homing_flag",
    "homing_sensor",
    "homing_bracket",
)


def add_turret_homing(a: Assembly, base=Pos(0, 0, 0)) -> Assembly:
    """Add the three homing parts. Parts are positioned in turret-local
    coordinates; `base` (default: identity) is pre-multiplied to lift
    them into a parent frame (e.g. world)."""
    a = a.add(
        "homing_flag",
        flag(),
        location=base * Pos(TURRET_HUB_X, FLAG_Y_CENTER, TURRET_HUB_Z),
    )
    a = a.add(
        "homing_sensor",
        sensor(),
        location=base,
    )
    a = a.add(
        "homing_bracket",
        bracket(),
        location=base,
    )
    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_turret_homing(Assembly())

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

# Pairwise no-interference. Flag-onto-shaft is tangent contact (zero
# overlap; passes assert_no_interference). Sensor prongs vs flag: prong
# Y faces are tangent to disc Y faces (zero overlap). Bracket foot
# tangent on cross-rail top, sensor outer face tangent on bracket post
# +X face — also zero-overlap.
_ALL = list(HOMING_PART_NAMES) + ["hub_shaft_stub", "cross_rail_stub"]
for _i, _a in enumerate(_ALL):
    for _b in _ALL[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: bottom face on the build plate (+Y up). Flag
    # is a thin disc with a centered bore + offset detect hole — all
    # vertical bores, clean print at default 45° overhang threshold.
    inspect(
        flag(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=45.0
        ),
        out="outputs",
        name="homing_flag",
    )
    # Bracket: foot + vertical post, no horizontal cantilever. Default
    # 45° threshold suffices.
    inspect(
        bracket(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=45.0
        ),
        out="outputs",
        name="homing_bracket",
    )
