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

Layout: flag disc + sensor live in the 8 mm of clear airspace between
the slice (g) driven pulley top (z = -51) and the slice (d) platform
extension drop-block bottom (z = -43, dynamic — the IDLE-pose
platform's drop block sweeps through this azimuth during rotor
transit). This slot opened up when the 2026-05-23 dump_servo retrofit
retired the shaft-mounted dump cam (which used to sit at z ∈ [-50,
-38] right between the pulley and the spider bottom at z = -35).
The original homing slice (h, 2026) placed the parts one slot down in
the housing-pulley airspace; floor-rise slices c5+c7 closed that
20-mm slot to 1 mm and orphaned them. The upper slot is tighter than
the original (8 mm vs 20 mm) because the platform drop-block sweep
caps it from above, so SENSOR_BODY_Z shrinks 8 → 6 mm to fit (slim
slot-type opto-interrupters like the Sharp GP1A57 / Omron EE-SX1041
are 5-7 mm in the slot dimension, well within reality). The bracket
bolts to the slice (a) cross-rail's top T-slot at -X side, in the
gap between the hub housing (x_max = +30) and the dump_servo bracket
footprint (well outboard at X ∈ [-92, -68] on the cross-rail top,
2026-05-23 onward). Sensor azimuth at heading 180° (-X direction);
detect feature on the disc at the same azimuth at home.

Members (3 functional + 2 stubs):
  * homing_flag       — printed disc (Ø32 × 3 mm thick, Ø8.4 shaft
                        clearance bore, single Ø4 detect hole at radius
                        12 from axis at heading 180°). Keyed to the
                        shaft via M3 set screw on a flat (real part);
                        modeled as clearance bore so
                        assert_no_interference reads tangent contact.
                        Z-center -47 (mid-airspace; 2.5 mm clear of
                        pulley top, 2.5 mm clear of platform drop-block
                        bottom).
  * homing_sensor     — slotted opto-interrupter stub (bought part).
                        Modeled as a body with two prongs above + below
                        the disc plane and a 3-mm-tall slot between
                        them; main block sits outboard at x ∈ [-40,
                        -28], prongs reach in to x = -10 (6 mm past
                        disc OD at radius 16). Beam axis (not modeled)
                        crosses the slot at (-12, 0, *) — aligned with
                        the detect hole at home.
  * homing_bracket    — printed L-bracket: foot bolts to cross-rail's
                        top T-slot at y = 0 with two M5 bolts spaced
                        22.5 mm along the rail; vertical post rises to
                        sensor body Z. Sensor mounts to post's +X face
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
    origin = hub axis at the cross-rail's top face Z plane
    +X     = operator's right            (parallel to world +X)
    +Y     = away from operator          (parallel to world +Y)
    +Z     = up                          (parallel to world +Z)

Detect feature design: Ø4 hole at radius 12 from axis at heading 180°
(the home azimuth). The beam (LED in top prong → photodiode in bottom
prong, beam axis vertical at (-12, 0, *)) is blocked by disc material
at all azimuths except home, where the hole aligns with the beam and
photodiode sees light. The hole rotates with the disc; firmware homes
once at startup, then step-counts. The disc is set-screwed to a flat
on the shaft, so the rotational offset between disc-home and arm-
indexed-position can be set during physical assembly; this slice ships
the disc at its home rotation (hole at heading 180°).

Print orientation (flag): bottom face on build plate, +Z up (build123d
default). All features (OD, bore, detect hole) are vertical bores —
clean print. Print orientation (bracket): bottom face on build plate,
+Z up. Foot prints flat, post is a vertical extrusion. No horizontal
overhangs; default 45° threshold suffices (no need to lift to 90° as
drive_bracket did, since this bracket has no cantilever plate).
"""


from build123d import Box, Cylinder, Pos

from cad_khana.mechanism.assembly import Assembly


from ..params import (
    TURRET_FRAME_CROSS_Y,
    TURRET_FRAME_FLOOR_Z,
    TURRET_HUB_X,
    TURRET_HUB_Y,
    TURRET_SHAFT_D,
    TURRET_SHAFT_TOP_Z,
    WALL_MIN,
)
from ..design_systems import color_of
from ..materials import material_of


# --- Cross-rail interface (private mirror of slice a) ------------------

MEMBER = 20.0                                              # 2020 extrusion
RAIL_TOP_Z = TURRET_FRAME_FLOOR_Z + MEMBER / 2             # = -120 after c7


# --- Adjacent-slice mirrors (private; bound assertions reference these) -

# Slice (b) housing X half-extent. Bracket foot inner edge must clear
# this with ≥ WALL_MIN gap (housing Z [-120, -60] still abuts the
# foot's Z band [-120, -112] in X; the post rises clear of housing
# top in Z but the foot does not).
HOUSING_HALF_X = 30.0

# Slice (g) driven pulley top face Z. Flag + sensor must clear this
# above with ≥ 1 mm gap. (Pulley body at z ∈ [-59, -51]; the bracket
# post passes outboard of the pulley in X — see post X-range below.)
PULLEY_TOP_Z = -51.0

# Slice (d) platform extension drop-block bottom Z. The IDLE-pose
# platform sweeps through azimuth 180° (the homing sensor's azimuth)
# during rotor transit; its drop-block bottom is the lower-Z surface
# crossing this zone (the stem above is at z ∈ [-15, -4], well above
# the homing parts). Flag + sensor must clear this below with ≥ 1 mm
# gap. Mirror of `turret_platform/assembly.py`'s EXTENSION_BOTTOM_Z;
# itself derived from `TURRET_DUMP_SERVO_LIFTER_TOP_Z + 2` in
# `params.py`. Update if the platform drop-block geometry changes.
DROP_BLOCK_BOTTOM_Z = -43.0

# --- Flag disc geometry ------------------------------------------------

FLAG_OD = 32.0                                             # disc diameter
FLAG_T = 3.0                                               # disc thickness
FLAG_BORE_D = TURRET_SHAFT_D + 0.4                         # = 8.4

# Detect feature: Ø4 hole at radius 12 from axis at heading 180°.
FLAG_DETECT_HOLE_D = 4.0
FLAG_DETECT_HOLE_R = 12.0

# Z-center: mid-airspace between pulley top (-51) and drop-block
# bottom (-43, dynamic sweep). 2.5 mm clear of pulley top, 2.5 mm
# clear of drop-block bottom.
FLAG_Z_CENTER = -47.0
_FLAG_TOP_Z = FLAG_Z_CENTER + FLAG_T / 2                   # = -45.5
_FLAG_BOTTOM_Z = FLAG_Z_CENTER - FLAG_T / 2                # = -48.5


# --- Sensor body (stub for bought slotted opto-interrupter) ------------

# Main block: solid box behind the slot. Prongs cantilever forward
# (toward axis) from the main block, with a slot Z-band between them.
# Slot Z matches disc plane Z range exactly (tangent contact at top +
# bottom, modeled as zero-overlap which assert_no_interference accepts).
SENSOR_MAIN_X = 12.0                                       # body X length
SENSOR_BODY_Y = 14.0                                       # body Y width (depth)
SENSOR_BODY_Z = 6.0                                        # body Z total (vertical)
SENSOR_PRONG_X = 18.0                                      # prong X reach

# Sensor outer +X face mounts to bracket post's +X face (on -X side of
# axis, "+X face" = face toward axis = least-negative X).
SENSOR_OUTER_X = -40.0
_SENSOR_MAIN_X_MAX = SENSOR_OUTER_X + SENSOR_MAIN_X        # = -28
_SENSOR_PRONG_TIP_X = _SENSOR_MAIN_X_MAX + SENSOR_PRONG_X  # = -10
_SENSOR_MAIN_CENTER_X = (SENSOR_OUTER_X + _SENSOR_MAIN_X_MAX) / 2
_SENSOR_PRONG_CENTER_X = (_SENSOR_MAIN_X_MAX + _SENSOR_PRONG_TIP_X) / 2

# Slot Z = disc plane Z. Top prong above slot, bottom prong below.
_SLOT_Z_TOP = _FLAG_TOP_Z                                  # = -45.5
_SLOT_Z_BOT = _FLAG_BOTTOM_Z                               # = -48.5
_SENSOR_TOP_Z = _SLOT_Z_TOP + (SENSOR_BODY_Z - FLAG_T) / 2  # = -44
_SENSOR_BOT_Z = _SLOT_Z_BOT - (SENSOR_BODY_Z - FLAG_T) / 2  # = -50
_SENSOR_BODY_CENTER_Z = (_SENSOR_TOP_Z + _SENSOR_BOT_Z) / 2  # = -47
_TOP_PRONG_CENTER_Z = (_SENSOR_TOP_Z + _SLOT_Z_TOP) / 2    # = -44.75
_BOT_PRONG_CENTER_Z = (_SENSOR_BOT_Z + _SLOT_Z_BOT) / 2    # = -49.25
_PRONG_T = (SENSOR_BODY_Z - FLAG_T) / 2                    # = 1.5


# --- Bracket geometry --------------------------------------------------

# Foot: 2 M5 bolts dropping into cross-rail's top T-slot at y = 0.
# Centered at -X side, outboard of the hub housing (x_max = +30) and
# inboard of the dump_servo bracket (x_min ≈ -92 on the cross-rail top,
# 2026-05-23 onward). With a 31 mm wide foot at center x = -47, the
# foot's inner edge sits 1.5 mm clear of the housing (-X face -30); the
# outer edge sits ≫ WALL_MIN clear of the servo bracket.
BRACKET_FOOT_X = 31.0
BRACKET_FOOT_Y = 14.0    # depth
BRACKET_FOOT_Z = 8.0     # vertical
_BRACKET_FOOT_CENTER_X = -47.0
_BRACKET_FOOT_X_MIN = _BRACKET_FOOT_CENTER_X - BRACKET_FOOT_X / 2  # = -62.5
_BRACKET_FOOT_X_MAX = _BRACKET_FOOT_CENTER_X + BRACKET_FOOT_X / 2  # = -31.5
_BRACKET_FOOT_BOTTOM_Z = RAIL_TOP_Z                        # = -120
_BRACKET_FOOT_TOP_Z = _BRACKET_FOOT_BOTTOM_Z + BRACKET_FOOT_Z  # = -112
_BRACKET_FOOT_CENTER_Z = (_BRACKET_FOOT_BOTTOM_Z + _BRACKET_FOOT_TOP_Z) / 2

BRACKET_BOLT_HOLE_D = 5.5                                  # M5 clearance
BRACKET_BOLT_SPACING_X = 22.5
_BRACKET_BOLT_HALF_X = BRACKET_BOLT_SPACING_X / 2

# Post: vertical wall hosting the sensor on its +X face (axis-side
# face). Centered on the foot center X; 14 mm wide so it threads
# between the two M5 bolts with WALL_MIN to each. Rises from foot top
# to sensor body top.
BRACKET_POST_X = 14.0
BRACKET_POST_Y = BRACKET_FOOT_Y                            # = 14
_BRACKET_POST_CENTER_X = _BRACKET_FOOT_CENTER_X            # = -47
_BRACKET_POST_X_MIN = _BRACKET_POST_CENTER_X - BRACKET_POST_X / 2  # = -54
_BRACKET_POST_X_MAX = _BRACKET_POST_CENTER_X + BRACKET_POST_X / 2  # = -40
_BRACKET_POST_BOTTOM_Z = _BRACKET_FOOT_TOP_Z               # = -112
_BRACKET_POST_TOP_Z = _SENSOR_TOP_Z                        # = -44
BRACKET_POST_Z = _BRACKET_POST_TOP_Z - _BRACKET_POST_BOTTOM_Z  # = 68
_BRACKET_POST_CENTER_Z = (_BRACKET_POST_BOTTOM_Z + _BRACKET_POST_TOP_Z) / 2


# --- Sanity asserts ----------------------------------------------------

# Flag clears pulley top (below) + drop-block bottom (above) by ≥ 1 mm.
_flag_to_pulley = _FLAG_BOTTOM_Z - PULLEY_TOP_Z
assert _flag_to_pulley >= 1.0, (
    f"flag bottom ({_FLAG_BOTTOM_Z}) too close to pulley top "
    f"({PULLEY_TOP_Z}); gap {_flag_to_pulley:.2f} mm < 1 mm"
)
_flag_to_drop = DROP_BLOCK_BOTTOM_Z - _FLAG_TOP_Z
assert _flag_to_drop >= 1.0, (
    f"flag top ({_FLAG_TOP_Z}) too close to drop-block bottom "
    f"({DROP_BLOCK_BOTTOM_Z}); gap {_flag_to_drop:.2f} mm < 1 mm"
)

# Sensor clears pulley top (below) + drop-block bottom (above) by ≥ 1 mm.
_sensor_to_pulley = _SENSOR_BOT_Z - PULLEY_TOP_Z
assert _sensor_to_pulley >= 1.0, (
    f"sensor bottom ({_SENSOR_BOT_Z}) too close to pulley top "
    f"({PULLEY_TOP_Z}); gap {_sensor_to_pulley:.2f} mm < 1 mm"
)
_sensor_to_drop = DROP_BLOCK_BOTTOM_Z - _SENSOR_TOP_Z
assert _sensor_to_drop >= 1.0, (
    f"sensor top ({_SENSOR_TOP_Z}) too close to drop-block bottom "
    f"({DROP_BLOCK_BOTTOM_Z}); gap {_sensor_to_drop:.2f} mm < 1 mm"
)

# Bracket post top stays below drop-block bottom by ≥ 1 mm so the
# transit sweep at azimuth 180° clears the post (post X-range
# [-54, -40] overlaps the drop-block sweep annulus at radius 25-53).
_post_to_drop = DROP_BLOCK_BOTTOM_Z - _BRACKET_POST_TOP_Z
assert _post_to_drop >= 1.0, (
    f"bracket post top ({_BRACKET_POST_TOP_Z}) too close to drop-block "
    f"bottom ({DROP_BLOCK_BOTTOM_Z}); gap {_post_to_drop:.2f} mm < 1 mm"
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

# Bolt holes fit inside the foot with WALL_MIN walls on each side, and
# don't pass through the post.
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
_inner_bolt_to_post = (_inner_bolt_x - BRACKET_BOLT_HOLE_D / 2) - _BRACKET_POST_X_MAX
assert _inner_bolt_to_post >= WALL_MIN, (
    f"inner bolt ({_inner_bolt_x}) outboard edge too close to post "
    f"+X face ({_BRACKET_POST_X_MAX}); gap {_inner_bolt_to_post:.2f} "
    f"mm < WALL_MIN ({WALL_MIN}); widen BRACKET_BOLT_SPACING_X or "
    f"shrink BRACKET_POST_X"
)
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
    Centered on placement origin (axis along ±Z = build123d default);
    bore is a Ø8.4 clearance hole (real part has an M3 set screw on a
    flat). One Ø4 detect hole at radius `detect_hole_r` from axis at
    heading 180° (the -X side, where the sensor sits)."""
    body = Cylinder(od / 2, thickness)
    body -= Cylinder(bore_d / 2, thickness * 3)
    body -= (
        Pos(-detect_hole_r, 0, 0)
        * Cylinder(detect_hole_d / 2, thickness * 3)
    )
    return body


def sensor():
    """Slotted opto-interrupter stub. Bought part — no inspect call.
    Modeled as a body with a 3-mm-tall slot in Z between two prongs.
    Returned in turret-local coordinates (no `base` shift); caller
    multiplies the placement."""
    main_block = (
        Pos(_SENSOR_MAIN_CENTER_X, 0, _SENSOR_BODY_CENTER_Z)
        * Box(SENSOR_MAIN_X, SENSOR_BODY_Y, SENSOR_BODY_Z)
    )
    top_prong = (
        Pos(_SENSOR_PRONG_CENTER_X, 0, _TOP_PRONG_CENTER_Z)
        * Box(SENSOR_PRONG_X, SENSOR_BODY_Y, _PRONG_T)
    )
    bot_prong = (
        Pos(_SENSOR_PRONG_CENTER_X, 0, _BOT_PRONG_CENTER_Z)
        * Box(SENSOR_PRONG_X, SENSOR_BODY_Y, _PRONG_T)
    )
    return main_block + top_prong + bot_prong


def bracket():
    """Printed L-bracket: foot bolts to cross-rail's top T-slot;
    vertical post rises to sensor body top. Sensor mounts to post's
    +X face (toward axis). Returned in turret-local coords."""
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

    post = (
        Pos(_BRACKET_POST_CENTER_X, 0, _BRACKET_POST_CENTER_Z)
        * Box(BRACKET_POST_X, BRACKET_POST_Y, BRACKET_POST_Z)
    )

    return foot + post


# --- Stubs for the standalone (duplicated, not imported) ---------------

def _shaft_stub(length: float):
    """Slice (b) Ø8 turret shaft mirror. Caller positions the base at
    the rail top; this function returns a cylinder centered on its own
    origin in X / Y, spanning Z ∈ [0, length]."""
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

HOMING_STATIC_PART_NAMES = ("homing_sensor", "homing_bracket")
HOMING_ROTOR_PART_NAMES = ("homing_flag",)
HOMING_PART_NAMES = HOMING_ROTOR_PART_NAMES + HOMING_STATIC_PART_NAMES


def build_turret_homing_static() -> Assembly:
    """Static homing sub-assembly: sensor + bracket, in turret-local
    coordinates. Both bolt to the bench frame; only the flag (keyed
    to the hub shaft) revolves."""
    a = Assembly()
    # homing_sensor material left unset — opto-interrupter housing,
    # render.py overrides.
    a = a.with_part(
        "homing_sensor",
        sensor(),
    )
    a = a.with_part(
        "homing_bracket",
        bracket(),
        material=material_of("homing_bracket"),
        color=color_of("homing_bracket"),
    )
    return a


def build_turret_homing_rotor() -> Assembly:
    """Rotor homing sub-assembly: the flag (keyed to the hub shaft),
    in turret-local coordinates."""
    return Assembly().with_part(
        "homing_flag",
        flag(),
        location=Pos(TURRET_HUB_X, TURRET_HUB_Y, FLAG_Z_CENTER),
        material=material_of("homing_flag"),
        color=color_of("homing_flag"),
    )


# --- Standalone assembly -----------------------------------------------

assembly = (
    Assembly()
    .with_subassembly("static", build_turret_homing_static())
    .with_subassembly("rotor", build_turret_homing_rotor())
)

# Stubs.
_SHAFT_LEN = TURRET_SHAFT_TOP_Z - RAIL_TOP_Z               # = 200
assembly = assembly.with_part(
    "hub_shaft_stub",
    _shaft_stub(length=_SHAFT_LEN),
    location=Pos(TURRET_HUB_X, TURRET_HUB_Y, RAIL_TOP_Z),
    material="steel",
)
assembly = assembly.with_part(
    "cross_rail_stub",
    _cross_rail_stub(),
    material="aluminium_anodized",
)

# Pairwise no-interference. Flag-onto-shaft is tangent contact (zero
# overlap; passes assert_no_interference). Sensor prongs vs flag: prong
# Z faces are tangent to disc Z faces (zero overlap). Bracket foot
# tangent on cross-rail top, sensor outer face tangent on bracket post
# +X face — also zero-overlap.
_ALL = (
    [f"rotor.{n}" for n in HOMING_ROTOR_PART_NAMES]
    + [f"static.{n}" for n in HOMING_STATIC_PART_NAMES]
    + ["hub_shaft_stub", "cross_rail_stub"]
)
assembly = assembly.assert_no_interference_within(_ALL)
