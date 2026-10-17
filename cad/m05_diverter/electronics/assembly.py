"""Electronics placement — board and PSU envelopes on plates hung under
the base floor (owner, 2026-09-27: placement only; the chip, firmware
and protocol are not designed).

Two plates, each in one sector of the base floor's hex, hanging from
the inner and outer horizontal rails of that sector on 2020 drops:

    logic  — the face at azimuth 180 (−X): MCU + stepper-driver stack,
             the ESP32 bridge, the servo buck — beside the trap servos'
             cable drop (r 112, azimuth 180), which lands their six
             wires next to the MCU and the buck.
    power  — the face at azimuth 0 (+X): the mains PSU, on its own plate
             on the opposite side, away from the logic boards.

Why under the base floor: below the column feet (z −36) nothing
exists outside the bottom catch (r ≤ 131), so a sector is free from the
rails down to the catch's own depth (the cradle's bottom, z −113), which
is the clearance a future stand must already leave — the plates go no
lower. Sectors 0 and 180 keep off the catch's draw-out (−Y) and the
stack motor (azimuth 120). Adjacent sectors cannot both take a 240-wide
plate (a sector is ±75 wide at the inner rail), hence opposite ones.
Every part here is static; nothing is near a moving part's sweep.

Every bought dimension below is a parameter with what derives from it,
so a substitute is a parameter change. The envelopes are generous
boxes, not models: the drivers' heatsinks, a board's standoffs and its
connectors are inside them.

Coordinate frame: the base floor's (= m05-local at the base floor):
    origin = column axis ∩ rail-top datum (z = 0)
    +Z     = up; the floor's rails occupy z −RAIL_SIZE..0
Each plate is built in a sector-local frame — +X radial outward
through its hex face's midpoint — and turned to its face azimuth.
"""

from __future__ import annotations

from typing import Callable

from build123d import Box, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from ..params import R_HEX_INNER, R_HEX_OUTER, RAIL_SIZE


# --- Parameters --------------------------------------------------------

# Face azimuth of each plate's sector. The horizontal rail on the face at
# azimuth 60·(k+1) is `inner_horiz_k` / `outer_horiz_k` (frame naming).
LOGIC_FACE_DEG = 180.0
POWER_FACE_DEG = 0.0


def face_rail_index(face_deg: float) -> int:
    """`k` of the inner/outer horizontal rails on the face at `face_deg`."""
    return round(face_deg / 60.0 - 1) % 6


RAIL_BOTTOM_Z = -RAIL_SIZE                       # −20 — the hangers bolt up to this face

# The plate: printed PLA, 5 mm. Radially from the inner rail's inner
# face to the outer rail's outer face, so each end lies under its rail;
# tangentially 240, which leaves 16 on the X1C's 256 bed (bed_fit).
PLATE_T = 5.0
PLATE_X0 = R_HEX_INNER - RAIL_SIZE / 2           # 130
PLATE_X1 = R_HEX_OUTER + RAIL_SIZE / 2           # 280
PLATE_W = 240.0
# The plate's underside sits at the bottom catch's depth (`base_floor`
# CATCH_*: the cradle reaches z −113), so hanging electronics do not
# deepen the clearance a stand must leave. Mirrored here (no sibling
# imports); the tower root asserts it against the live cradle.
PLATE_Z_BOT = -113.0
PLATE_Z_TOP = PLATE_Z_BOT + PLATE_T              # −108

# Hangers: 2020 extrusion drops (bought, aluminium) from a rail's bottom
# face to the plate's top face. One under the inner rail on the sector
# centreline — the inner vertex brackets hang to z −26 at |y| ≥ 19, so
# that is the only room under the inner rail — and two under the outer
# rail at ±HANGER_OUTER_Y, inside the outer vertex brackets (|y| ≥ 94)
# and wide enough apart for the reach slot to pass between them.
HANGER_SIZE = 20.0
HANGER_OUTER_Y = 70.0
HANGER_L = RAIL_BOTTOM_Z - PLATE_Z_TOP           # 88

# Bought envelopes: (radial X, tangential Y, height Z), placed on the
# plate's top face. Each is a box the real part, its standoffs and its
# plugged connectors fit inside.
#
# MCU + stepper-driver stack: one MCU with a 5-axis driver shield on it.
# Five stepper channels = m02's three rim drives, m03's turret drive and
# m05's stack drive (the modelled motors); the five servos (four trap
# servos, m03's dump servo) take PWM pins straight off the MCU. A Mega
# 2560 is 101.6 × 53.3; the shield, heatsinks and headers set 45 tall.
MCU_STACK = (60.0, 102.0, 45.0)
MCU_STACK_AT = (160.0, -110.0)                   # its −X/−Y corner on the plate
# The ESP32 bridge on standoffs (a dev board is ~55 × 28).
BRIDGE = (28.0, 55.0, 20.0)
BRIDGE_AT = (160.0, 10.0)
# The servo buck: 5–6 V for the five servos (MG90S stall ≈ 0.7 A each,
# so a ≥ 5 A module); ~60 × 40.
SERVO_BUCK = (40.0, 60.0, 22.0)
SERVO_BUCK_AT = (200.0, 10.0)
# The PSU: mains → 24 V for the stepper drivers. A 150 W enclosed
# supply is ~159 × 97 × 30; this is generous for a larger one.
PSU = (100.0, 200.0, 40.0)
PSU_AT = (155.0, -100.0)

# Reach for wiring and service: from outside the machine, over the
# plate's components, under the vertex brackets (z −26), between the
# outer hangers — a slot SERVICE_REACH_Y either side of the sector centreline, from
# SERVICE_REACH_X0 inboard out to SERVICE_REACH_X1. Starts outboard of
# the inner columns (x ≤ 154), whose feet stop at z −36.
SERVICE_REACH_X0 = 160.0
SERVICE_REACH_X1 = 400.0
SERVICE_REACH_Y = 58.0                           # 116 wide, between the outer hangers (|y| ≥ 60)
SERVICE_REACH_Z_TOP = -27.0


# --- Part functions (sector-local) -------------------------------------

def board_plate() -> Part:
    """The printed plate, sector-local."""
    x0, x1 = PLATE_X0, PLATE_X1
    return Pos((x0 + x1) / 2, 0, (PLATE_Z_BOT + PLATE_Z_TOP) / 2) * Box(
        x1 - x0, PLATE_W, PLATE_T,
    )


def hanger(x: float, y: float) -> Part:
    """A 2020 drop at (x, y), from the rail's bottom face to the plate's top."""
    return Pos(x, y, (RAIL_BOTTOM_Z + PLATE_Z_TOP) / 2) * Box(HANGER_SIZE, HANGER_SIZE, HANGER_L)


def envelope(size: tuple[float, float, float], at: tuple[float, float]) -> Part:
    """A bought part's envelope standing on the plate, its −X/−Y corner at `at`."""
    sx, sy, sz = size
    return Pos(at[0] + sx / 2, at[1] + sy / 2, PLATE_Z_TOP + sz / 2) * Box(sx, sy, sz)


def service_reach(top_of_components: float) -> Part:
    """The reach slot above a plate's components, sector-local."""
    z0 = top_of_components
    return Pos(
        (SERVICE_REACH_X0 + SERVICE_REACH_X1) / 2, 0, (z0 + SERVICE_REACH_Z_TOP) / 2,
    ) * Box(SERVICE_REACH_X1 - SERVICE_REACH_X0, 2 * SERVICE_REACH_Y, SERVICE_REACH_Z_TOP - z0)


_HANGERS = {
    "hanger_in": (R_HEX_INNER, 0.0),
    "hanger_out_pos": (R_HEX_OUTER, HANGER_OUTER_Y),
    "hanger_out_neg": (R_HEX_OUTER, -HANGER_OUTER_Y),
}


# --- Builders ----------------------------------------------------------

def _build_plate(
    face_deg: float,
    boards: dict[str, tuple[tuple[float, float, float], tuple[float, float]]],
    material_fn: Callable[[str], str | None] | None,
) -> Assembly:
    """One plate with its hangers and boards, turned to `face_deg`."""
    def _mt(name: str) -> str | None:
        return material_fn(name) if material_fn is not None else None

    turn = Rot(0, 0, face_deg)
    a = Assembly().with_part("board_plate", turn * board_plate(), material=_mt("board_plate"))
    for name, (x, y) in _HANGERS.items():
        a = a.with_part(name, turn * hanger(x, y), material=_mt(name))
    for name, (size, at) in boards.items():
        a = a.with_part(name, turn * envelope(size, at), material=_mt(name))

    tallest = PLATE_Z_TOP + max(size[2] for size, _ in boards.values())
    # Starts at the tallest board's top, so that board touches it (allowed:
    # min 0) and every other part stands clear.
    a = a.with_keepout("service_reach", turn * service_reach(tallest))
    # Each hanger stands on the plate; each board sits on it; nothing
    # on the plate touches anything else on it.
    for name in _HANGERS:
        a = a.assert_tangent_contact(name, "board_plate")
    for name in boards:
        a = a.assert_tangent_contact(name, "board_plate")
    a = a.assert_no_interference_within([*_HANGERS, *boards])
    a = a.assert_clear_of([*_HANGERS, *boards], "service_reach", name="service_reach_clear")
    return a.assert_solid_count("board_plate", detail="the plate is one printed piece")


def build_electronics(
    material_fn: Callable[[str], str | None] | None = None,
) -> Assembly:
    """Both plates, base-floor-local. Static. Their rests on the rails,
    and clearances to the frame, the drive train and the catch, are the
    tower root's (`m05_diverter/assembly.py` `_add_electronics_checks`)."""
    logic = _build_plate(LOGIC_FACE_DEG, {
        "mcu_stack": (MCU_STACK, MCU_STACK_AT),
        "bridge": (BRIDGE, BRIDGE_AT),
        "servo_buck": (SERVO_BUCK, SERVO_BUCK_AT),
    }, material_fn)
    power = _build_plate(POWER_FACE_DEG, {"psu": (PSU, PSU_AT)}, material_fn)
    return (
        Assembly()
        .with_subassembly("logic", logic)
        .with_subassembly("power", power)
        .assert_no_interference_between("logic", "power")
    )


assembly = build_electronics()
