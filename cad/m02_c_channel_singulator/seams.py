"""Thirds seams: split a Ø310 ring part into sectors that fit the bed.

The bowl and the stator liner both reach r 155 — past the X1C's 256 mm
bed whole — so each prints as SEAM_COUNT sectors (owner 2026-09-13;
`_notes/state/m02-singulator.md`). A 120° sector of r 155 fits a
195 mm square; with the ears below, ≈ 206.

Joint (decided in the seams slice, 2026-09-17): **bolted radial ears,
outside the ring.** At each seam both sectors carry a flat tab on
their own side of the seam plane, from just inside the wall's outer
face out to r + SEAM_EAR_OUT, so the two tabs meet face-to-face on the
seam plane and a tangential bolt pulls them together. Outside, not
inside: inside the bowl the rotor flange, bearing and pinion fill the
annulus, and inside the liner is the brick surface. Not dovetails: a
dovetail in a 3 mm wall has nothing to bear on. The floor's radial
seam inboard of the skirt is spanned by the turntable bearing's lower
ring, which bolts to the floor on both sides of it (detail layer).

Seam bolts (fastening pass, 2026-11-09): each ear pair takes
``seam_bolt_zs`` M3 socket-head bolts, a washer under the head and
under the nut (bd_warehouse's ISO bodies, fused into one part), through teardrop holes roofed toward the part's
print-up (`teardrop.py`). The head sits on sector k's side of seam k,
the nut on sector k−1's. Hardware and spacing are a session's choice,
not the owner's (owner deferred, 2026-11-09). ``with_seam_bolts``
places them and holds their claims in the builder that owns the ring;
each bolt's tool corridors are held against the rest of the machine
in `cad/assembly.py`.

Sector k spans [phase + k·360/n, phase + (k+1)·360/n] in m02-local
degrees (0 = the exit arc's centre). The phase is a per-instance
choice, like the motor azimuth: the seam lines must miss the exit
arc, the motor, and wherever a gantry bar ends under the floor — so
the chain picks it per stage (`cad/m02_chain/params.py`
§Seams).
"""

from __future__ import annotations

import math
import operator
from collections.abc import Callable, Sequence
from functools import cache, reduce

from bd_warehouse.fastener import HexNut, PlainWasher, SocketHeadCapScrew
from build123d import Box, Cylinder, Location, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from .params import HOLE_ROOF_DEG, HOPPER_WALL_H, SKIRT_TOP_Z
from .teardrop import teardrop_cutter


SEAM_COUNT = 3
# fmt: off
SEAM_EAR_T = 5.0         # tangential thickness of each tab (a pair is 10)
SEAM_EAR_OUT = 12.0      # radial reach past the ring's outer face
SEAM_EAR_OVERLAP = 0.5   # tab starts this far inside the outer face, so it fuses

# Seam bolts. Bought parts, from bd_warehouse's ISO tables — a substitute
# is a change here:
SEAM_BOLT_SIZE = "M3-0.5"      # ISO 4762 socket head cap screw
SEAM_BOLT_D = 3.0              # its thread major Ø → the hole
SEAM_BOLT_L = 16.0             # its length under the head
SEAM_NUT_TYPE = "iso4032"      # hex nut, same size
SEAM_WASHER_SIZE = "M3"        # ISO 7089 plain washer, under the head and under the nut (PLA ears)
# fmt: on
SEAM_WASHER_TYPE = "iso7089"
# Designed:
# fmt: off
SEAM_BOLT_HOLE_CLEAR = 0.4     # diametral clearance through both ears
SEAM_BOLT_R = SEAM_EAR_OUT / 2.0   # axis outside the ring's outer face: mid-tab radially
SEAM_BOLT_EDGE = 6.5           # axis in from the ear's top and bottom edges
SEAM_BOLT_PITCH_MAX = 50.0     # most axis-to-axis spacing up a tall ear
SEAM_DRIVER_D = 8.0            # the hex key's corridor, out along the axis from the head (as ARC_SCREW_ACCESS_D)
SEAM_NUT_TOOL_D = 10.0         # a 5.5 nut driver's corridor, out along the axis from the nut
SEAM_TOOL_L = 60.0             # each corridor's reach (as ARC_SCREW_ACCESS_L)
SEAM_TOOL_MARGIN = 0.5         # least clearance any body keeps from a corridor (as ARC_SCREW_ACCESS_MARGIN)
# fmt: on

SEAM_BOLT_HOLE_D = SEAM_BOLT_D + SEAM_BOLT_HOLE_CLEAR


def seam_azimuths(phase_deg: float, n: int = SEAM_COUNT) -> tuple[float, ...]:
    """The n seam azimuths, degrees in [-180, 180)."""
    return tuple(_wrap(phase_deg + k * 360.0 / n) for k in range(n))


def sector_index(az_deg: float, phase_deg: float, n: int = SEAM_COUNT) -> int:
    """Which sector contains azimuth ``az_deg``."""
    return int(((az_deg - phase_deg) % 360.0) // (360.0 / n))


def angular_gap_deg(a: float, b: float) -> float:
    """Unsigned angle between two azimuths, degrees in [0, 180]."""
    return abs(_wrap(a - b))


def angular_gap_to_span_deg(a: float, lo: float, hi: float) -> float:
    """Unsigned angle from azimuth ``a`` to the arc ``lo`` → ``hi``
    (the short way round, under 180°): 0 inside it, else the gap to
    the nearer end."""
    span = angular_gap_deg(lo, hi)
    to_lo, to_hi = angular_gap_deg(a, lo), angular_gap_deg(a, hi)
    return 0.0 if to_lo + to_hi <= span + 1e-9 else min(to_lo, to_hi)


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def split_ring(
    ring: Part,
    *,
    phase_deg: float,
    r_out: float,
    ear_z: tuple[float, float],
    hole_apex_up: bool,
    n: int = SEAM_COUNT,
    ear_t: float = SEAM_EAR_T,
    ear_out: float = SEAM_EAR_OUT,
) -> list[Part]:
    """Cut ``ring`` (a solid of revolution about +Z, outer radius
    ``r_out``) into ``n`` sectors at ``phase_deg``, each carrying an
    ear at both of its seams over ``ear_z`` = (z0, z1), drilled for
    the seam bolts with teardrops roofed +Z if ``hole_apex_up`` (the
    part's print-up), −Z if not."""
    span = 360.0 / n
    assert 0.0 < ear_t and 0.0 < ear_out and ear_z[0] < ear_z[1]
    # The ear must lie inside its own sector's angle at the ring's
    # outer face, or it would cross into the neighbour.
    assert math.degrees(ear_t / r_out) < span / 2.0
    # Each washer lies on its tab, inside the ring's outer face.
    assert _washer().washer_diameter / 2.0 <= min(SEAM_BOLT_R, ear_out - SEAM_BOLT_R), "seam washer runs off its tab"
    big_r = r_out + ear_out + 50.0
    z_lo, z_hi = -1000.0, 1000.0
    r0, r1 = r_out - SEAM_EAR_OVERLAP, r_out + ear_out
    holes = _seam_holes(r_out, ear_z, hole_apex_up, ear_t)
    sectors = []
    for k in range(n):
        a0 = phase_deg + k * span
        wedge = (
            Rot(0, 0, a0)
            * Pos(0, 0, z_lo)
            * Cylinder(
                big_r,
                z_hi - z_lo,
                arc_size=span,
                align=None,
            )
        )
        body = ring & wedge
        ear = Box(r1 - r0, ear_t, ear_z[1] - ear_z[0], align=None)
        # Start seam: the tab lies on the +tangential side (inside the
        # sector); end seam: on the −tangential side.
        body += Rot(0, 0, a0) * Pos(r0, 0.0, ear_z[0]) * ear
        body += Rot(0, 0, a0 + span) * Pos(r0, -ear_t, ear_z[0]) * ear
        body -= Rot(0, 0, a0) * holes
        body -= Rot(0, 0, a0 + span) * holes
        sectors.append(body)
    return sectors


def seam_bolt_zs(z0: float, z1: float) -> tuple[float, ...]:
    """Bolt heights up an ear spanning z0..z1: SEAM_BOLT_EDGE in from
    each edge, evenly, no more than SEAM_BOLT_PITCH_MAX apart; one at
    mid-height where two washers would not fit."""
    washer_d = _washer().washer_diameter
    assert z1 - z0 >= washer_d, f"a {z1 - z0:.1f} mm ear is too short for a seam bolt's washer"
    run = z1 - z0 - 2.0 * SEAM_BOLT_EDGE
    if run < washer_d:
        return ((z0 + z1) / 2.0,)
    count = math.ceil(run / SEAM_BOLT_PITCH_MAX) + 1
    return tuple(z0 + SEAM_BOLT_EDGE + i * run / (count - 1) for i in range(count))


def seam_bolt_name(k: int, i: int) -> str:
    """Leaf name of the i-th bolt (from the bottom) at seam k."""
    return f"seam_bolt_{k}_{i}"


def _seam_holes(r_out: float, ear_z: tuple[float, float], apex_up: bool, ear_t: float) -> Part:
    """Cutter for one seam's bolt holes through both tabs, in the seam
    frame (seam plane y = 0, radial +X)."""
    x = r_out + SEAM_BOLT_R
    cutters = [
        teardrop_cutter(
            SEAM_BOLT_HOLE_D,
            (x, -ear_t - 1.0, z),
            (0, 1, 0),
            2.0 * ear_t + 2.0,
            apex_up=apex_up,
            roof_deg=HOLE_ROOF_DEG,
        )
        for z in seam_bolt_zs(*ear_z)
    ]
    return reduce(operator.add, cutters)


def _rod(d: float, y0: float, y1: float) -> Part:
    """A cylinder of diameter ``d`` along +Y from y0 to y1."""
    return Pos(0, (y0 + y1) / 2.0, 0) * Rot(90, 0, 0) * Cylinder(d / 2.0, y1 - y0)


@cache
def _screw() -> SocketHeadCapScrew:
    return SocketHeadCapScrew(size=SEAM_BOLT_SIZE, length=SEAM_BOLT_L, fastener_type="iso4762", simple=True)


@cache
def _nut() -> HexNut:
    return HexNut(size=SEAM_BOLT_SIZE, fastener_type=SEAM_NUT_TYPE, simple=True)


@cache
def _washer() -> PlainWasher:
    return PlainWasher(size=SEAM_WASHER_SIZE, fastener_type=SEAM_WASHER_TYPE)


def _stack_heights(ear_t: float) -> tuple[float, float]:
    """(head top, nut face) along the bolt axis from the seam plane:
    where the two tool corridors start."""
    seat = ear_t + _washer().washer_thickness
    return seat + _screw().head_height, -(seat + _nut().nut_thickness)


# bd_warehouse stacks along +Z; this turns +Z onto the seam frame's +Y.
_ALONG_Y = Rot(-90, 0, 0)


@cache
def seam_bolt(ear_t: float = SEAM_EAR_T) -> Part:
    """Screw, two washers and nut fused into one body, in the seam frame
    at its own axis (along +Y, origin on the seam plane): head and washer
    on the +y tab, washer and nut on the −y tab. Bought, not inspected."""
    seat = ear_t + _washer().washer_thickness
    nut_foot = -seat - _nut().nut_thickness
    tip = seat - SEAM_BOLT_L
    assert tip <= nut_foot - 1.0, "seam bolt too short to take its nut"
    # bd_warehouse's simple screw draws the shank under the thread's major
    # Ø, so its nut floats 0.27 off it. The SEAM_BOLT_D cylinder is the
    # thread's outer envelope and the nut's bore, and fuses the stack.
    stack = (
        Pos(0, 0, ear_t) * _washer()
        + Pos(0, 0, seat) * _screw()
        + Pos(0, 0, (tip + seat) / 2.0) * Cylinder(SEAM_BOLT_D / 2.0, seat - tip)
        + Pos(0, 0, -seat) * _washer()
        + Pos(0, 0, nut_foot) * _nut()
    )
    assert len(stack.solids()) == 1, "seam bolt stack did not fuse into one body"
    return _ALONG_Y * stack


def seam_tools(ear_t: float = SEAM_EAR_T) -> Part:
    """The hex key's way in to the head and the nut driver's to the nut,
    each SEAM_TOOL_L out along the axis, in ``seam_bolt``'s frame. A
    keep-out, never placed."""
    head_top, nut_face = _stack_heights(ear_t)
    return _rod(SEAM_DRIVER_D, head_top, head_top + SEAM_TOOL_L) + _rod(
        SEAM_NUT_TOOL_D, nut_face - SEAM_TOOL_L, nut_face
    )


def seam_bolt_locations(
    phase_deg: float,
    r_out: float,
    ear_z: tuple[float, float],
    n: int = SEAM_COUNT,
) -> tuple[tuple[int, int, Location], ...]:
    """(seam k, bolt i, location) for every seam bolt of a ring split by
    ``split_ring`` with the same arguments, m02-local."""
    return tuple(
        (k, i, Rot(0, 0, az) * Pos(r_out + SEAM_BOLT_R, 0, z))
        for k, az in enumerate(seam_azimuths(phase_deg, n))
        for i, z in enumerate(seam_bolt_zs(*ear_z))
    )


def with_seam_bolts(
    a: Assembly,
    sectors: Sequence[str],
    material_of: Callable[[str], str | None],
    *,
    phase_deg: float,
    r_out: float,
    ear_z: tuple[float, float],
) -> Assembly:
    """Place every seam bolt of the ring whose sectors (in order) are
    ``sectors`` and hold its claims: it bears on the two tabs it joins
    (tangent contact with each sector — the washers' faces), and its
    tool corridors (``seam_bolt_{k}_{i}_tools``, a named keep-out) keep
    SEAM_TOOL_MARGIN clear of everything else in ``a`` — the ring's
    own parts. The rest of the machine is held in `cad/assembly.py`."""
    n = len(sectors)
    placed = seam_bolt_locations(phase_deg, r_out, ear_z, n)
    for k, i, loc in placed:
        name = seam_bolt_name(k, i)
        a = a.with_part(name, seam_bolt(), location=loc, material=material_of(name))
    everything = tuple(p.name for p in a.placed_parts)
    for k, i, loc in placed:
        name = seam_bolt_name(k, i)
        a = (
            a.assert_tangent_contact(name, sectors[k])
            .assert_tangent_contact(name, sectors[(k - 1) % n])
            .with_keepout(f"{name}_tools", loc * seam_tools())
            .assert_clear_of(
                [p for p in everything if p != name],
                f"{name}_tools",
                name=f"{name}_tools_own",
                min_mm=SEAM_TOOL_MARGIN,
            )
        )
    return a


# Every seam-bolt leaf name any ring can carry, for the tag and material
# tables: the hopper liner's ear (SKIRT_TOP_Z up to HOPPER_WALL_H) is
# the tallest. A taller one elsewhere is caught by `paint`'s coverage
# check, which names the missing leaf.
SEAM_BOLT_NAMES = tuple(
    seam_bolt_name(k, i) for k in range(SEAM_COUNT) for i in range(len(seam_bolt_zs(SKIRT_TOP_Z, HOPPER_WALL_H)))
)
