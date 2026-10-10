"""
sorted-studs — the top feed: m03's DUMP platform tipping a brick into
the flare on m05's top column. A CHECK module: the machine's three
units, the dump tilt declared on top, and probes for where the brick
goes.

    khana check cad/check_dump_feed.py

`khana check cad/assembly.py` holds m03 clear of m05 at the rest pose
only — it drives no joint — and says nothing about where a brick lands.
This target adds both, on the three units placed as `build_top` places
them (its cross-unit pair claims stay at home — see
`build_dump_feed_check`):

1. **The flare clear of everything above it, over the dump tilt.** The
   flare turns with the column but is round, so it is its own rotation
   envelope (m05 exports it as the keep-out `flare_sweep`) and no stack
   turn is needed. What moves is m03: the platform parked at DUMP tips
   outer-down to `MAX_TILT_DEG` and the servo arm lifts with it. m03 and
   the m02 chain stay `FLARE_M03_CLEARANCE` off the sweep at every
   sampled tilt (`DUMP_TILT_STEP_DEG` apart — sampled, so a closer
   approach between two samples is not seen).

2. **Where the brick lands.** The brick's centre of mass leaves the
   platform's outer edge as a point mass and falls. It is in the flare
   if its path stays `DUMP_LANDING_MARGIN` clear of m05's keep-out
   `flare_miss` — outside the rim's inner face, from the rim plane up —
   having started inside the mouth (`dump_exit_inside_mouth`).
   Each probe is the fastest path from one height above the acrylic,
   across the lateral scatter, followed down until it enters the built
   column. A slower path from the same point lies under a faster one,
   down to a dead drop off the edge, so the fastest bounds them all. One
   probe for the lowest centre of mass and one for the highest. The
   fastest and the dead-drop path of each also record the radius at
   which they enter the column (`*_meets_flare_r`).

The exit is geometry, not an assumption: the frame's outer wall is cut
flush with the acrylic over the acrylic's width (`turret_platform`
`exit_edge_cutter`; held on the built frame by its `exit_edge_flush`,
which rides in here four times), so the acrylic plane continued to the
frame's outer face is the top edge a brick leaves over. Until
2026-10-10 a 5 mm rim stood there and this was the check's second
assumption.

Part 2 rests on the assumptions below. None is measured;
`_notes/backlog/bench-validation.md` carries the bench test.

- **The brick leaves at full tilt** (`dump_tilt_lead`): the platform
  reaches `MAX_TILT_DEG` before the nearest-starting brick reaches the
  edge. At a lower tilt the edge is higher and further out, and the
  probes are not built there.
- **It slides the whole way.** Across the acrylic and the 5 mm of
  printed wall top beyond it, at one friction floor (`DUMP_MU_K_MIN`),
  without catching on the joint between them or on the side rims' ends
  either side of the exit.
- **Point mass.** No tumble, no bounce off the funnel or the rim. A
  long brick whose centre lands inside is taken to follow it in.

Scope: the platform named `dump` parked at DUMP (rotor at rest). The
other three cartridges are the same part. The rotor's own turn is not
driven here.
"""

from __future__ import annotations

import math

from build123d import Location, Plane, Polygon, Pos, Vector, extrude

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.motion import Motion

from .assembly import build_top
from .design_systems import DEFAULT_SCHEMA
from .m03_scanner.animated_assembly import MAX_TILT_DEG, SERVO_ARM_TILT_SCALE
from .m03_scanner.params import BRICK_SETTLED_H, PLATFORM_ACRYLIC_SIZE, PLATFORM_ACRYLIC_T
from .m03_scanner.sizing import SCATTER_HALF_MM
from .m05_diverter.assembly import FLOOR_PLAN
from .m05_diverter.ramp_mechanism.assembly import FLARE_MOUTH_R

# --- Clearance ---------------------------------------------------------

# m03 stands on the deck by hand with no locating feature, a few mm of
# placement slop (as `cad/assembly.py` `DECK_EDGE_MARGIN`), and the
# flare turns under it.
FLARE_M03_CLEARANCE = 5.0
DUMP_TILT_STEP_DEG = 1.25

# --- Landing: physical assumptions, all unmeasured ---------------------

G_MM_S2 = 9810.0
# Lowest kinetic friction a brick is taken to have on the acrylic. It
# sets the fastest exit: lower is faster. Handbook ABS on PMMA is
# 0.3..0.5; 0.2 is a guess at a floor (a smooth tile, a dusty platform).
DUMP_MU_K_MIN = 0.2
# Slowest the platform is taken to tilt. An MG90S-class servo is rated
# ~600°/s unloaded; half that is a guess at it lifting the platform.
DUMP_TILT_RATE_MIN_DEG_S = 300.0
# The tilt must be done this many times sooner than the first brick can
# reach the edge.
DUMP_TILT_LEAD_MIN = 1.5
# Where the brick's centre of mass starts: within the landing scatter
# m03 is sized for (`sizing.SCATTER_HALF_MM`, itself a placeholder),
# either way from the platform centre, along the slide and across it.
DUMP_CG_SCATTER = SCATTER_HALF_MM
# Its height above the acrylic: from a plate lying flat (taken as 0) to
# half the settled height m03's sweep is cleared for.
DUMP_CG_H_MAX = BRICK_SETTLED_H / 2.0
# What a path keeps from the miss zone, for what the point mass leaves
# out.
DUMP_LANDING_MARGIN = 5.0

# Slide from the platform centre to the exit edge — the frame's outer
# face, `PLATFORM_ACRYLIC_SIZE` being the frame's outer footprint.
_EDGE_Y = PLATFORM_ACRYLIC_SIZE / 2.0
_SLIDE_MIN = _EDGE_Y - DUMP_CG_SCATTER
_SLIDE_MAX = _EDGE_Y + DUMP_CG_SCATTER

_PATH_STEP_S = 0.0005       # sampling of a path, seconds
_RIBBON_T = 0.5             # probe sheet thickness

_TOP_IDX = next(idx for idx, role in FLOOR_PLAN if role == "top")
_TOP_ROT = f"m05.rotating_stack.f{_TOP_IDX}"
_DUMP_PLATFORM = "m03.turret.rotor.platform_dump"
_SERVO_ARM = "m03.turret.servo_arm"


def slide_accel(tilt_deg: float = MAX_TILT_DEG) -> float:
    """Fastest acceleration down the platform held at a tilt, mm/s²."""
    t = math.radians(tilt_deg)
    return G_MM_S2 * (math.sin(t) - DUMP_MU_K_MIN * math.cos(t))


def exit_speed_max(tilt_deg: float = MAX_TILT_DEG) -> float:
    """Fastest exit, mm/s: the longest slide at that tilt and least
    friction, from rest."""
    return math.sqrt(2.0 * max(slide_accel(tilt_deg), 0.0) * _SLIDE_MAX)


def tilt_lead() -> float:
    """Time for the nearest-starting brick to reach the edge — sliding
    at the full-tilt acceleration from the moment the servo starts, so
    sooner than it can — over the time the platform takes to tilt."""
    t_edge = math.sqrt(2.0 * _SLIDE_MIN / slide_accel())
    t_tilt = MAX_TILT_DEG / DUMP_TILT_RATE_MIN_DEG_S
    return t_edge / t_tilt


def exit_frame(a: Assembly, tilt_deg: float) -> tuple[Vector, Vector, Vector, Vector]:
    """The exit, read off the machine with the DUMP platform posed at a
    tilt: the point on the acrylic's top plane at the middle of the
    frame's outer face, and unit vectors down the slide, across it, and
    up out of the acrylic. The acrylic is a centred box whose +Y is
    radially outward from the turret hub."""
    posed = a.with_joint_angle(_DUMP_PLATFORM, tilt_deg)
    acrylic: Location = posed.part(f"{_DUMP_PLATFORM}.acrylic").location
    here = acrylic.position
    across = (acrylic * Pos(1, 0, 0)).position - here
    along = (acrylic * Pos(0, 1, 0)).position - here
    normal = (acrylic * Pos(0, 0, 1)).position - here
    edge = here + along * _EDGE_Y + normal * (PLATFORM_ACRYLIC_T / 2.0)
    return edge, along, across, normal


def path(
    origin: Vector, along: Vector, speed: float, column, z_floor: float,
    *, across: Vector | None = None, half_w: float = 0.0,
) -> list[Vector]:
    """Point-mass path from `origin`, leaving at `speed` along the unit
    vector `along`, sampled until it enters the placed column's material
    (that sample is the last) or falls under `z_floor`. With `across`
    and `half_w`, the path is a probe sheet's section — itself and a
    copy `half_w` either side, each with its `_RIBBON_T` of thickness —
    and stops when any of it enters."""
    sides = [Vector(0, 0, 0)]
    if across is not None and half_w > 0.0:
        inward = across.cross(Vector(0, 0, 1)) * _RIBBON_T + Vector(0, 0, -_RIBBON_T)
        edges = [across * half_w, across * -half_w]
        sides += edges + [inward] + [e + inward for e in edges]
    pts, t = [origin], 0.0
    while True:
        t += _PATH_STEP_S
        p = origin + along * (speed * t) + Vector(0, 0, -0.5 * G_MM_S2 * t * t)
        pts.append(p)
        if p.Z < z_floor or any(column.is_inside(p + s) for s in sides):
            return pts


def ribbon(pts: list[Vector], across: Vector, half_w: float):
    """A path across the lateral scatter: `pts` (in a vertical plane) as
    a thin sheet, `half_w` either way along the horizontal unit vector
    `across`. The sheet's outer face is the path; its thickness lies
    inside it, toward the axis and down."""
    start = pts[0]
    out = Vector(0, 0, 1).cross(across)             # horizontal, the way the brick goes
    plane = Plane(origin=start, x_dir=out, z_dir=out.cross(Vector(0, 0, 1)))
    line = [((p - start).dot(out), p.Z - start.Z) for p in pts]
    back = [(s - _RIBBON_T, z - _RIBBON_T) for s, z in reversed(line)]
    return extrude(plane * Polygon(*line, *back, align=None), amount=half_w, both=True)


def meets_column_r(pts: list[Vector], column) -> float:
    """Radius from the m05 axis of a path's last sample if that is in
    the placed column's material; 0 if the path fell clean through
    (straight down the throat)."""
    p = pts[-1]
    return math.hypot(p.X, p.Y) if column.is_inside(p) else 0.0


def build_dump_feed_check(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The three units where the machine places them, the dump tilt
    declared, the flare's sweep held clear of m03 and the chain, and the
    landing probes."""
    assert 0.0 <= DUMP_CG_SCATTER < _EDGE_Y, (
        f"scatter ±{DUMP_CG_SCATTER} puts a brick's centre off the platform "
        f"(edge at {_EDGE_Y}): there is no slide to time"
    )
    machine = build_top(schema)
    # The units as `build_top` composed them, without its own claims:
    # the ~1000 cross-unit pairs that touch the tilting platform would
    # each be re-held at every sample (7 min 40 s, green, 2026-10-10)
    # to say what the sweep claims below say about the one m05 part in
    # reach. Each unit's own claims still ride in.
    top = Assembly()
    for node in machine.subassemblies:
        top = top.with_subassembly(node.name, node.assembly, location=node.location)

    # Two claims, split by what moves: the parts the tilt drives are
    # re-measured at every sample, everything else once.
    tilting = sorted(
        pp.name for pp in top.placed_parts
        if pp.name.startswith((f"{_DUMP_PLATFORM}.", f"{_SERVO_ARM}."))
    )
    standing = sorted(
        pp.name for pp in top.placed_parts
        if pp.name.startswith(("m03.", "m02_chain.")) and pp.name not in tilting
    )
    assert tilting, "no part under the dump platform or the servo arm — the tilt claim would be vacuous"
    a = (
        top
        .assert_clear_of(
            tilting, f"{_TOP_ROT}.flare_sweep",
            name="flare_sweep_clear_of_dump_tilt", min_mm=FLARE_M03_CLEARANCE,
        )
        .assert_clear_of(
            standing, f"{_TOP_ROT}.flare_sweep",
            name="flare_sweep_clear_of_standing", min_mm=FLARE_M03_CLEARANCE,
        )
        .assert_scalar(
            "dump_tilt_lead", tilt_lead(), ge=DUMP_TILT_LEAD_MIN,
            detail=f"time to the edge over time to full tilt: µ_k ≥ {DUMP_MU_K_MIN}, "
                   f"tilt ≥ {DUMP_TILT_RATE_MIN_DEG_S:.0f}°/s, start within "
                   f"±{DUMP_CG_SCATTER:.0f} mm — all unmeasured",
        )
        .assert_scalar(
            "dump_exit_speed_max", exit_speed_max(),
            detail=f"mm/s off the edge at {MAX_TILT_DEG:.0f}° after {_SLIDE_MAX:.0f} mm "
                   f"at µ_k {DUMP_MU_K_MIN} — unmeasured",
        )
    )

    edge, along, across, normal = exit_frame(top, MAX_TILT_DEG)
    column_pp = top.part(f"{_TOP_ROT}.column")
    column = column_pp.location * column_pp.part
    z_floor = column.bounding_box().min.Z
    speed = exit_speed_max()

    # `flare_miss` is the way out over the rim, so it only speaks for a
    # path that starts inside the mouth: the exit edge's ends, across
    # the scatter, as a radius from the column's axis.
    axis = column_pp.location.position
    exit_r = max(
        math.hypot(p.X - axis.X, p.Y - axis.Y)
        for p in (edge + across * DUMP_CG_SCATTER, edge - across * DUMP_CG_SCATTER)
    )
    a = a.assert_scalar(
        "dump_exit_inside_mouth", exit_r, le=FLARE_MOUTH_R - DUMP_LANDING_MARGIN,
        detail=f"radius of the exit edge's ends at ±{DUMP_CG_SCATTER:.0f} mm, against "
               f"the mouth radius less the landing margin",
    )

    for name, cg_h in (("flat", 0.0), ("tall", DUMP_CG_H_MAX)):
        origin = edge + normal * cg_h
        fastest = path(origin, along, speed, column, z_floor)
        slowest = path(origin, along, 0.0, column, z_floor)
        # The probe: the fastest path across the scatter, up to the last
        # sample at which none of it has entered the column.
        abreast = path(
            origin, along, speed, column, z_floor,
            across=across, half_w=DUMP_CG_SCATTER,
        )
        probe = f"landing_{name}"
        a = (
            a
            .with_part(probe, ribbon(abreast[:-1], across, DUMP_CG_SCATTER))
            .assert_clear_of(
                probe, f"{_TOP_ROT}.flare_miss",
                name=probe, min_mm=DUMP_LANDING_MARGIN,
            )
        )
        for pace, pts in (("fast", fastest), ("slow", slowest)):
            a = a.assert_scalar(
                f"landing_{pace}_{name}_meets_flare_r", meets_column_r(pts, column),
                detail="radius from the m05 axis where the path (lateral 0) first "
                       "enters the column's material; 0 = straight down the throat",
            )

    return a.with_motion(
        Motion(
            "dump_tilt",
            lambda t: {_DUMP_PLATFORM: t, _SERVO_ARM: t * SERVO_ARM_TILT_SCALE},
            ts=tuple(
                i * DUMP_TILT_STEP_DEG
                for i in range(int(round(MAX_TILT_DEG / DUMP_TILT_STEP_DEG)) + 1)
            ),
        )
    )


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``.
assembly = build_dump_feed_check(DEFAULT_SCHEMA)
