"""M03 scanner — time-parameterized assembly factory.

`build_m03_at(t)` returns the static m03 ``Assembly`` at animation
parameter ``t ∈ [0, 1]``, with the rotor (spider + platforms + cam +
shaft + flag + drive pulley) revolved around the hub Z axis on a
stepwise dwell/transit schedule, and the platform currently at the
DUMP heading tipping outer-down within its dwell to mime the kicker
dumping a brick.

The turret floor frame, hub housing, bearings, kicker (mount + lever
+ pin), drive motor + bracket, and homing sensor + bracket all stay
fixed — they're bolted to the deck, not the spider. The dump cam
**is** on the rotor (it's keyed to the hub shaft); the kicker lever
is the static follower bolted to the frame that rides over the cam
profile as it sweeps past.

Used by the GLB animation exporter (see ``export_glb_animated.py``)
and as the unit of regression insurance for `khana check` across
mid-motion frames.

The t→Assembly model is the project's animation primitive: motion is
expressed mathematically here (``theta_rotor_deg = rotor_schedule(t)``,
``tilt_deg = tilt_schedule(t)``), the cad-khana sweep is a thin
wrapper that samples ``factory(t)`` at N values.
"""

from __future__ import annotations

import math
import sys
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from build123d import Pos, Rot  # noqa: E402

from cad_khana.mechanism.assembly import Assembly  # noqa: E402
from cad_khana.mechanism.check import check  # noqa: E402

from assembly import (  # noqa: E402
    TURRET_BASE,
    TURRET_ARM_NAMES,
    TURRET_DUMP_CAM_NAMES,
    TURRET_PLATFORM_NAMES,
    add_m03,
    add_m03_internal_checks,
)
from params import TURRET_HINGE_PIN_RADIAL, TURRET_HINGE_PIN_Z  # noqa: E402

# Parts that revolve with the spider. The dump cam is keyed to the
# hub shaft and rides the rotor; the kicker lever (and its mount +
# pin) is the static follower bolted to the frame. Hub bearings stay
# with the housing — physically the outer race is static and the
# inner spins with the shaft, but the bearing-as-a-whole is rendered
# as one body, and the visual cost of pinning it to the housing side
# is nil.
ROTATING_TURRET_PARTS = frozenset(
    (
        "hub_shaft",
        "drive_pulley_shaft",
        "homing_flag",
        *TURRET_ARM_NAMES,
        *TURRET_PLATFORM_NAMES,
        *TURRET_DUMP_CAM_NAMES,
    )
)

# Number of stations on the turret (= number of dwells per
# revolution). The scanner is a 4-station turret; the dwell/transit
# schedule below assumes this.
N_STATIONS = 4

# Fraction of each station's quarter spent dwelling (vs transiting to
# the next station). 0.7 over 8 s gives 1.4 s dwell + 0.6 s transit.
DWELL_FRAC = 0.7

# Peak platform tilt during a dump dwell. ~30° outer-down is enough
# to send a brick sliding into the diverter on the acrylic top
# surface (acrylic-on-PLA static friction coefficient ≈ 0.3 → tan
# 30° = 0.577 > 0.3). Tune by eye in the subsite preview.
MAX_TILT_DEG = 30.0

# Per-station hinge pin geometry in **turret-local** (= before
# TURRET_BASE lifts the unit into m03-local). For each static station
# the hinge pin sits at radius TURRET_HINGE_PIN_RADIAL from the hub
# axis along its arm's radial line. The tilt factory returns the
# rotation that tips the platform's radially-outward edge toward -Z
# (positive α = outer-down regardless of station): outer-edge
# directions are LOAD +X, IMAGE +Y, IDLE -X, DUMP -Y and pin axes
# alternate ±Y, ±X — signs derived from the right-hand rule on
# `pin_axis × outer`. Applied in the static turret-local frame
# (before rotor rotation), so the pin axis is always X- or Y-aligned
# and a plain Euler ``Rot(...)`` is enough — no arbitrary-axis math.
# The build_m03_at composition lifts the rotation into m03-local via
# conjugation by TURRET_BASE (see _tilt_xform below).
_STATION_TILT = {
    # static heading → (pin position in turret-local, tilt rotation factory)
    "load":  ((+TURRET_HINGE_PIN_RADIAL, 0.0, TURRET_HINGE_PIN_Z),
              lambda a: Rot(0, +a, 0)),
    "image": ((0.0, +TURRET_HINGE_PIN_RADIAL, TURRET_HINGE_PIN_Z),
              lambda a: Rot(-a, 0, 0)),
    "idle":  ((-TURRET_HINGE_PIN_RADIAL, 0.0, TURRET_HINGE_PIN_Z),
              lambda a: Rot(0, -a, 0)),
    "dump":  ((0.0, -TURRET_HINGE_PIN_RADIAL, TURRET_HINGE_PIN_Z),
              lambda a: Rot(+a, 0, 0)),
}


def _quarter(t: float) -> tuple[int, float]:
    """``t ∈ [0, 1] → (quarter index 0..3, local_t ∈ [0, 1)``).
    Maps ``t == 1.0`` back to ``(0, 0.0)`` so the animation loops
    cleanly."""
    if t >= 1.0:
        return 0, 0.0
    q = int(t * N_STATIONS)
    return q, t * N_STATIONS - q


def rotor_schedule(t: float, *, dwell_frac: float = DWELL_FRAC) -> float:
    """Stepwise rotor angle. Dwells for ``dwell_frac`` of each
    quarter (platform parked at a station, camera takes a picture),
    then snaps 90° to the next station over the remaining
    ``1 - dwell_frac`` of the quarter."""
    q, local_t = _quarter(t)
    if local_t < dwell_frac:
        sub = 0.0
    else:
        sub = (local_t - dwell_frac) / (1.0 - dwell_frac)
    return 90.0 * (q + sub)


def tilt_schedule(
    t: float,
    *,
    dwell_frac: float = DWELL_FRAC,
    max_deg: float = MAX_TILT_DEG,
) -> float:
    """Bell-shaped platform tilt within each dwell window; zero during
    transit. Smooth ``sin²`` bump centred at the dwell midpoint so the
    platform tips up and back down within the dwell."""
    _q, local_t = _quarter(t)
    if local_t >= dwell_frac:
        return 0.0
    s = local_t / dwell_frac
    return max_deg * math.sin(math.pi * s) ** 2


# Static heading (deg, around +Z from +X toward +Y) of each station's
# platform at θ=0. The platform whose static heading equals
# ``270° - 90°*q`` is the one parked at the DUMP zone during dwell q.
_STATION_STATIC_HEADING = {
    "load":  0,
    "image": 90,
    "idle":  180,
    "dump":  270,
}


def _station_at_dump(q: int) -> str:
    """Which station's platform is currently parked at the DUMP zone
    during dwell q. Inverse of ``static_heading == 270 - 90*q (mod 360)``."""
    target = (270 - 90 * q) % 360
    for name, h in _STATION_STATIC_HEADING.items():
        if h == target:
            return name
    raise AssertionError(f"no station with static heading {target}°")


def build_m03_at(t: float) -> Assembly:
    """``t ∈ [0, 1]`` ↔ one full rotor revolution. Stepwise dwell-and-
    transit schedule snaps the rotor 90° per quarter; during each
    dwell, the platform parked at the DUMP zone tips outer-down (the
    kicker dumping a brick).

    Composition order (right-to-left, applied to static m03-local
    placement):
        rotor_xform * tilt_xform * static_location

    Where ``rotor_xform`` is a rotation around the hub Z axis
    (m03-local) and ``tilt_xform`` is a per-station rotation around
    the DUMP-bound platform's hinge pin. Both are pre-multiplied,
    leaving each part's static placement intact. The tilt is applied
    in the **static** turret-local frame (before rotor rotation) so
    the pin axis is always X- or Y-aligned — plain Euler ``Rot(...)``
    suffices, no arbitrary-axis math.

    Internal checks are added afterward; they're name-based, so the
    location shuffle leaves them well-formed."""
    theta_rotor_deg = rotor_schedule(t)
    tilt_deg = tilt_schedule(t)
    q, _ = _quarter(t)
    dump_station = _station_at_dump(q)

    a = add_m03(Assembly())
    # Strip back to just the placements; we'll re-add assertions last.
    a = replace(a, assertions=())

    tilt_parts: dict[str, object] = {}
    if tilt_deg != 0.0:
        pin_pos, tilt_factory = _STATION_TILT[dump_station]
        # Conjugate the turret-local pivot into m03-local. TURRET_BASE
        # is a pure translation, so the pin axis direction is
        # preserved; only the pivot point shifts. For LOAD/IDLE the
        # pivot already lies on the correct axis line (axis ±Y,
        # shift along Y is a no-op) but IMAGE/DUMP have axis ±X and
        # need the explicit lift.
        pin_in_m03 = TURRET_BASE * Pos(*pin_pos)
        tilt_xform = pin_in_m03 * tilt_factory(tilt_deg) * pin_in_m03.inverse()
        for sub in ("acrylic", "frame"):
            tilt_parts[f"platform_{dump_station}_{sub}"] = tilt_xform

    rotor_xform = None
    if theta_rotor_deg != 0.0:
        rotor_xform = (
            TURRET_BASE * Rot(0, 0, theta_rotor_deg) * TURRET_BASE.inverse()
        )

    def _new_location(p):
        loc = p.location
        if p.name in tilt_parts:
            loc = tilt_parts[p.name] * loc
        if rotor_xform is not None and p.name in ROTATING_TURRET_PARTS:
            loc = rotor_xform * loc
        return loc

    a = replace(
        a,
        parts=tuple(replace(p, location=_new_location(p)) for p in a.parts),
    )
    a = add_m03_internal_checks(a)
    return a


assembly = build_m03_at(0.0)


if __name__ == "__main__":
    # Per-frame static check at sampled t — regression insurance for
    # mid-motion interferences. The static m03 is already clean; this
    # catches any rotor pose where the spider or a platform pin clips a
    # rig member.
    out_root = HERE / "outputs" / "animated"
    for t in (0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875):
        check(build_m03_at(t), out=str(out_root / f"t_{t:.3f}"))
