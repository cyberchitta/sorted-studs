"""M03 scanner — time-parameterized assembly factory.

``build_m03_at(t)`` returns the static m03 ``Assembly`` at animation
parameter ``t ∈ [0, 1]`` with three joint DOFs updated:

  * ``turret_rotor`` → ``rotor_schedule(t)`` — stepwise 90° dwell-and-
    transit revolution about the hub Z axis.
  * ``turret_rotor.platform_<dump_station>`` → ``tilt_schedule(t)`` —
    bell-shaped outer-down tilt of the platform parked at the DUMP
    zone during the current dwell.
  * ``kicker_lever`` → ``tilt_schedule(t) * KICKER_LEVER_TILT_SCALE``
    — co-tilt of the bench-mounted lever in lock-step with the
    platform tilt; the lifter pad rises against the platform's
    kicker extension.

The flat-trajectory ``replace(parts=...)`` rebuild from the prior rev
is gone — joint structure carries the kinematic intent directly, so
``cad_khana.export.export_animated_glb`` emits one animation channel
per joint on the parent node instead of inferring rigid-body groups
by trajectory match.

The turret floor frame, hub housing + bearings, motor + bracket +
motor pulley, homing sensor + bracket, kicker mount + pin, and the
entire camera rig all stay fixed. The dump cam **is** on the rotor
(keyed to the hub shaft); the kicker lever is the static follower
that picks up the cam profile.

Used by the GLB animation exporter (see ``export_glb_animated.py``)
and as the unit of regression insurance for ``khana check`` across
mid-motion frames.

The t→Assembly model is the project's animation primitive: motion is
expressed mathematically here (``theta_rotor_deg = rotor_schedule(t)``,
``tilt_deg = tilt_schedule(t)``), and the cad-khana sweep is a thin
wrapper that samples ``factory(t)`` at N values.
"""

from __future__ import annotations

import math
import sys
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from cad_khana.mechanism.assembly import Assembly  # noqa: E402
from cad_khana.mechanism.check import check  # noqa: E402

from assembly import add_m03, add_m03_internal_checks  # noqa: E402


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

# Kicker lever co-tilt scale. The lever rotates around the kicker pin
# in lock-step with the platform tilt — its lifter pad pushes UP on
# the platform's kicker extension, so the lever's tilt amount is
# `platform_tilt * KICKER_LEVER_TILT_SCALE`. The scale ≈ 0.83 matches
# the rise of the lifter pad (radius ≈ 96 mm from pivot) to the rise
# of the platform's kicker extension drop block (radius ≈ 55 mm from
# its hinge pin) — to first order, equal vertical rise means contact
# stays tight at peak tilt.
KICKER_LEVER_TILT_SCALE = 0.83


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
    kicker dumping a brick) and the kicker lever co-tilts to drive it.

    Three joints are animated:
      * ``turret_rotor`` — rotor azimuth.
      * ``turret_rotor.platform_<dump_station>`` — the one platform
        currently at DUMP. Per-station axis direction (set in
        ``assembly.py``'s ``_PLATFORM_PIN_TURRET``) makes positive
        joint angle ↔ outer-DOWN tilt, so the same ``tilt_deg`` value
        drives whichever station happens to be at DUMP.
      * ``kicker_lever`` — bench-mounted follower, co-tilts.

    Internal checks survive intact — they're name-based; nested joint
    composition leaves part names unchanged."""
    theta_rotor_deg = rotor_schedule(t)
    tilt_deg = tilt_schedule(t)
    q, _ = _quarter(t)
    dump_station = _station_at_dump(q)

    a = add_m03(Assembly())
    a = a.with_joint_angle("turret_rotor", theta_rotor_deg)
    if tilt_deg != 0.0:
        a = a.with_joint_angle(
            f"turret_rotor.platform_{dump_station}", tilt_deg
        )
        a = a.with_joint_angle(
            "kicker_lever", tilt_deg * KICKER_LEVER_TILT_SCALE
        )
    # Re-add the static internal-check pair set so per-frame regression
    # still catches mid-motion overlaps. The pre-existing baseline at
    # tilt frames carries five intentionally-failing assertions
    # (spider × platform_dump_frame, spider × kicker_lever, pin_idle ×
    # kicker_lever, platform_dump_frame × kicker_lever, plus the loss
    # of the c7 dump_cam × kicker_lever overlap as the lever lifts);
    # the regression value is in surfacing *new* failures, not
    # rebuilding the baseline.
    return add_m03_internal_checks(a)


assembly = build_m03_at(0.0)


if __name__ == "__main__":
    # Per-frame static check at sampled t — regression insurance for
    # mid-motion interferences. The static m03 is already clean; this
    # catches any rotor pose where the spider or a platform pin clips a
    # rig member.
    out_root = HERE / "outputs" / "animated"
    for t in (0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875):
        check(build_m03_at(t), out=str(out_root / f"t_{t:.3f}"))
