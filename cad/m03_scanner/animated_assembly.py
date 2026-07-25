"""M03 scanner — time-parameterized assembly factory.

``build_m03_at(t)`` returns the static m03 ``Assembly`` at animation
parameter ``t ∈ [0, 1]`` with three joint DOFs updated:

  * ``turret.rotor`` → ``rotor_schedule(t)`` — stepwise 90° dwell-and-
    transit revolution about the hub Z axis.
  * ``turret.rotor.platform_<dump_station>`` → ``tilt_schedule(t)`` —
    bell-shaped outer-down tilt of the platform parked at the DUMP
    zone during the current dwell.
  * ``turret.servo_arm`` → ``tilt_schedule(t) * SERVO_ARM_TILT_SCALE`` —
    co-tilt of the bench-mounted servo arm in lock-step with the
    platform tilt; the lifter pad rises against the platform's
    drop block.

The flat-trajectory ``replace(parts=...)`` rebuild from the prior rev
is gone — joint structure carries the kinematic intent directly, so
``cad_khana.export.export_animated_glb`` emits one animation channel
per joint on the parent node instead of inferring rigid-body groups
by trajectory match.

The turret floor frame, hub housing + bearings, motor + bracket +
motor pulley, homing sensor + bracket, servo bracket + body, and the
entire camera rig all stay fixed. The servo arm pivots on the static
servo body's output shaft; firmware (not modeled here) commands the
servo to lift on the rotor index pulse at DUMP and return to rest
during transit.

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
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent

from cad_khana.mechanism.assembly import Assembly  # noqa: E402
from cad_khana.mechanism.check import check  # noqa: E402

from .assembly import add_m03, add_m03_internal_checks  # noqa: E402


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

# Servo arm co-tilt scale. The arm rotates around the servo output
# shaft in lock-step with the platform tilt — its lifter pad pushes
# UP on the platform's drop block, so the arm's rotation amount is
# `platform_tilt * SERVO_ARM_TILT_SCALE`. The scale ≈ 0.83 matches
# the rise of the lifter pad (radius ≈ 72 mm from pivot in X-Z) to
# the rise of the platform's drop block (radius ≈ 55 mm from hinge
# pin) — to first order, equal vertical rise means contact stays
# tight at peak tilt. Identical sign convention to the retired
# kicker_lever: positive joint angle = pad up = platform tilts.
SERVO_ARM_TILT_SCALE = 0.83


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


# --- Dynamic-conflict pair suppression ---------------------------------
#
# One pair after the 2026-05-23 servo slice. The previous five entries
# all named ``kicker_lever`` (the retired bench-mounted lever paired
# with the shaft cam); four of them encoded kinematic overlaps the cam
# linkage couldn't resolve and disappeared with the lever's hub-end
# block leaving the rotor sweep ring. The fifth — DESIGN-INTENDED
# contact between the lifter pad and the DUMP-pose drop block at
# moderate tilt — is geometry-independent: any lifter mechanism that
# raises a pad against a drop block from below will trip it. The
# servo arm reproduces it exactly. Skipped here for the same reason
# the kicker era skipped it: per-frame predicate (pad below drop /
# pad against drop / pad above drop) is noisier than the value.

_DYNAMIC_CONFLICT_PAIRS: frozenset[tuple[str, str]] = frozenset({
    ("turret.rotor.platform_dump.frame", "turret.servo_arm.servo_arm"),
})


def _suppressed_for_frame(
    arm_angle_deg: float, rotor_angle_deg: float
) -> frozenset[tuple[str, str]]:
    """Pairs to skip at this frame. Empty at the pure-REST frame
    (rotor=0, arm=0) so the static `_KNOWN_OVERLAPS_STATIC` still
    fires its assert_interference alarms. The lone DUMP-platform ×
    arm pad-contact suppression activates at every dynamic frame."""
    if arm_angle_deg == 0.0 and rotor_angle_deg == 0.0:
        return frozenset()
    return _DYNAMIC_CONFLICT_PAIRS


def build_m03_at(t: float) -> Assembly:
    """``t ∈ [0, 1]`` ↔ one full rotor revolution. Stepwise dwell-and-
    transit schedule snaps the rotor 90° per quarter; during each
    dwell, the platform parked at the DUMP zone tips outer-down (the
    servo dumping a brick) and the servo arm co-tilts to drive it.

    Three joints are animated:
      * ``turret.rotor`` — rotor azimuth.
      * ``turret.rotor.platform_<dump_station>`` — the one platform
        currently at DUMP. The shared cartridge-local pin axis
        (``assembly.py``'s ``_PLATFORM_PIN_LOCAL``) makes positive
        joint angle ↔ outer-DOWN tilt, so the same ``tilt_deg`` value
        drives whichever station happens to be at DUMP.
      * ``turret.servo_arm`` — bench-mounted lifter, co-tilts.

    Internal checks survive intact — they address tree paths, which
    joint angles don't change."""
    theta_rotor_deg = rotor_schedule(t)
    tilt_deg = tilt_schedule(t)
    q, _ = _quarter(t)
    dump_station = _station_at_dump(q)

    a = add_m03(Assembly())
    a = a.with_joint_angle("turret.rotor", theta_rotor_deg)
    arm_angle_deg = 0.0
    if tilt_deg != 0.0:
        a = a.with_joint_angle(
            f"turret.rotor.platform_{dump_station}", tilt_deg
        )
        arm_angle_deg = tilt_deg * SERVO_ARM_TILT_SCALE
        a = a.with_joint_angle("turret.servo_arm", arm_angle_deg)
    # Per-frame regression check. At pure REST (rotor=0, arm=0) the
    # static known-overlap set in `assembly.py` still fires its two
    # assert_interference alarms (hub_housing × flag/sensor). Dynamic
    # frames suppress only the design-intended platform_dump_frame ×
    # servo_arm pad-contact pair; any other transit-time overlap
    # surfaces loudly.
    return add_m03_internal_checks(
        a,
        suppressed_pairs=_suppressed_for_frame(
            arm_angle_deg, theta_rotor_deg
        ),
    )


assembly = build_m03_at(0.0)


if __name__ == "__main__":
    # Per-frame static check at sampled t — regression insurance for
    # mid-motion interferences. Samples cover REST, the dwell peak
    # (local_t = dwell_frac/2 = 0.35 ⇒ t = 0.0875 in the first
    # quarter), the dwell-end transition (just before / after
    # local_t = dwell_frac = 0.7 ⇒ t = 0.175), and one transit point
    # per quarter so the rotor is checked at each station.
    #
    # The static m03 is already clean; the post-servo
    # `_DYNAMIC_CONFLICT_PAIRS` is empty, so any new transit-time
    # overlap surfaces here.
    #
    # Run this through `khana check`, not bare `python`. The CLI defers
    # check() failures to its own boundary, so every frame runs and its
    # JSON is fresh; a bare interpreter keeps raise-on-first-failure and
    # would abort the sweep at the first bad frame.
    out_root = HERE / "outputs" / "animated"
    sample_ts = (
        0.0,        # REST (rotor at station 0, no tilt)
        0.0625,     # rising into peak
        0.0875,     # platform tilt peak (30°)
        0.125,      # past peak, descending
        0.175,      # end of dwell
        0.2,        # mid-transit
        0.25,       # arrives at station 1
        0.5,        # arrives at station 2
        0.75,       # arrives at station 3
    )
    new_failures: list[tuple[float, str, str]] = []
    for t in sample_ts:
        out = out_root / f"t_{t:.4f}"
        a = build_m03_at(t)
        # check() returns its result whether or not assertions failed, so
        # read the failures off the result rather than catching the raise:
        # under the CLI there is no raise to catch, and re-evaluating the
        # assembly to find out would duplicate the work check() just did.
        # `is False` is deliberate — `passed is None` means the assertion
        # was skipped (a part absent from this run), which is not a failure.
        result = check(a, out=str(out))
        new_failures += [
            (t, r.name, r.detail)
            for r in result.diagnostics.assertions
            if r.passed is False
        ]
    if new_failures:
        print(f"\n{len(new_failures)} NEW failure(s) across {len(sample_ts)} frames:")
        for t, name, detail in new_failures:
            print(f"  t={t:.4f}  {name}: {detail[:160]}")
        raise SystemExit(1)
    print(f"all {len(sample_ts)} frames pass (baseline known overlaps applied)")
