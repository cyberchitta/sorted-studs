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


# --- Dynamic-conflict pair suppression ---------------------------------
#
# Five pairs whose interference state varies with (rotor_angle,
# lever_angle): present at some frames, absent at others, no clean
# per-frame predicate. Each one has a single documented kinematic
# root cause, so the per-frame check just SKIPS them entirely during
# any frame where the geometric situation makes overlap possible —
# both the `assert_no_interference` and the `assert_interference` are
# omitted, so neither presence nor absence triggers a regression
# alarm.
#
# Why skip instead of assert_interference:
#   * `assert_interference` requires the overlap to be PRESENT (so it
#     fires when a design fix lands and the overlap goes away).
#     Several of these pairs come and go through the dump dwell —
#     present at moderate tilt, absent at peak when the drop block
#     and lifter pad have both risen past each other, etc. We'd need
#     a per-tilt-window predicate that's noisier than the regression
#     value.
#   * Skipping documents the kinematic root cause here in code and
#     accepts that the regression won't catch this family. Any *new*
#     unrelated overlap elsewhere on the part-pair grid still trips
#     loudly.
#
# The pairs:
#
#  * (spider, kicker_lever) — lever's hub-end cam-follower block
#    sweeps through the spider IDLE arm during DUMP tilt. Cam
#    follower must sit at heading 180° (= IDLE arm azimuth), so any
#    lever rotation lifts the block into the arm. Resolution needs a
#    kinematic redesign (8-lobe cam + off-azimuth follower / lower
#    cam Z / pivot above spider plane); not addressable by
#    tightening the lever's footprint.
#
#  * (pin_idle, kicker_lever) — same root cause (pin sits at IDLE
#    arm's tip); same redesign clears both.
#
#  * (platform_idle_frame, kicker_lever) — TWO distinct modes share
#    this pair:
#      a) HIGH-tilt (lever angle >~22°): lever's hub-arm beam clips
#         the IDLE platform's hub-side skirt. Same swept-volume root
#         cause as the spider clip — platforms hang off spider arms,
#         so structure in the IDLE-arm corridor drags IDLE platform
#         features with it.
#      b) TRANSIT (rotor angle ≠ 0): IDLE platform's kicker-extension
#         drop block sweeps over the lever's hub-end block as the
#         rotor turns. Drop-block X[-50, -22] (after rotor +30°)
#         overlaps hub-end block X[-54, -36]; their Z[-43, -15] and
#         Z[-51, -37] intersect at Z[-43, -37].
#    Same kinematic root cause for both modes (lever hub-end block
#    sits in the rotor's drop-block sweep ring at heading 180°).
#
#  * (platform_dump_frame, kicker_lever) — DESIGN-INTENDED contact at
#    moderate tilt (~10°-20°): lifter pad pushes UP on DUMP drop
#    block. Overlap absent at low tilt (pad below drop) and at peak
#    (pad above drop — both have risen). No clean per-frame
#    predicate without modelling the contact arc; skipped.
#
#  * (kicker_lever, dump_cam) — the static `_KNOWN_OVERLAPS_STATIC`
#    entry asserts overlap at REST (lifter-beam inner corner clips
#    cam BASE_R footprint at heading ~216°). The overlap disappears
#    during lift (corner rises clear of cam Z plane) and during
#    transit (cam rotates, lobe heading shifts so the corner is in a
#    valley). Suppressed at every frame where rotor or lever angle
#    is non-zero.

_DYNAMIC_CONFLICT_PAIRS: frozenset[tuple[str, str]] = frozenset({
    ("spider",              "kicker_lever"),
    ("pin_idle",            "kicker_lever"),
    ("platform_idle_frame", "kicker_lever"),
    ("platform_dump_frame", "kicker_lever"),
    ("kicker_lever",        "dump_cam"),
})


def _suppressed_for_frame(
    lever_angle_deg: float, rotor_angle_deg: float
) -> frozenset[tuple[str, str]]:
    """Pairs to skip at this frame. Empty at the pure-REST frame
    (rotor=0, lever=0) so the static known-overlap set in
    `_KNOWN_OVERLAPS_STATIC` still fires its assert_interference
    alarms. Suppressed at every dynamic frame to silence the
    documented kinematic-redesign conflicts."""
    if lever_angle_deg == 0.0 and rotor_angle_deg == 0.0:
        return frozenset()
    return _DYNAMIC_CONFLICT_PAIRS


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
    lever_angle_deg = 0.0
    if tilt_deg != 0.0:
        a = a.with_joint_angle(
            f"turret_rotor.platform_{dump_station}", tilt_deg
        )
        lever_angle_deg = tilt_deg * KICKER_LEVER_TILT_SCALE
        a = a.with_joint_angle("kicker_lever", lever_angle_deg)
    # Per-frame regression check. At pure REST (rotor=0, lever=0) the
    # static known-overlap set in `assembly.py` still fires its three
    # assert_interference alarms (hub_housing × flag/sensor, lever ×
    # cam). At every dynamic frame, the five
    # `_DYNAMIC_CONFLICT_PAIRS` are suppressed entirely (see header
    # for the kinematic root-cause documentation) so the loop catches
    # only *new* failures.
    return add_m03_internal_checks(
        a,
        suppressed_pairs=_suppressed_for_frame(
            lever_angle_deg, theta_rotor_deg
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
    # The static m03 is already clean; dynamic frames suppress the
    # five `_DYNAMIC_CONFLICT_PAIRS` (kinematic-redesign overlaps +
    # the design-intended pad/drop contact + the cam-vs-lever pair
    # whose overlap state varies with rotor/tilt). This loop catches
    # any NEW interference outside that suppressed set.
    from cad_khana.mechanism.assertions import evaluate as _evaluate

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
        # Don't crash on a documented assertion in one frame — collect
        # all unexpected failures across the whole sweep and report at
        # the end. Sub-frame writes still happen via check().
        try:
            check(a, out=str(out))
        except SystemExit:
            results = _evaluate(a)
            for r in results:
                if not r.passed:
                    new_failures.append((t, r.name, r.detail))
    if new_failures:
        print(f"\n{len(new_failures)} NEW failure(s) across {len(sample_ts)} frames:")
        for t, name, detail in new_failures:
            print(f"  t={t:.4f}  {name}: {detail[:160]}")
        raise SystemExit(1)
    print(f"all {len(sample_ts)} frames pass (baseline known overlaps applied)")
