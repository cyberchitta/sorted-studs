"""
M03 turret — unit-internal composition + within-unit checks.

Standalone composition: every turret sub-assembly placed in
turret-local coordinates (hub axis at origin, +Z vertical, +Y toward
the back of the machine when placed in world). Plus pairwise
`assert_no_interference` for every turret-internal part pair.

Sub-assemblies:
  * turret_frame                  — 4-member 2020 floor frame (U-shape:
                                    back rail + 2 side rails + cross
                                    rail; front X-rail dropped in
                                    option a' since the new DUMP at
                                    heading 270° lives over y=0 in
                                    world and a member at y=-R would
                                    block the brick drop into the
                                    diverter column)
  * turret_hub                    — pillow-block + Ø8 shaft + 2 bearings
  * turret_arm                    — single 4-arm spider + 4 stock pins
  * turret_platform (×4 stations) — 4 cartridges, one per station
                                    (IMAGE / LOAD / IDLE / DUMP — option
                                    a' rename, 2026-05-05; firing
                                    heading shifted +90°, IDLE is the
                                    empty-cycle dwell at heading 180°)
  * turret_dump_cam               — printed 4-lobe cam keyed to the shaft
  * turret_dump_kicker            — bench-mounted L-lever (mount + lever
                                    + pivot pin) at DUMP
  * turret_drive                  — NEMA17 + GT2 belt + printed L-bracket
                                    (driven 60T pulley on the shaft,
                                    driver 20T on the motor; 3:1
                                    reduction; bracket bolts to the
                                    cross-rail at +X side)
  * turret_homing                 — printed flag disc keyed to the shaft
                                    + slotted opto-interrupter on a
                                    printed bracket (bolts to the
                                    cross-rail at -X side, between hub
                                    housing and kicker mount)

The cam + kicker make up the active-push tilt mechanism: a cam lobe
arrives at heading 180° at every dwell (the cam-follower contact
heading — independent of the firing-station heading because the cam
is 4-fold symmetric), pushes the lever's hub-end follower outward,
and the lever's lifter pad — cantilevered in -Y past the pivot at
turret (-110, 0) — rises against the DUMP-station platform's tab at
turret (-30, -25) to tilt it outer-down. At rest (cam valley) the
lifter retracts and per-arm hinge stops on the spider hold each
platform level.

This integrate ships REST geometry only (cam in valley orientation,
lever at rest, lifter retracted). Cam-follower kinematics under shaft
rotation — actual lift contact angle, peak-stroke clearance to the
platform tab, brick-tilt sufficiency given the lever ratio — are
deferred to a future kinematic-sweep slice.
"""

import sys
from pathlib import Path

from build123d import Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from turret.turret_arm import assembly as _turret_arm  # noqa: E402
from turret.turret_drive import assembly as _turret_drive  # noqa: E402
from turret.turret_dump_cam import assembly as _turret_dump_cam  # noqa: E402
from turret.turret_dump_kicker import assembly as _turret_dump_kicker  # noqa: E402
from turret.turret_frame import assembly as _turret_frame  # noqa: E402
from turret.turret_homing import assembly as _turret_homing  # noqa: E402
from turret.turret_hub import assembly as _turret_hub  # noqa: E402
from turret.turret_platform import assembly as _turret_platform  # noqa: E402


# --- Public composition builder ----------------------------------------

TURRET_FRAME_NAMES = _turret_frame.TURRET_FRAME_PART_NAMES
TURRET_HUB_NAMES = _turret_hub.HUB_PART_NAMES
TURRET_ARM_NAMES = _turret_arm.ARM_PART_NAMES
TURRET_STATIONS = ("image", "load", "idle", "dump")
TURRET_PLATFORM_NAMES = tuple(
    f"platform_{station}_{sub_name}"
    for station in TURRET_STATIONS
    for sub_name, _ in _turret_platform._IMAGE_LOCS
)
TURRET_DUMP_CAM_NAMES = (_turret_dump_cam.DUMP_CAM_PART_NAME,)
TURRET_DUMP_KICKER_NAMES = _turret_dump_kicker.KICKER_PART_NAMES
TURRET_DRIVE_NAMES = _turret_drive.DRIVE_PART_NAMES
TURRET_HOMING_NAMES = _turret_homing.HOMING_PART_NAMES

TURRET_PART_NAMES = (
    list(TURRET_FRAME_NAMES)
    + list(TURRET_HUB_NAMES)
    + list(TURRET_ARM_NAMES)
    + list(TURRET_PLATFORM_NAMES)
    + list(TURRET_DUMP_CAM_NAMES)
    + list(TURRET_DUMP_KICKER_NAMES)
    + list(TURRET_DRIVE_NAMES)
    + list(TURRET_HOMING_NAMES)
)


def add_turret(a: Assembly, base=Pos(0, 0, 0)) -> Assembly:
    """Add every turret part. Parts are placed in turret-local
    coordinates; `base` (default: identity) is pre-multiplied to lift
    the entire unit into a parent frame (e.g. world)."""
    a = _turret_frame.add_turret_frame(a, base=base)
    a = _turret_hub.add_turret_hub(a, base=base)
    a = _turret_arm.add_turret_arm(a, base=base)
    for station in TURRET_STATIONS:
        a = _turret_platform.add_turret_platform(a, station, base=base)
    a = _turret_dump_cam.add_turret_dump_cam(a, base=base)
    a = _turret_dump_kicker.add_turret_dump_kicker(a, base=base)
    a = _turret_drive.add_turret_drive(a, base=base)
    a = _turret_homing.add_turret_homing(a, base=base)
    return a


# Known overlaps documented at integrate level — pairs that should
# `assert_no_interference` once their underlying design issue lands.
# Each entry is (part_a, part_b, reason). The reason mirrors the live
# state in the queue under "Suggested next CAD moves" §0a.
#
# Slice c7 (2026-05-05) folded these in. The first three are inherited
# c5 latent bugs surfaced when the c5 floor rise narrowed two airspaces
# (housing-top → pulley-bottom and cam-top → spider-bottom) without
# updating the parts that lived in them; the c5 progress entry's
# "khana check passes" claim was true only at standalones with stale
# local mirrors, not at this turret/assembly.py-level integrate. The
# fourth (kicker_lever/dump_cam) is a c7-introduced clash where the
# cam rise pushed the cam Z range into the lifter beam's slab.
_KNOWN_OVERLAPS = (
    (
        "hub_housing",
        "homing_flag",
        "homing flag's c4-era Z=-100 was orphaned by c5's housing-top "
        "rise to -80 (and c7's to -58); the flag now sits inside the "
        "housing volume. Redesign queued: relocate flag/sensor pair "
        "out of the now-closed housing-pulley airspace.",
    ),
    (
        "hub_housing",
        "homing_sensor",
        "homing sensor's c4-era Z=-100 was orphaned by c5+c7 housing "
        "rise; sensor body now sits inside the housing volume. Pairs "
        "with the homing_flag overlap above; same redesign clears both.",
    ),
    (
        "kicker_lever",
        "dump_cam",
        "c7 cam rise (-64 → -44 center) lifted cam Z into the lifter-"
        "beam slab; option a' (2026-05-05) swung the beam from Y=+30 "
        "to Y=-25, moving the clipping corner from (X=-12, Y=+19) at "
        "heading ~122° to (X=-19, Y=-14) at heading ~216°. The BASE_R "
        "clip persists at the new heading. Redesign queued (notched "
        "beam or smaller cam BASE_R).",
    ),
)


def add_turret_internal_checks(a: Assembly) -> Assembly:
    """Pairwise interference checks for all turret part pairs.

    Most pairs assert NO interference; a small allow-list of known
    overlaps (`_KNOWN_OVERLAPS`) is downgraded to `assert_interference`
    with a documented reason — the assertions surface as alarms that
    fire when the underlying design fix lands and the overlap goes
    away.
    """
    known = {(a, b) for a, b, _ in _KNOWN_OVERLAPS}
    known |= {(b, a) for a, b, _ in _KNOWN_OVERLAPS}
    reasons = {(a, b): r for a, b, r in _KNOWN_OVERLAPS}
    reasons |= {(b, a): r for a, b, r in _KNOWN_OVERLAPS}
    for i in range(len(TURRET_PART_NAMES)):
        for j in range(i + 1, len(TURRET_PART_NAMES)):
            pair = (TURRET_PART_NAMES[i], TURRET_PART_NAMES[j])
            if pair in known:
                a = a.assert_interference(*pair, reason=reasons[pair])
            else:
                a = a.assert_no_interference(*pair)
    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_turret_internal_checks(add_turret(Assembly()))


if __name__ == "__main__":
    check(assembly, out="outputs")
