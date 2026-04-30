"""
M03 turret — unit-internal composition + within-unit checks.

Standalone composition: every turret sub-assembly placed in
turret-local coordinates (hub axis at origin, +Y vertical, +Z toward
the back of the machine when placed in world). Plus pairwise
`assert_no_interference` for every turret-internal part pair.

Sub-assemblies:
  * turret_frame                  — 5-member 2020 floor frame
  * turret_hub                    — pillow-block + Ø8 shaft + 2 bearings
  * turret_arm                    — single 4-arm spider + 4 stock pins
  * turret_platform (×4 stations) — 4 cartridges, one per station
                                    (IMAGE / LOAD / DUMP / RETURN)
  * turret_dump_cam               — printed 4-lobe cam keyed to the shaft
  * turret_dump_kicker            — bench-mounted L-lever (mount + lever
                                    + pivot pin) at DUMP

The cam + kicker make up the active-push tilt mechanism: a cam lobe
arrives at heading 180° at every dwell, pushes the lever's hub-end
follower outward, and the lever's lifter pad rises against the
DUMP-station platform's tab to tilt it outer-down. At rest (cam
valley) the lifter retracts and per-arm hinge stops on the spider hold
each platform level.

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
from turret.turret_dump_cam import assembly as _turret_dump_cam  # noqa: E402
from turret.turret_dump_kicker import assembly as _turret_dump_kicker  # noqa: E402
from turret.turret_frame import assembly as _turret_frame  # noqa: E402
from turret.turret_hub import assembly as _turret_hub  # noqa: E402
from turret.turret_platform import assembly as _turret_platform  # noqa: E402


# --- Public composition builder ----------------------------------------

TURRET_FRAME_NAMES = _turret_frame.TURRET_FRAME_PART_NAMES
TURRET_HUB_NAMES = _turret_hub.HUB_PART_NAMES
TURRET_ARM_NAMES = _turret_arm.ARM_PART_NAMES
TURRET_STATIONS = ("image", "load", "dump", "return")
TURRET_PLATFORM_NAMES = tuple(
    f"platform_{station}_{sub_name}"
    for station in TURRET_STATIONS
    for sub_name, _ in _turret_platform._IMAGE_LOCS
)
TURRET_DUMP_CAM_NAMES = (_turret_dump_cam.DUMP_CAM_PART_NAME,)
TURRET_DUMP_KICKER_NAMES = _turret_dump_kicker.KICKER_PART_NAMES

TURRET_PART_NAMES = (
    list(TURRET_FRAME_NAMES)
    + list(TURRET_HUB_NAMES)
    + list(TURRET_ARM_NAMES)
    + list(TURRET_PLATFORM_NAMES)
    + list(TURRET_DUMP_CAM_NAMES)
    + list(TURRET_DUMP_KICKER_NAMES)
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
    return a


def add_turret_internal_checks(a: Assembly) -> Assembly:
    """Pairwise no-interference for all turret part pairs."""
    for i in range(len(TURRET_PART_NAMES)):
        for j in range(i + 1, len(TURRET_PART_NAMES)):
            a = a.assert_no_interference(TURRET_PART_NAMES[i], TURRET_PART_NAMES[j])
    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_turret_internal_checks(add_turret(Assembly()))


if __name__ == "__main__":
    check(assembly, out="outputs")
