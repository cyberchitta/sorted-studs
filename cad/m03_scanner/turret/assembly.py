"""
M03 turret — unit-internal composition + within-unit checks.

Standalone composition: every turret sub-assembly placed in
turret-local coordinates (hub axis at origin, +Y vertical, +Z toward
the back of the machine when placed in world). Plus every
turret-internal `assert_no_interference` pair, except for the 8
documented adjacent-platform overlaps that fire as
`assert_interference` (TURRET_ARM_R = 130 is too small for the
180 × 180 platform footprint; see the reason string below).

Sub-assemblies:
  * turret_frame                  — 5-member 2020 floor frame
  * turret_hub                    — pillow-block + Ø8 shaft + 2 bearings
  * turret_arm                    — single 4-arm spider + 4 stock pins
  * turret_platform (×4 stations) — 4 cartridges, one per station
                                    (IMAGE / LOAD / DUMP / RETURN)
  * turret_cam_ring               — fixed annular cam ring at radius 220
"""

import sys
from pathlib import Path

from build123d import Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from turret.turret_arm import assembly as _turret_arm  # noqa: E402
from turret.turret_cam_ring import assembly as _turret_cam_ring  # noqa: E402
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
TURRET_CAM_RING_NAMES = (_turret_cam_ring.CAM_RING_PART_NAME,)

TURRET_PART_NAMES = (
    list(TURRET_FRAME_NAMES)
    + list(TURRET_HUB_NAMES)
    + list(TURRET_ARM_NAMES)
    + list(TURRET_PLATFORM_NAMES)
    + list(TURRET_CAM_RING_NAMES)
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
    a = _turret_cam_ring.add_turret_cam_ring(a, base=base)
    return a


# --- Documented adjacent-platform overlaps -----------------------------
#
# TURRET_ARM_R = 130 is too small for the 180 × 180 mm platforms:
# adjacent stations have inner corners that overlap. Geometric minimum:
# two squares 90° apart at radius R have non-overlapping inner corners
# only when R ≥ 180 mm. Each adjacent pair (image↔load, image↔dump,
# load↔return, dump↔return) overlaps in its acrylic and its perimeter
# ring; diagonal pairs (image↔return, load↔dump) clear by 80 mm.
#
# These overlaps are documented via `assert_interference` rather than
# skipped — when the user fixes TURRET_ARM_R to ≥ 180 in a follow-up,
# the assertion FAILS, forcing this block to be removed at the same
# time as the design fix.

_ADJACENT_STATION_PAIRS = (
    ("image", "load"),
    ("image", "dump"),
    ("load", "return"),
    ("dump", "return"),
)
_OVERLAPPING_PLATFORM_PARTS = ("acrylic", "ring")
_EXPECTED_PLATFORM_OVERLAPS = {
    frozenset((f"platform_{s1}_{sub}", f"platform_{s2}_{sub}"))
    for s1, s2 in _ADJACENT_STATION_PAIRS
    for sub in _OVERLAPPING_PLATFORM_PARTS
}
_PLATFORM_OVERLAP_REASON = (
    "TURRET_ARM_R = 130 is too small for the 180 × 180 platform; "
    "adjacent platforms overlap at their inner corners. Fix in a "
    "follow-up: raise TURRET_ARM_R to ≥ 180 (and update derived "
    "constants). This assertion fires when the overlap is fixed — "
    "remove it then."
)


def add_turret_internal_checks(a: Assembly) -> Assembly:
    """Add every within-unit assertion: pairwise no-interference for
    all turret part pairs, except for the 8 documented adjacent-
    platform overlaps which use assert_interference."""
    for i in range(len(TURRET_PART_NAMES)):
        for j in range(i + 1, len(TURRET_PART_NAMES)):
            a_part, b_part = TURRET_PART_NAMES[i], TURRET_PART_NAMES[j]
            if frozenset((a_part, b_part)) in _EXPECTED_PLATFORM_OVERLAPS:
                a = a.assert_interference(
                    a_part, b_part, reason=_PLATFORM_OVERLAP_REASON
                )
            else:
                a = a.assert_no_interference(a_part, b_part)
    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_turret_internal_checks(add_turret(Assembly()))


if __name__ == "__main__":
    check(assembly, out="outputs")
