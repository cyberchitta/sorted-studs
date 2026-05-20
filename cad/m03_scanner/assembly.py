"""
M03 scanner — flat unit composition + within-unit checks.

Composes the integrated camera rig + turret in m03-local coordinates
(brick centroid on the IMAGE platform top face is the origin):

  * Rig parts (`arch/`, `camera_mount_esp32cam/`) placed at m03
    identity — rig-local frame matches m03, so all rig coordinates
    in `params.py` are m03-local-equivalent.
  * Turret parts placed at m03-local `Pos(0, -TURRET_ARM_R, 0)` so
    the turret hub axis lands at m03-local `(0, -ARM_R, *)` and the
    DUMP platform centroid sits at m03-local origin (option a',
    2026-05-05).

Within-unit checks at this level are the union of arch-internal +
turret-internal + arch×turret pairs. After Phase 2 (2026-05-06), the
old rig T-frame (rails A/B/C/D + the B↔C corner brace + two L-
gussets + one T-gusset) collapses into the new `arch/` (2 posts +
2 inward stubs + rail D) plus the shared back rail (`frame/`'s
`floor_back_x`, bumped +20 mm in Y). Below-* clamps now grip the
shared back rail; back-upper hangs from rail D's underside via the
plain `clamp_mount(...)` body (the wedge variant was demolished
with rail C).

Sub-assemblies (all in `cad/m03_scanner/<sub>/`):
  Rig:    arch, camera_mount_esp32cam (×4 vertices)
  Turret: frame, hub, arm, turret_platform (×4 stations), dump_cam,
          dump_kicker, drive, homing

`turret_platform/` keeps its `turret_` prefix because the bare
`platform` directory name collides with Python's stdlib `platform`
module — `from platform import assembly` would resolve to the
already-imported stdlib copy after `cad_khana`/`build123d` load.
The other turret subs (frame, hub, arm, ...) are safe to drop the
prefix on; the m05 collisions on `frame`/`drive` are handled by
`cad/assembly.py`'s per-unit module eviction.
"""

import sys
from pathlib import Path

from build123d import Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

sys.path.insert(0, str(Path(__file__).resolve().parent))

from params import (  # noqa: E402
    RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
    RAIL_PAD_CLAMP_BACK_UPPER_POS,
    RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
    RAIL_PAD_CLAMP_BELOW_LEFT_POS,
    RAIL_PAD_CLAMP_BELOW_RIGHT_CLAMP_ROT,
    RAIL_PAD_CLAMP_BELOW_RIGHT_POS,
    RAIL_PAD_CLAMP_FRONT_LOW_CLAMP_ROT,
    RAIL_PAD_CLAMP_FRONT_LOW_POS,
    TURRET_ARM_R,
    VERTEX_BACK_UPPER_POS,
    VERTEX_BELOW_LEFT_POS,
    VERTEX_BELOW_RIGHT_POS,
    VERTEX_FRONT_LOW_POS,
    horizontal_x_roll_deg,
    lens_axis_unit,
    rotation_for_normal,
)
from arch import assembly as _arch  # noqa: E402
from arm import assembly as _arm  # noqa: E402
from camera_mount_esp32cam import assembly as _camera_mount  # noqa: E402
from drive import assembly as _drive  # noqa: E402
from dump_cam import assembly as _dump_cam  # noqa: E402
from dump_kicker import assembly as _dump_kicker  # noqa: E402
from frame import assembly as _frame  # noqa: E402
from homing import assembly as _homing  # noqa: E402
from hub import assembly as _hub  # noqa: E402
from turret_platform import assembly as _turret_platform  # noqa: E402

from design_systems import DESIGN_SYSTEM, color_of  # noqa: E402
from materials import material_of  # noqa: E402


# --- Per-vertex camera specs ------------------------------------------
#
# Each instance contributes one printed body (`{prefix}_clamp_mount`)
# placed at the clamp pose plus one PCB stub (`{prefix}_pcb`) and one
# M12 lens stub (`{prefix}_lens`) placed at the vertex pose. The
# printed body's plate orientation is parameterised by `mount_in_clamp`
# so all four instances derive from one `clamp_mount(...)` function.

_CAMERA_INSTANCES = {
    "cam_front_low": {
        "clamp_pos": RAIL_PAD_CLAMP_FRONT_LOW_POS,
        "clamp_rot": RAIL_PAD_CLAMP_FRONT_LOW_CLAMP_ROT,
        "vertex_pos": VERTEX_FRONT_LOW_POS,
    },
    "cam_back_upper": {
        "clamp_pos": RAIL_PAD_CLAMP_BACK_UPPER_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
        "vertex_pos": VERTEX_BACK_UPPER_POS,
    },
    "cam_below_left": {
        "clamp_pos": RAIL_PAD_CLAMP_BELOW_LEFT_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
        "vertex_pos": VERTEX_BELOW_LEFT_POS,
    },
    "cam_below_right": {
        "clamp_pos": RAIL_PAD_CLAMP_BELOW_RIGHT_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BELOW_RIGHT_CLAMP_ROT,
        "vertex_pos": VERTEX_BELOW_RIGHT_POS,
    },
}


# --- Naming -----------------------------------------------------------

# Rig.
ARCH_NAMES = _arch.ARCH_PART_NAMES
CAMERA_PART_NAMES = tuple(
    name
    for prefix in _CAMERA_INSTANCES
    for name in (f"{prefix}_clamp_mount", f"{prefix}_pcb", f"{prefix}_lens")
)
CAMERA_RIG_PART_NAMES = list(ARCH_NAMES) + list(CAMERA_PART_NAMES)

# Turret.
TURRET_FRAME_NAMES = _frame.TURRET_FRAME_PART_NAMES
TURRET_HUB_NAMES = _hub.HUB_PART_NAMES
TURRET_ARM_NAMES = _arm.ARM_PART_NAMES
TURRET_STATIONS = ("image", "load", "idle", "dump")
TURRET_PLATFORM_NAMES = tuple(
    f"platform_{station}_{sub_name}"
    for station in TURRET_STATIONS
    for sub_name, _ in _turret_platform._IMAGE_LOCS
)
TURRET_DUMP_CAM_NAMES = (_dump_cam.DUMP_CAM_PART_NAME,)
TURRET_DUMP_KICKER_NAMES = _dump_kicker.KICKER_PART_NAMES
TURRET_DRIVE_NAMES = _drive.DRIVE_PART_NAMES
TURRET_HOMING_NAMES = _homing.HOMING_PART_NAMES

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

M03_PART_NAMES = list(CAMERA_RIG_PART_NAMES) + list(TURRET_PART_NAMES)


# --- Composition builders --------------------------------------------

# Turret placement in m03-local. With option a' (2026-05-05), DUMP
# centroid lives at turret-local (0, -ARM_R, 0); placing the unit at
# m03-local (0, -ARM_R, 0) lifts DUMP centroid to m03-local origin
# and IMAGE centroid to m03-local (0, +ARM_R, 0)... no wait — the
# turret-internal DUMP is at (0, -R), and applying base=Pos(0, -R, 0)
# puts DUMP at m03-local (0, -2R, 0). The historical placement here
# is `Pos(0, -TURRET_ARM_R, 0)` so turret-internal hub (= origin)
# lands at m03-local (0, -ARM_R, 0); the cad/assembly.py top-level
# then lifts the whole m03 unit so the rig brick centroid reaches
# world (0, +250, 1030).
TURRET_BASE = Pos(0, -TURRET_ARM_R, 0)


def add_camera_rig(a: Assembly) -> Assembly:
    """Add every camera-rig part in m03-local coordinates."""
    a = _arch.add_arch(a)
    for prefix, spec in _CAMERA_INSTANCES.items():
        clamp_loc = Pos(*spec["clamp_pos"]) * spec["clamp_rot"]
        axis = lens_axis_unit(spec["vertex_pos"])
        camera_loc = Pos(*spec["vertex_pos"]) * rotation_for_normal(
            axis, roll_deg=horizontal_x_roll_deg(axis)
        )
        mount_in_clamp = _camera_mount.mount_in_clamp_for(**spec)
        a = (
            a
            .add(
                f"{prefix}_clamp_mount",
                _camera_mount.clamp_mount(mount_in_clamp=mount_in_clamp),
                location=clamp_loc,
                material=material_of(f"{prefix}_clamp_mount"),
                color=color_of(f"{prefix}_clamp_mount"),
            )
            .add(
                f"{prefix}_pcb", _camera_mount.camera_pcb_stub(),
                location=camera_loc,
            )
            .add(
                f"{prefix}_lens", _camera_mount.m12_lens_stub(),
                location=camera_loc,
            )
        )
    return a


def add_turret(a: Assembly, base=Pos(0, 0, 0)) -> Assembly:
    """Add every turret part. Parts are placed in turret-local
    coordinates; `base` is pre-multiplied to lift the unit into the
    m03-local frame (default: `Pos(0, -TURRET_ARM_R, 0)` via
    `add_m03`).
    """
    a = _frame.add_turret_frame(a, base=base)
    a = _hub.add_turret_hub(a, base=base)
    a = _arm.add_turret_arm(a, base=base)
    for station in TURRET_STATIONS:
        a = _turret_platform.add_turret_platform(a, station, base=base)
    a = _dump_cam.add_turret_dump_cam(a, base=base)
    a = _dump_kicker.add_turret_dump_kicker(a, base=base)
    a = _drive.add_turret_drive(a, base=base)
    a = _homing.add_turret_homing(a, base=base)
    return a


def add_m03(a: Assembly) -> Assembly:
    """Add every rig + turret part in m03-local coordinates."""
    a = add_camera_rig(a)
    a = add_turret(a, base=TURRET_BASE)
    return a


# --- Internal-checks helpers -----------------------------------------
#
# Turret-internal known overlaps (kept identical to the prior
# `turret/assembly.py` `_KNOWN_OVERLAPS`). Each entry is
# (part_a, part_b, reason); the integrate loop downgrades these
# pairs to `assert_interference(reason=...)` so they self-fail when
# the underlying design fix lands and the overlap goes away.

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


def add_m03_internal_checks(a: Assembly) -> Assembly:
    """Add every within-m03 cross-boundary check.

    Three blocks:
      1. Rig-internal — every non-arch (camera mounts) vs every arch
         member.
      2. Turret-internal — every pair, with the small allow-list of
         known overlaps downgraded to `assert_interference(reason=…)`.
      3. Cross — every turret part vs every rig part.
    """
    # 1. Rig-internal.
    non_arch = [n for n in CAMERA_RIG_PART_NAMES if n not in ARCH_NAMES]
    for internal in non_arch:
        for member in ARCH_NAMES:
            a = a.assert_no_interference(internal, member)

    # 2. Turret-internal.
    known = {(p, q) for p, q, _ in _KNOWN_OVERLAPS}
    known |= {(q, p) for p, q, _ in _KNOWN_OVERLAPS}
    reasons = {(p, q): r for p, q, r in _KNOWN_OVERLAPS}
    reasons |= {(q, p): r for p, q, r in _KNOWN_OVERLAPS}
    for i in range(len(TURRET_PART_NAMES)):
        for j in range(i + 1, len(TURRET_PART_NAMES)):
            pair = (TURRET_PART_NAMES[i], TURRET_PART_NAMES[j])
            if pair in known:
                a = a.assert_interference(*pair, reason=reasons[pair])
            else:
                a = a.assert_no_interference(*pair)

    # 3. Cross — turret × rig.
    for tp in TURRET_PART_NAMES:
        for rp in CAMERA_RIG_PART_NAMES:
            a = a.assert_no_interference(tp, rp)

    return a


# --- Design-system coverage ------------------------------------------
#
# Every named m03 part must appear in the active design system's
# tagging dict (`None` allowed for hardware / optical glass / acrylic
# — those keep their material's intrinsic color). A part added without
# a tag fails here instead of silently rendering off-palette. Swapping
# `DESIGN_SYSTEM` to a palette with missing tags also fails here, so
# the swap surface is loud.

_untagged = [n for n in M03_PART_NAMES if n not in DESIGN_SYSTEM.tag_of]
assert not _untagged, (
    f"{len(_untagged)} m03 part(s) missing a tag for design system "
    f"'{DESIGN_SYSTEM.name}' in design_systems.py: {_untagged}"
)


# --- Standalone assembly ---------------------------------------------

assembly = add_m03_internal_checks(add_m03(Assembly()))


if __name__ == "__main__":
    check(assembly, out="outputs")
