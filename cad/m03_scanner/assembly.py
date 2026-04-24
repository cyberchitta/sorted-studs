"""
M03 scanner — top-level assembly.

Composes every sub-assembly in world coordinates and runs the cross-
boundary interference checks. Per-sub-system mechanism + per-part
printability live in each sub-assembly's standalone `assembly.py`;
this file only covers pairs that span two sub-systems.

Sub-assemblies placed here:

  * skeleton                    — 2020-extrusion frame (rev 2 chassis;
                                  replaces the rev-1 six-panel enclosure,
                                  which stays in the tree as standalone
                                  rev-1 reference)
  * platform                    — Ø360 collar + Ø180 optical disc + load cells
  * camera_bracket              — Pi HQ Camera + 2.8 mm lens + printed plate
  * camera_bracket_rail_clamp   — rev-2 rail clamp supporting the camera
                                  bracket off `rail_front_mid`. Replaces
                                  the rev-1 `camera_bracket_adapter/`
                                  wedge (kept in the tree as standalone
                                  rev-1 reference).
  * mirror_mount (×3)           — rigid rev 1 mount at each mirror pose
                                  (back-upper, below-left, below-right)
  * mirror_mount_rail_clamp     — rev-2 rail clamp supporting each mirror
                                  mount off a skeleton rail; three
                                  instances (below-left on
                                  `rail_floor_left`, below-right on
                                  `rail_floor_right`, back-upper on
                                  `rail_ceil_back`). Replaces the rev-1
                                  `below_mirror_floor_bracket/` and
                                  `back_upper_wall_bracket/` wedges (both
                                  kept in the tree as standalone rev-1
                                  reference).
  * load_cell_standoff (×3)     — printed column carrying each load cell's
                                  fixed end; rev-1 bottom face sat on the
                                  enclosure floor, now floats until the
                                  platform's rev-2 hinged-quad replacement
                                  lands (see NEXT_STEPS.md)

Placement references come from `params.py`; those values are copied
from `docs/m03-multi-view-scanner.md`. If the reference doc's
parameter table changes, update `params.py` to match — never inline
mirror/camera coordinates in this file.

Cross-boundary checks:

  * Every internal part vs every skeleton member. Guards against a
    rev bump in any sub-assembly pushing a part into the frame.
    Replaces the rev-1 "every internal part vs every enclosure
    panel" loop.
  * Every mirror mount vs the camera bracket (mirrors sit at ≥ 250 mm
    from the camera; assertion is a regression guard against future
    mount growth into the direct-viewpoint cone).
  * Every mirror mount vs the platform (below-mirrors sit at
    y = -140 mm, ≥ 120 mm below the platform collar; assertion guards
    against future mirror-support brackets climbing up into the
    platform / load-cell envelope).

Mirror-to-mirror interference is not asserted: the three mirrors are
fixed by the tetrahedral optimiser in `klods-syn/nbs/200_viewpoints.md`
and their plate extents don't approach each other (closest centre pair
is ≈ 460 mm apart). A check would pass trivially and add noise.

Transitional state (rev 2 mid-migration — see NEXT_STEPS.md):
  The load-cell standoffs were each designed in rev 1 to sit tangent to
  a specific enclosure panel that is no longer placed. They remain in
  the tree at their original world poses — geometrically correct, but
  physically "floating" until each gets its own rev-2 rail-clamp /
  skeleton attachment (the platform's rev-2 hinged-quad replacement
  drops the standoffs altogether; see NEXT_STEPS.md). Cross-boundary
  assertions against the former enclosure panels are gone; assertions
  against the skeleton take their place.
"""

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

from build123d import Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

# `params.py` sits next to this file, so a plain module-level import
# works once this directory is on sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from params import (  # noqa: E402
    CAMERA_POS,
    CAMERA_ROT_DEG,
    LOAD_CELL_STANDOFF_HEADING_DEG,
    LOAD_CELL_STANDOFF_POSITIONS,
    MIRROR_BACK_UPPER_NORMAL,
    MIRROR_BACK_UPPER_POS,
    MIRROR_BELOW_LEFT_NORMAL,
    MIRROR_BELOW_LEFT_POS,
    MIRROR_BELOW_RIGHT_NORMAL,
    MIRROR_BELOW_RIGHT_POS,
    RAIL_CLAMP_BACK_UPPER_CLAMP_ROT,
    RAIL_CLAMP_BACK_UPPER_POS,
    RAIL_CLAMP_BELOW_LEFT_CLAMP_ROT,
    RAIL_CLAMP_BELOW_LEFT_POS,
    RAIL_CLAMP_BELOW_RIGHT_CLAMP_ROT,
    RAIL_CLAMP_BELOW_RIGHT_POS,
    RAIL_CLAMP_CAMERA_CLAMP_ROT,
    RAIL_CLAMP_CAMERA_POS,
    horizontal_x_roll_deg,
    rotation_for_normal,
)


# --- Sub-assembly loading -----------------------------------------------
#
# Each sub-directory holds a bare `assembly.py` with no `__init__.py`.
# Three of the four names (`platform` in particular) collide with stdlib
# modules, so the namespace-package trick used in
# `cad/m05_diverter/assembly.py` doesn't survive. Load each by file
# path via importlib.util — same pattern for all four keeps the
# top-level uniform.

def _load(subdir: str) -> ModuleType:
    path = Path(__file__).resolve().parent / subdir / "assembly.py"
    spec = importlib.util.spec_from_file_location(
        f"m03_scanner_{subdir}", path
    )
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


_skel = _load("skeleton")
_plat = _load("platform")
_cam = _load("camera_bracket")
_mirror = _load("mirror_mount")
_standoff = _load("load_cell_standoff")
_rail_clamp = _load("mirror_mount_rail_clamp")
_cam_clamp = _load("camera_bracket_rail_clamp")


# --- Placement constants ------------------------------------------------

# The platform's local +Z points out of the top face; the scanner's
# world "up" is +Y. Rot(-90, 0, 0) maps local +Z onto world +Y and
# keeps the local origin at the world origin. See platform/assembly.py.
_PLATFORM_LOC = Rot(-90, 0, 0)

# Camera placement: local +Z (lens axis) rotated to 25° below horizontal
# toward +Z; origin translated to the PCB rear plane in world.
_CAMERA_LOC = Pos(*CAMERA_POS) * Rot(*CAMERA_ROT_DEG)

# Rev-2 camera rail clamp placement. Clamp-local +X = along
# `rail_front_mid` long axis (world +X); +Y = away from rail (world
# +Z, from front wall toward brick); +Z = world -Y (right-handed
# completion). Rx(+90°) carries clamp-local axes onto these world
# directions; the clamp origin sits on the rail's +Z (inboard) face at
# the bracket back face Y.
_CAM_CLAMP_LOC = Pos(*RAIL_CLAMP_CAMERA_POS) * RAIL_CLAMP_CAMERA_CLAMP_ROT

# Mirror placements: local +Z = mirror outward normal. Translation
# places the reflective face centre at the mirror's world coordinates.
# Roll about the outward normal is picked by `horizontal_x_roll_deg`
# so mount-local +X lands on a world-horizontal direction — matching
# each below-mirror's floor-bracket local +X axis, which is how the
# mount's four-corner bolt pattern stays axis-aligned to the
# `below_mirror_floor_bracket` top-face bolt pattern. Back-upper has
# no downstream bracket yet; the same convention keeps its roll
# well-defined for when that bracket lands.
_MIRROR_POSES = {
    "mirror_back_upper": (
        Pos(*MIRROR_BACK_UPPER_POS)
        * rotation_for_normal(
            MIRROR_BACK_UPPER_NORMAL,
            roll_deg=horizontal_x_roll_deg(MIRROR_BACK_UPPER_NORMAL),
        )
    ),
    "mirror_below_left": (
        Pos(*MIRROR_BELOW_LEFT_POS)
        * rotation_for_normal(
            MIRROR_BELOW_LEFT_NORMAL,
            roll_deg=horizontal_x_roll_deg(MIRROR_BELOW_LEFT_NORMAL),
        )
    ),
    "mirror_below_right": (
        Pos(*MIRROR_BELOW_RIGHT_POS)
        * rotation_for_normal(
            MIRROR_BELOW_RIGHT_NORMAL,
            roll_deg=horizontal_x_roll_deg(MIRROR_BELOW_RIGHT_NORMAL),
        )
    ),
}

# Rev-2 rail clamp placements (one per mirror). Clamp-local +X = along
# the supporting rail long axis; +Y = away from rail (clamp body
# rises in +Y); +Z = from rail toward mount.
#   * below-left  — `rail_floor_left`, Ry(+90°): local +X → world -Z,
#                   +Y → world +Y, +Z → world +X.
#   * below-right — X-mirror of below-left: `rail_floor_right`,
#                   Ry(-90°): +X → world +Z, +Z → world -X.
#   * back-upper  — `rail_ceil_back`, Rx(180°): grips rail's -Y face
#                   so +Y → world -Y (clamp hangs below rail) and
#                   +Z → world -Z (mount is inboard of rail). Pure Ry
#                   heading can't express this; specs carry a full
#                   `Rot` (`clamp_rot`) rather than a heading float.
# Clamp-local geometry is identical for the two below mirrors by
# mirror symmetry; back-upper has a longer mount-back reach so its
# beam tilt + length differ (derived inside `clamp()` from
# `mount_in_clamp`).
_RAIL_CLAMP_SPECS = {
    "rail_clamp_below_left": {
        "clamp_pos": RAIL_CLAMP_BELOW_LEFT_POS,
        "clamp_rot": RAIL_CLAMP_BELOW_LEFT_CLAMP_ROT,
        "mirror_pos": MIRROR_BELOW_LEFT_POS,
        "mirror_normal": MIRROR_BELOW_LEFT_NORMAL,
    },
    "rail_clamp_below_right": {
        "clamp_pos": RAIL_CLAMP_BELOW_RIGHT_POS,
        "clamp_rot": RAIL_CLAMP_BELOW_RIGHT_CLAMP_ROT,
        "mirror_pos": MIRROR_BELOW_RIGHT_POS,
        "mirror_normal": MIRROR_BELOW_RIGHT_NORMAL,
    },
    "rail_clamp_back_upper": {
        "clamp_pos": RAIL_CLAMP_BACK_UPPER_POS,
        "clamp_rot": RAIL_CLAMP_BACK_UPPER_CLAMP_ROT,
        "mirror_pos": MIRROR_BACK_UPPER_POS,
        "mirror_normal": MIRROR_BACK_UPPER_NORMAL,
    },
}

# Load-cell standoff placements. Local +Y = world +Y; local +X aligns
# with the supported beam's radial outward direction under Ry(heading),
# where heading = the load-cell's platform-local azimuth (derivation
# in params.py). One placement per load cell, in the same index order
# as the platform's `load_cell_{i}` parts.
_STANDOFF_LOCS = [
    Pos(*pos) * Rot(0, heading, 0)
    for pos, heading in zip(
        LOAD_CELL_STANDOFF_POSITIONS,
        LOAD_CELL_STANDOFF_HEADING_DEG,
    )
]


# --- Assembly composition -----------------------------------------------

assembly = Assembly()

# Skeleton frame members. Each member in `_skel._MEMBERS` is already
# positioned at its world location (the skeleton sub-assembly bakes
# the `Pos(...)` into each part). The names follow the skeleton's
# convention: `post_{fl,fr,bl,br}`, `rail_{floor,ceil}_{front,back,left,right}`.
_SKELETON_NAMES = tuple(_skel._MEMBERS.keys())
for _name, _part in _skel._MEMBERS.items():
    assembly = assembly.add(_name, _part)

# Platform (collar + disc + three load cells). Sibling sub-assembly
# names aren't prefixed since these are the only collar / disc / load
# cells in the whole rig.
assembly = (
    assembly
    .add("collar", _plat.collar(), location=_PLATFORM_LOC)
    .add("disc", _plat.disc_stub(), location=_PLATFORM_LOC)
)
_LOAD_CELL_NAMES = tuple(
    f"load_cell_{i}" for i in range(len(_plat.LOAD_CELL_AZ_DEG))
)
for _idx, _az in enumerate(_plat.LOAD_CELL_AZ_DEG):
    # _load_cell_location returns the beam pose in the platform's local
    # frame; compose _PLATFORM_LOC on the outside to lift into world.
    assembly = assembly.add(
        _LOAD_CELL_NAMES[_idx],
        _plat.load_cell_stub(),
        location=_PLATFORM_LOC * _plat._load_cell_location(_az),
    )

# Camera bracket + PCB + lens stand-in.
assembly = (
    assembly
    .add("camera_plate", _cam.bracket(), location=_CAMERA_LOC)
    .add("camera_pcb", _cam.camera_pcb_stub(), location=_CAMERA_LOC)
    .add("camera_lens", _cam.lens_stub(), location=_CAMERA_LOC)
)

# Rev-2 camera rail clamp. The clamp's mount_in_clamp is computed from
# the camera bracket's world placement so the pad lands coplanar with
# the bracket back face.
_cam_clamp_mount_in_clamp = _cam_clamp.mount_in_clamp_for(
    clamp_pos=RAIL_CLAMP_CAMERA_POS,
    clamp_rot=RAIL_CLAMP_CAMERA_CLAMP_ROT,
    camera_pos=CAMERA_POS,
    camera_rot_deg=CAMERA_ROT_DEG,
)
assembly = assembly.add(
    "rail_clamp_camera",
    _cam_clamp.clamp(mount_in_clamp=_cam_clamp_mount_in_clamp),
    location=_CAM_CLAMP_LOC,
)

# Three rigid mirror mounts. Each contributes a bracket + mirror stand-in
# under a unique prefix.
_MIRROR_PART_NAMES = []
for _prefix, _loc in _MIRROR_POSES.items():
    _bracket_name = f"{_prefix}_bracket"
    _mirror_name = f"{_prefix}_glass"
    _MIRROR_PART_NAMES.extend([_bracket_name, _mirror_name])
    assembly = (
        assembly
        .add(_bracket_name, _mirror.bracket(), location=_loc)
        .add(_mirror_name, _mirror.mirror_stub(), location=_loc)
    )

# Rev-2 rail clamps (one per mirror). Each instance passes its own
# mount_in_clamp Location built from the per-instance placement +
# mirror constants above.
_RAIL_CLAMP_NAMES = tuple(_RAIL_CLAMP_SPECS.keys())
for _name, _spec in _RAIL_CLAMP_SPECS.items():
    _loc = Pos(*_spec["clamp_pos"]) * _spec["clamp_rot"]
    _mount_in_clamp = _rail_clamp.mount_in_clamp_for(**_spec)
    assembly = assembly.add(
        _name,
        _rail_clamp.clamp(mount_in_clamp=_mount_in_clamp),
        location=_loc,
    )

# Three load-cell standoffs (one printed column under each load cell's
# fixed end). Names mirror the platform's `load_cell_{i}` indices so
# cross-boundary assertions pair cleanly.
_STANDOFF_NAMES = tuple(
    f"load_cell_standoff_{i}" for i in range(len(_STANDOFF_LOCS))
)
for _name, _loc in zip(_STANDOFF_NAMES, _STANDOFF_LOCS):
    assembly = assembly.add(_name, _standoff.bracket(), location=_loc)


# --- Cross-boundary assertions ------------------------------------------

# Internal parts that must clear every skeleton member.
_PLATFORM_NAMES = ["collar", "disc", *_LOAD_CELL_NAMES]
_CAMERA_BRACKET_NAMES = ("camera_plate", "camera_pcb", "camera_lens")
_INTERNAL_NAMES = (
    _PLATFORM_NAMES
    + list(_CAMERA_BRACKET_NAMES)
    + ["rail_clamp_camera"]
    + _MIRROR_PART_NAMES
    + list(_STANDOFF_NAMES)
    + list(_RAIL_CLAMP_NAMES)
)

for _internal in _INTERNAL_NAMES:
    for _member in _SKELETON_NAMES:
        assembly = assembly.assert_no_interference(_internal, _member)

# Mirror mounts vs camera-bracket triplet. Regression guard — the
# mirrors are well clear today, but any future growth in the kinematic
# mount stack or in the lens envelope could close the gap.
for _mirror_part in _MIRROR_PART_NAMES:
    for _cam_part in _CAMERA_BRACKET_NAMES:
        assembly = assembly.assert_no_interference(_mirror_part, _cam_part)

# Mirror mounts vs platform. Same logic — below-mirrors currently sit
# ≥ 120 mm below the platform collar, but a future floor-bracket rev
# could extend the mount up toward the platform aperture.
for _mirror_part in _MIRROR_PART_NAMES:
    for _plat_part in _PLATFORM_NAMES:
        assembly = assembly.assert_no_interference(_mirror_part, _plat_part)

# Camera rail clamp vs camera bracket triplet. Pad face is coplanar
# with the bracket back face minus MOUNT_GAP (tangent treatment, same
# convention as the mirror rail clamps); this assertion enforces
# tangency-without-overlap. Also catches drift in CAMERA_POS,
# CAMERA_ROT_DEG, CAMERA_BRACKET_PLATE_T, or any of the camera rail-
# clamp placement constants.
for _cam_part in _CAMERA_BRACKET_NAMES:
    assembly = assembly.assert_no_interference("rail_clamp_camera", _cam_part)

# Camera rail clamp vs every mirror mount. Camera clamp sits at world
# x ≈ 0, y ∈ [98, 177], z ∈ [-280, -231]; mirrors are well clear
# today (below at y ≈ -140, back-upper at y ≈ 180 but z ≈ 180).
# Regression guard against future clamp or mirror-mount growth.
for _mirror_part in _MIRROR_PART_NAMES:
    assembly = assembly.assert_no_interference("rail_clamp_camera", _mirror_part)

# Camera rail clamp vs platform. Camera clamp sits forward of the
# platform (z ≤ -231); regression guard for any future shuttle or
# disc-retainer feature that grows into the front hemisphere.
for _plat_part in _PLATFORM_NAMES:
    assembly = assembly.assert_no_interference("rail_clamp_camera", _plat_part)

# Mirror rail clamps vs everything else. Each clamp's pad face is
# coplanar with its supported mirror mount plate back face minus
# MOUNT_GAP (tangent treatment as the rev-1 floor bracket's top-face
# convention); the clamp's bottom face is coplanar with its supporting
# skeleton rail, already covered by the skeleton assertions in the
# _INTERNAL_NAMES loop.
for _clamp_name in _RAIL_CLAMP_NAMES:
    # vs every mirror-mount part (the supported mirror is the target
    # contact; the other two are regression guards).
    for _mirror_part in _MIRROR_PART_NAMES:
        assembly = assembly.assert_no_interference(_clamp_name, _mirror_part)

    # vs platform parts — below-mirror clamps at y ∈ [-220, -85],
    # back-upper clamp at y ∈ [125, 320]; platform at y ≈ 0; all
    # regression guards.
    for _plat_part in _PLATFORM_NAMES:
        assembly = assembly.assert_no_interference(_clamp_name, _plat_part)

    # vs camera bracket triplet + camera rail clamp — below mirror
    # clamps at world |x| ≥ 205, z ∈ [-50, 100]; back-upper clamp at
    # x = 0, z ∈ [130, 300]; camera at x ≈ 0, z ≈ -226; camera clamp
    # at x ∈ [-30, 30], z ∈ [-280, -231]. Well clear.
    for _cam_part in _CAMERA_BRACKET_NAMES:
        assembly = assembly.assert_no_interference(_clamp_name, _cam_part)
    assembly = assembly.assert_no_interference(_clamp_name, "rail_clamp_camera")

# Mirror rail clamp vs mirror rail clamp. Below-left/right sit in
# opposite X hemispheres; back-upper sits on the back ceiling rail
# with no X overlap with the below-mirror clamps at today's envelopes.
# Regression guard against any future clamp footprint expansion.
for _i in range(len(_RAIL_CLAMP_NAMES)):
    for _j in range(_i + 1, len(_RAIL_CLAMP_NAMES)):
        assembly = assembly.assert_no_interference(
            _RAIL_CLAMP_NAMES[_i], _RAIL_CLAMP_NAMES[_j]
        )


# Load-cell standoffs vs. everything else. Each standoff's top face sits
# MOUNT_GAP below its own load-cell's bottom face (coplanar-tangent
# treatment matches the mirror brackets' mount-gap convention); the
# bottom face is coplanar with the enclosure floor, already covered by
# the _INTERNAL_NAMES loop.
#
# The az=210 and az=330 standoffs sit ~2 mm outside the `.below_left` /
# `.below_right` floor-bracket bbox +X edges (see the
# load_cell_standoff/ module docstring). The assertions below enforce
# that margin as a regression guard.
for _idx, _standoff_name in enumerate(_STANDOFF_NAMES):
    # vs. every skeleton member — already added in the _INTERNAL_NAMES
    # loop above.

    # vs. the supported load cell (MOUNT_GAP = 0.3 mm keeps the tangent
    # contact clean) plus the other two as regression guards.
    for _plat_part in _PLATFORM_NAMES:
        assembly = assembly.assert_no_interference(_standoff_name, _plat_part)

    # vs. every mirror-mount part (bracket + glass).
    for _mirror_part in _MIRROR_PART_NAMES:
        assembly = assembly.assert_no_interference(_standoff_name, _mirror_part)

    # vs. the camera bracket triplet + camera rail clamp (the az=90
    # standoff sits at world z ≈ -220, the camera clamp at z ∈ [-280,
    # -231] and the bracket at z ≈ -225 — standoff is a vertical column
    # at y ∈ [-240, -48], so it's entirely below the camera clamp's y
    # ∈ [98, 177]. Regression guard).
    for _cam_part in _CAMERA_BRACKET_NAMES:
        assembly = assembly.assert_no_interference(_standoff_name, _cam_part)
    assembly = assembly.assert_no_interference(_standoff_name, "rail_clamp_camera")

    # vs. every rev-2 mirror rail clamp. The az=210 standoff sits closest
    # to the below-left clamp (~2 mm outside the clamp's +X bbox edge per
    # the load_cell_standoff/ docstring); az=330 sits symmetrically
    # close to the below-right clamp; back-upper clamp hangs at y ≥
    # 125, comfortably above all three standoffs. Regression guard.
    for _clamp_name in _RAIL_CLAMP_NAMES:
        assembly = assembly.assert_no_interference(_standoff_name, _clamp_name)

# Standoff vs. standoff — the three columns sit at r = 220 around the
# platform with 120° angular separation, so centre-to-centre is
# 2·220·sin(60°) ≈ 381 mm. Regression guard against any future footprint
# expansion.
for _i in range(len(_STANDOFF_NAMES)):
    for _j in range(_i + 1, len(_STANDOFF_NAMES)):
        assembly = assembly.assert_no_interference(
            _STANDOFF_NAMES[_i], _STANDOFF_NAMES[_j]
        )


if __name__ == "__main__":
    check(assembly, out="outputs")
