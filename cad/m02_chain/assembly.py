"""
m02 chain — top-level standalone assembly.

Composes the chain's structural skeleton — two column extensions
(`col1_ext`, `col2_ext`) rising from the m05 top deck plane, and an
S1 yoke crossbar (`s1__yoke`) spanning between them at the S1 m02
V-bottom plane — plus the LOAD ramp (`s1__load_ramp`) and the
spliced S1 m02 singulator (parts prefixed `s1__`). Coordinate frame,
anchors, and the deferred stator-mounting interface are documented
in `params.py`.

Cross-boundary checks (within the chain):

- yoke ↔ m02 rotor / motor / stem — the yoke threads underneath
  the V-bottom plane and inside the stator's inner footprint;
  every chain skeleton part must clear every m02 internal part.
- col1 / col2 ↔ m02 stator / rotor / motor / stem — the columns
  end at the V-bottom plane; tangent contact at z=YOKE_TOP_Z_LOCAL
  is allowed but no part may overlap volumetrically.
- LOAD ramp ↔ m02 stator — the ramp inlet sits in the chute-exit
  azimuth (chain -X side of S1); the ramp must NOT crash the
  outer stator wall outboard of the chute opening.

m05 column tops + top deck and m03 turret parts are EXTERNAL to the
chain — cross-unit asserts live in `cad/assembly.py`.

The m02 stator-mounting interface (handoff sub-task 2) is DEFERRED;
the stator sits at its geometric S1 position with no plate / clamp /
cradle modeled. See `params.py` module docstring.

Composition strategy mirrors `cad/assembly.py`:

The m02 unit's `assembly.py` imports its own sub-modules via bare
names (`from rotor import ...`) after a `sys.path.insert`, which
collide with the chain's identically-named modules. We load it via
`runpy.run_path` with the colliding cached modules evicted before
and after, snapshot/restore `sys.path`, then re-add every part of
the m02 assembly into the chain with a chain-local pose and `s1__`
prefix.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

from build123d import Box, Color, Location, Part, Pos, Rot

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

from design_systems import DESIGN_SYSTEM, color_of
from materials import material_of
from params import (
    CHAIN_WORLD_POSE_XYZ,
    COL1_LOCAL_XY,
    COL1_ROT_DEG,
    COL2_LOCAL_XY,
    COL_LEN,
    COL_Z_BOT_LOCAL,
    COL_Z_TOP_LOCAL,
    EXT_SIZE,
    M03_LOAD_TOP_Z,
    M03_TURRET_ARM_R,
    M05_TOP_DECK_T,
    M05_TOP_VERT_Z,
    RAMP_FLOOR_T,
    RAMP_OUTLET_LOCAL,
    RAMP_PITCH_DEG,
    RAMP_SLOPE_LEN,
    RAMP_WALL_H,
    RAMP_WALL_T,
    RAMP_WIDTH,
    S1_AXIS_LOCAL_XY,
    S1_M02_HEADING_DEG,
    S1_VBOT_Z_LOCAL,
    YOKE_ANGLE_DEG,
    YOKE_LEN,
    YOKE_MID_XY,
    YOKE_TOP_Z_LOCAL,
)

# Re-exports the top-level `cad/assembly.py` reads from this module's
# `runpy` namespace to verify mirrored anchors and place the chain in
# world. Keep in sync with the cross-checks in `cad/assembly.py`.
__all__ = (
    "assembly",
    "CHAIN_WORLD_POSE_XYZ",
    "M05_TOP_VERT_Z",
    "M05_TOP_DECK_T",
    "M03_TURRET_ARM_R",
    "M03_LOAD_TOP_Z",
)


THIS_DIR = Path(__file__).resolve().parent


# --- Part builders -----------------------------------------------------

def _column_ext() -> Part:
    """20×20×COL_LEN 2020 extrusion, centered at origin, axis +Z."""
    return Box(EXT_SIZE, EXT_SIZE, COL_LEN)


def _yoke_crossbar() -> Part:
    """20×20×YOKE_LEN 2020 extrusion, centered at origin, axis +X
    in part-local. Placement rotates and translates to the chain-local
    yoke position."""
    return Box(YOKE_LEN, EXT_SIZE, EXT_SIZE)


def _load_ramp() -> Part:
    """Open chute — flat floor plate + two side walls. Built in a
    ramp-local frame with origin at the OUTLET centerline on the floor
    TOP surface, +X pointing up the slope toward the inlet, +Y across
    the ramp's width, +Z normal to the floor (out of the brick contact
    surface). Placement rotates by -RAMP_PITCH_DEG about Y (tilting
    +X up into +Z) and translates the outlet origin to chain-local
    RAMP_OUTLET_LOCAL.
    """
    # Floor — top face at z=0, body below.
    floor = Box(RAMP_SLOPE_LEN, RAMP_WIDTH, RAMP_FLOOR_T)
    floor = Pos(RAMP_SLOPE_LEN / 2.0, 0.0, -RAMP_FLOOR_T / 2.0) * floor

    # Side walls — built off the floor's +Y / -Y edges, rising in +Z.
    wall = Box(RAMP_SLOPE_LEN, RAMP_WALL_T, RAMP_WALL_H)
    wall_pos = (
        Pos(
            RAMP_SLOPE_LEN / 2.0,
            (RAMP_WIDTH - RAMP_WALL_T) / 2.0,
            RAMP_WALL_H / 2.0,
        )
        * wall
    )
    wall_neg = (
        Pos(
            RAMP_SLOPE_LEN / 2.0,
            -(RAMP_WIDTH - RAMP_WALL_T) / 2.0,
            RAMP_WALL_H / 2.0,
        )
        * wall
    )
    return floor + wall_pos + wall_neg


# --- m02 sub-assembly loader (runpy + module eviction) ----------------

# Names that the m02 standalone `assembly.py` (and its sub-tree) imports
# via the bare `from <name> import …` pattern after a `sys.path.insert`.
# These collide with the chain's own `params` / `design_systems` /
# `materials` / etc., so we evict them before and after the runpy load.
_M02_MODULE_LOCAL_HEADS = frozenset(
    {
        "params",
        "design_systems",
        "materials",
        "detail_variations",
        "material_variations",
        "rotor",
        "stator",
        "drive",
    }
)


def _evict_m02_module_locals() -> None:
    for name in list(sys.modules):
        head = name.split(".", 1)[0]
        if head in _M02_MODULE_LOCAL_HEADS:
            del sys.modules[name]


def _load_m02_namespace() -> dict:
    """Run m02's `assembly.py` in isolation and return its module
    namespace. The m02 dir sits as a sibling under `cad/`."""
    saved_path = sys.path.copy()
    _evict_m02_module_locals()
    try:
        m02_dir = THIS_DIR.parent / "m02_c_channel_singulator"
        return runpy.run_path(
            str(m02_dir / "assembly.py"),
            run_name="<m02_c_channel_singulator>",
        )
    finally:
        sys.path[:] = saved_path
        _evict_m02_module_locals()


_M02_NS = _load_m02_namespace()
_M02_ASSY: Assembly = _M02_NS["assembly"]


# --- Splice helper -----------------------------------------------------

def _splice_m02(
    top: Assembly,
    mod: Assembly,
    base: Location,
    prefix: str,
) -> Assembly:
    """Re-add every placed part from ``mod`` into ``top`` with its
    location pre-multiplied by ``base``, name prefixed, and color /
    material re-resolved through the chain's design system. Within-
    unit assertions are dropped — the m02 standalone `khana check`
    validates them.
    """
    for p in mod.parts:
        full_name = f"{prefix}{p.name}"
        top = top.with_part(
            full_name,
            p.part,
            location=base * p.location,
            color=color_of(full_name),
            material=material_of(full_name),
        )
    return top


# --- Design-system coverage --------------------------------------------

# Every named chain part (chain-own + spliced m02) must appear in the
# active design system. Mirrors the m02 / m05 coverage check pattern.

_S1_PREFIX = "s1__"
_CHAIN_OWN_NAMES = ("col1_ext", "col2_ext", f"{_S1_PREFIX}yoke", f"{_S1_PREFIX}load_ramp")
_M02_PART_NAMES_LOCAL = tuple(p.name for p in _M02_ASSY.parts)
_S1_M02_NAMES = tuple(f"{_S1_PREFIX}{n}" for n in _M02_PART_NAMES_LOCAL)
_ALL_CHAIN_NAMES = _CHAIN_OWN_NAMES + _S1_M02_NAMES

_untagged = [n for n in _ALL_CHAIN_NAMES if n not in DESIGN_SYSTEM.tag_of]
assert not _untagged, (
    f"{len(_untagged)} chain part(s) missing a tag for design system "
    f"'{DESIGN_SYSTEM.name}' in design_systems.py: {_untagged}"
)


# --- Compose -----------------------------------------------------------

def _kw(name: str) -> dict:
    """Color + material kwargs for the active design system."""
    return {"color": color_of(name), "material": material_of(name)}


# Column placements. col1 rotated 30° about Z to match m05
# `outer_vert_0`'s orientation (one face aligned with the bracket
# plate convention). col2 rotated to the yoke angle so its faces
# align with the yoke crossbar end.
_COL1_LOC = (
    Pos(COL1_LOCAL_XY[0], COL1_LOCAL_XY[1], (COL_Z_BOT_LOCAL + COL_Z_TOP_LOCAL) / 2.0)
    * Rot(0, 0, COL1_ROT_DEG)
)
_COL2_LOC = (
    Pos(COL2_LOCAL_XY[0], COL2_LOCAL_XY[1], (COL_Z_BOT_LOCAL + COL_Z_TOP_LOCAL) / 2.0)
    * Rot(0, 0, YOKE_ANGLE_DEG)
)

# Yoke placement. Box's long axis is +X (length YOKE_LEN). Rotate by
# YOKE_ANGLE_DEG about Z, translate to YOKE_MID_XY at z =
# YOKE_TOP_Z_LOCAL - EXT_SIZE/2 (top face at YOKE_TOP_Z_LOCAL).
_YOKE_LOC = (
    Pos(YOKE_MID_XY[0], YOKE_MID_XY[1], YOKE_TOP_Z_LOCAL - EXT_SIZE / 2.0)
    * Rot(0, 0, YOKE_ANGLE_DEG)
)

# LOAD ramp placement. Tilt about ramp-local Y by -RAMP_PITCH_DEG so
# +X tilts up into +Z, then translate to chain-local outlet.
_RAMP_LOC = (
    Pos(*RAMP_OUTLET_LOCAL)
    * Rot(0, -RAMP_PITCH_DEG, 0)
)

# m02 splice base. Translate to S1 axis at the V-bottom plane, then
# rotate by S1_M02_HEADING_DEG about Z so the chute fires in chain-local
# -X (toward LOAD).
_M02_SPLICE_BASE = (
    Pos(S1_AXIS_LOCAL_XY[0], S1_AXIS_LOCAL_XY[1], S1_VBOT_Z_LOCAL)
    * Rot(0, 0, S1_M02_HEADING_DEG)
)


assembly = (
    Assembly()
    .with_part("col1_ext", _column_ext(), location=_COL1_LOC, **_kw("col1_ext"))
    .with_part("col2_ext", _column_ext(), location=_COL2_LOC, **_kw("col2_ext"))
    .with_part(
        f"{_S1_PREFIX}yoke", _yoke_crossbar(),
        location=_YOKE_LOC, **_kw(f"{_S1_PREFIX}yoke"),
    )
    .with_part(
        f"{_S1_PREFIX}load_ramp", _load_ramp(),
        location=_RAMP_LOC, **_kw(f"{_S1_PREFIX}load_ramp"),
    )
)

assembly = _splice_m02(assembly, _M02_ASSY, _M02_SPLICE_BASE, _S1_PREFIX)


# --- Intra-chain interference asserts ----------------------------------
#
# Chain skeleton parts vs m02 internals at S1. m02's standalone
# `khana check` covers m02's internal pairs (rotor↔stator, rotor↔stem,
# etc.); the chain only needs the cross-pair asserts.

_CHAIN_SKELETON = ("col1_ext", "col2_ext", f"{_S1_PREFIX}yoke", f"{_S1_PREFIX}load_ramp")
_S1_M02_TARGETS = (
    f"{_S1_PREFIX}rotor",
    f"{_S1_PREFIX}stator",
    f"{_S1_PREFIX}stem",
    f"{_S1_PREFIX}motor",
)

for s in _CHAIN_SKELETON:
    for m in _S1_M02_TARGETS:
        assembly = assembly.assert_no_interference(s, m)

# Chain skeleton self-pairs (columns vs yoke, ramp vs columns).
for a, b in (
    ("col1_ext", f"{_S1_PREFIX}yoke"),
    ("col2_ext", f"{_S1_PREFIX}yoke"),
    ("col1_ext", "col2_ext"),
    (f"{_S1_PREFIX}load_ramp", "col1_ext"),
    (f"{_S1_PREFIX}load_ramp", "col2_ext"),
    (f"{_S1_PREFIX}load_ramp", f"{_S1_PREFIX}yoke"),
):
    assembly = assembly.assert_no_interference(a, b)


if __name__ == "__main__":
    check(assembly, out="outputs")
