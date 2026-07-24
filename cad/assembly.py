"""
sorted-studs — top-level integration assembly.

Slice (2026-05-26): m05 diverter + m03 scanner (rig + turret) + m02
chain (S1 stage + column extensions + LOAD ramp). The chain delivers
bricks onto the m03 turret's LOAD station; the turret rotates them
through IMAGE → IDLE → DUMP over the m05 hub axis.

Constraints landed in this slice:

  * **DUMP over hub.** The turret's DUMP station is its "down
    position" — at every dwell the dump servo tilts that platform so
    the brick falls. For the brick to drop into the diverter's
    central column, DUMP must sit on the column's hub axis at world
    `(0, 0, *)`. Option (a') lands the firing heading at 270° (-Y in
    turret-local), so DUMP lives at turret-local `(0, -TURRET_ARM_R,
    0)` and the turret-local origin maps to world
    `(0, +TURRET_ARM_R, *)`.
  * **Turret rests on the deck.** The turret floor frame's bottom
    face sits on the diverter top deck's top face at world z =
    `TOP_VERT_Z + TOP_DECK_T = 890`.

Cross-module no-interference is asserted (every turret part vs every
m05 part). Within-module assertions are validated by each module's
own `khana check`.

Coordinate frame (world):
    +X = operator's right
    +Y = away from operator (back of machine)
    +Z = up (gravity is -Z)
    z=0 = diverter base-floor rail datum.

Composition strategy:

Each unit's `assembly.py` is standalone — it inserts its own
directory into `sys.path` and imports its sub-assemblies via bare
names. The bare names collide across units (every unit owns a
`params`, `materials`, `design_systems`), so we run each unit's
`assembly.py` through `runpy.run_path` with the colliding cached
modules evicted before/after every load.

Each unit then composes as a true subtree — `with_subassembly(name,
…, location=world_pose)` — so parts keep their tree paths under the
unit root (`m05.…`, `m03.…`, `m02_chain.…`) and every unit's joints
survive to the top level: the whole machine can animate from here.
Within-unit assertions ride along nested and are intentionally not
evaluated by this root's `check()` (root-only evaluate) — each unit's
own `khana check` validates them.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

from build123d import Location, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check


THIS_DIR = Path(__file__).resolve().parent


# --- Cross-unit load helper -------------------------------------------
#
# Names that every unit's `assembly.py` (and its sub-assembly tree)
# imports via the bare ``from <name> import …`` pattern after a
# ``sys.path.insert(0, <unit dir>)``. These collide across units, so
# we evict them from ``sys.modules`` between loads and restore
# ``sys.path`` to its pre-load snapshot. Anything imported as a
# dotted name underneath one of these (e.g. ``params.foo``) is also
# evicted via the prefix check.

_MODULE_LOCAL_HEADS = frozenset(
    {
        # shared across every unit
        "params",
        "design_systems",
        "materials",
        "detail_variations",
        "material_variations",
        # m02 internals (held in reserve — m02 returns in a later slice)
        "rotor",
        "stator",
        # m03 internals (Phase-2 integrated frame, 2026-05-06)
        "arch",
        "arm",
        "camera_mount_esp32cam",
        "dump_servo",
        "homing",
        "hub",
        "sizing",
        "turret_platform",
        # shared between m02 (drive), m03 (drive, frame), m05 (frame)
        "drive",
        "frame",
        # m05 internals
        "basket",
        "ramp_mechanism",
        "base_floor",
        "inner_vertex_bracket",
        "outer_vertex_bracket",
    }
)


def _evict_module_locals() -> None:
    for name in list(sys.modules):
        head = name.split(".", 1)[0]
        if head in _MODULE_LOCAL_HEADS:
            del sys.modules[name]


def _load_unit_namespace(rel_path: str) -> dict:
    """Run a unit's `assembly.py` in isolation and return its module
    namespace. `rel_path` is the unit directory relative to `THIS_DIR`
    (e.g. `"m05_diverter"` or `"m03_scanner/turret"`).
    """
    saved_path = sys.path.copy()
    _evict_module_locals()
    try:
        path = THIS_DIR / rel_path / "assembly.py"
        return runpy.run_path(str(path), run_name=f"<{rel_path}>")
    finally:
        sys.path[:] = saved_path
        _evict_module_locals()


# --- Load each unit ---------------------------------------------------

_M05_NS = _load_unit_namespace("m05_diverter")
_M03_NS = _load_unit_namespace("m03_scanner")
_M02_CHAIN_NS = _load_unit_namespace("m02_chain")

_M05_ASSY: Assembly = _M05_NS["assembly"]
_M03_ASSY: Assembly = _M03_NS["assembly"]
_M02_CHAIN_ASSY: Assembly = _M02_CHAIN_NS["assembly"]


# --- Interface values pulled from the loaded namespaces --------------

# m05: top-of-through-columns Z + top-deck thickness — both live at
# module-level in `m05_diverter/assembly.py`. The deck top face is
# the surface the turret floor rests on.
_M05_TOP_VERT_Z: float = _M05_NS["TOP_VERT_Z"]
_M05_TOP_DECK_T: float = _M05_NS["TOP_DECK_T"]
M05_DECK_TOP_Z: float = _M05_TOP_VERT_Z + _M05_TOP_DECK_T   # 890

# Turret arm radius (hub axis → station centroid). Mirrors
# `cad/m03_scanner/params.py:TURRET_ARM_R`.
TURRET_ARM_R: float = 125.0

# Turret floor frame: cross-section centerline Z and member side.
# Mirrors `cad/m03_scanner/params.py:TURRET_FRAME_FLOOR_Z` and the
# 2020 cross-section MEMBER constant local to
# `cad/m03_scanner/frame/assembly.py`.
_TURRET_FLOOR_CL_Z: float = -130.0
_TURRET_FLOOR_MEMBER: float = 20.0
# Bottom face of the floor 2020s in turret-local coords.
_TURRET_FLOOR_BOTTOM_Z: float = _TURRET_FLOOR_CL_Z - _TURRET_FLOOR_MEMBER / 2.0  # -140


# --- World poses ------------------------------------------------------

# m05 at world identity.
_M05_POSE = Location()

# m03 placement. The m03 unit's internal frame has the rig at
# identity (brick centroid at m03 origin) and the turret offset to
# m03-local `Pos(0, -TURRET_ARM_R, 0)`, so:
#   - m03-local (0, 0, 0) = brick centroid on the IMAGE platform top
#     face → world (0, +2·TURRET_ARM_R, _TURRET_WORLD_Z) = (0, +250,
#     1030).
#   - m03-local (0, -TURRET_ARM_R, 0) = turret hub axis → world
#     (0, +TURRET_ARM_R, _TURRET_WORLD_Z) = (0, +125, 1030).
#   - m03-local (0, -2·TURRET_ARM_R, 0) = DUMP centroid → world
#     (0, 0, _TURRET_WORLD_Z) over the diverter hub axis (option a',
#     2026-05-05).
# Z chosen so the turret floor's bottom face lands on the m05 deck
# top face: world z = M05_DECK_TOP_Z - _TURRET_FLOOR_BOTTOM_Z =
# 890 - (-140) = 1030.
_TURRET_WORLD_Z = M05_DECK_TOP_Z - _TURRET_FLOOR_BOTTOM_Z   # 890 - (-140) = 1030
_M03_POSE = Pos(0, 2 * TURRET_ARM_R, _TURRET_WORLD_Z)

# m02 chain placement. The chain owns its own local frame with origin
# at the LOAD platform top centroid; one transform places it in world.
# Anchors mirrored inside `m02_chain/params.py` are cross-checked here
# against the live m05 / m03 values.
_M02_CHAIN_PARAMS = _M02_CHAIN_NS  # exposes the module dict; pull the
# mirrored constants the chain hard-codes so we can verify them.
_CHAIN_WORLD_POSE_XYZ: tuple = _M02_CHAIN_NS["CHAIN_WORLD_POSE_XYZ"]
_M02_CHAIN_POSE = Pos(*_CHAIN_WORLD_POSE_XYZ)

# Chain mirror cross-checks. The chain hard-codes M05_TOP_VERT_Z,
# M05_TOP_DECK_T, M03_TURRET_ARM_R, and M03_LOAD_TOP_Z (= turret world
# z). If any drifts, fail loudly here so the chain's anchors don't
# silently desync from the rest of the assembly.
assert _M02_CHAIN_NS["M05_TOP_VERT_Z"] == _M05_TOP_VERT_Z, (
    f"chain M05_TOP_VERT_Z ({_M02_CHAIN_NS['M05_TOP_VERT_Z']}) != live "
    f"m05 TOP_VERT_Z ({_M05_TOP_VERT_Z}); update "
    f"cad/m02_chain/params.py"
)
assert _M02_CHAIN_NS["M05_TOP_DECK_T"] == _M05_TOP_DECK_T, (
    f"chain M05_TOP_DECK_T ({_M02_CHAIN_NS['M05_TOP_DECK_T']}) != live "
    f"m05 TOP_DECK_T ({_M05_TOP_DECK_T}); update "
    f"cad/m02_chain/params.py"
)
assert _M02_CHAIN_NS["M03_TURRET_ARM_R"] == TURRET_ARM_R, (
    f"chain M03_TURRET_ARM_R ({_M02_CHAIN_NS['M03_TURRET_ARM_R']}) != "
    f"live m03 TURRET_ARM_R ({TURRET_ARM_R}); update "
    f"cad/m02_chain/params.py"
)
assert _M02_CHAIN_NS["M03_LOAD_TOP_Z"] == _TURRET_WORLD_Z, (
    f"chain M03_LOAD_TOP_Z ({_M02_CHAIN_NS['M03_LOAD_TOP_Z']}) != live "
    f"_TURRET_WORLD_Z ({_TURRET_WORLD_Z}); update "
    f"cad/m02_chain/params.py"
)


# --- Compose ----------------------------------------------------------

assembly = (
    Assembly()
    .with_subassembly("m05", _M05_ASSY, location=_M05_POSE)
    .with_subassembly("m03", _M03_ASSY, location=_M03_POSE)
    .with_subassembly("m02_chain", _M02_CHAIN_ASSY, location=_M02_CHAIN_POSE)
)


# --- Cross-unit no-interference --------------------------------------
#
# Every cross-unit pair (m03 × m05, chain × m05, chain × m03), each
# unit addressed by its subtree path. Within-unit pairs are covered by
# each unit's own standalone `khana check` (nested assertions are
# root-only and not evaluated here).

assembly = assembly.assert_no_interference_between("m03", "m05")
assembly = assembly.assert_no_interference_between("m02_chain", "m05")
assembly = assembly.assert_no_interference_between("m02_chain", "m03")


if __name__ == "__main__":
    check(assembly, out="outputs")
