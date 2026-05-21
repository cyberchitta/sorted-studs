"""
sorted-studs — top-level integration assembly.

Slice (2026-05-05): turret-on-diverter + camera rig at IMAGE
station. The merged m03 unit (rig + turret, post Phase-1 collapse)
is placed at world `Pos(0, +250, 1030)` so the rig brick centroid
lands on the IMAGE-station platform top face. This is a
visualisation slice — cross-unit assertions are expected to fail
where the unmodified rig's footprint runs off the deck OD or
crashes the m05 column. The m02 singulator is still decoupled.

Constraints landed in this slice:

  * **DUMP over hub.** The turret's DUMP station is its "down
    position" — at every dwell the cam-kicker tilts that platform so
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
modules evicted before/after every load. We then re-add every part
of each unit's assembly into a single top-level `Assembly`, with the
location pre-multiplied by the unit's world pose and the name
prefixed with the unit tag.
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
        "dump_cam",
        "dump_kicker",
        "homing",
        "hub",
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

_M05_ASSY: Assembly = _M05_NS["assembly"]
_M03_ASSY: Assembly = _M03_NS["assembly"]


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


# --- Compose ----------------------------------------------------------

_PREFIX = {
    "m05": "m05__",
    "m03": "m03__",
}


def _splice(top: Assembly, mod: Assembly, base: Location, prefix: str) -> Assembly:
    """Re-add every placed part from ``mod`` into ``top`` with its
    location pre-multiplied by ``base`` and its name prefixed.
    Within-unit assertions are intentionally dropped — each unit's
    own ``khana check`` validates them.
    """
    for p in mod.parts:
        top = top.with_part(
            f"{prefix}{p.name}",
            p.part,
            location=base * p.location,
            color=p.color,
            material=p.material,
        )
    return top


assembly = Assembly()
assembly = _splice(assembly, _M05_ASSY, _M05_POSE, _PREFIX["m05"])
assembly = _splice(assembly, _M03_ASSY, _M03_POSE, _PREFIX["m03"])


# --- Cross-unit no-interference --------------------------------------
#
# Every cross-unit pair (m03-vs-m05). Within-unit pairs (rig×rig,
# turret×turret, rig×turret) are covered by `m03_scanner/assembly.py`'s
# standalone check.

_M03_NAMES = [f"{_PREFIX['m03']}{p.name}" for p in _M03_ASSY.parts]
_M05_NAMES = [f"{_PREFIX['m05']}{p.name}" for p in _M05_ASSY.parts]

for n3 in _M03_NAMES:
    for n5 in _M05_NAMES:
        assembly = assembly.assert_no_interference(n3, n5)


if __name__ == "__main__":
    check(assembly, out="outputs")
