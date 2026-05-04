"""
sorted-studs — top-level integration assembly.

Composes the three module assemblies into one stack:

  * m05 (diverter)   — at world identity. Hex tower; bricks fall down
                       through its central hub, sorted into bins.
  * m03 (scanner)    — placed so its DUMP station sits over the
                       diverter's hub axis. Bench rests on the diverter
                       top deck.
  * m02 (singulator) — placed so its EXIT (rotor-base rim, +X side)
                       sits over the scanner's LOAD station, with a
                       free-fall gap for the brick handoff.

Brick flow (top → bottom):

    singulator EXIT  ──drop──▶  scanner LOAD station
                                scanner shuffles to IMAGE
                                scanner shuttles to DUMP
        scanner DUMP  ──drop──▶  diverter hub  ──▶  bin

Coordinate frame (world):
    +X = operator's right
    +Y = away from operator (back of machine)
    +Z = up (gravity is -Z)
    z=0 = floor / diverter base-floor rail datum.

Composition strategy:

Each module's `assembly.py` is a standalone script: it inserts its own
directory into ``sys.path`` and imports its sub-assemblies via bare
names (``from params import …``). The three modules use overlapping
bare names (every module owns a ``params``, ``materials``,
``design_systems``), so naively importing all three top-level
assemblies in one process collides in ``sys.modules`` — the second
load reuses the first's ``params``. We side-step that by running each
module's ``assembly.py`` through ``runpy.run_path`` with the colliding
cached modules evicted before/after every load. Each module sees a
clean namespace; we collect its module-level ``assembly`` object and
move on.

Once loaded, we re-add every placed part of each module's assembly
into a single top-level ``Assembly`` with its location pre-multiplied
by the module's world pose and its name prefixed by the module tag.
Within-module assertions are validated by running each module's own
``khana check`` — they are intentionally not copied here. The
top-level only adds *cross-module* no-interference assertions.

V1 placement is geometric only — the support structure between
modules (singulator support arm/column, scanner-to-deck mount, dump
chute, load chute) is deferred. See ``cad/NEXT_STEPS.md``.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

from build123d import Location, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check


THIS_DIR = Path(__file__).resolve().parent


# --- Cross-module load helper -----------------------------------------
#
# Names that every module's `assembly.py` (and its sub-assembly tree)
# imports via the bare ``from <name> import …`` pattern after a
# ``sys.path.insert(0, <module dir>)``. These collide across modules,
# so we evict them from ``sys.modules`` between loads and restore
# ``sys.path`` to its pre-load snapshot. Anything imported as a
# dotted name underneath one of these (e.g. ``params.foo``) is also
# evicted via the prefix check.

_MODULE_LOCAL_HEADS = frozenset(
    {
        # shared across every module
        "params",
        "design_systems",
        "materials",
        "detail_variations",
        "material_variations",
        # m02 internals
        "drive",
        "rotor",
        "stator",
        # m03 internals
        "camera_rig",
        "turret",
        # m05 internals
        "frame",
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


def _load_module_namespace(module_dirname: str) -> dict:
    saved_path = sys.path.copy()
    _evict_module_locals()
    try:
        path = THIS_DIR / module_dirname / "assembly.py"
        return runpy.run_path(str(path), run_name=f"<{module_dirname}>")
    finally:
        sys.path[:] = saved_path
        _evict_module_locals()


# --- Load each module ------------------------------------------------

_M05_NS = _load_module_namespace("m05_diverter")
_M03_NS = _load_module_namespace("m03_scanner")
_M02_NS = _load_module_namespace("m02_c_channel_singulator")

_M05_ASSY: Assembly = _M05_NS["assembly"]
_M03_ASSY: Assembly = _M03_NS["assembly"]
_M02_ASSY: Assembly = _M02_NS["assembly"]


# --- Interface values pulled from the loaded namespaces --------------
#
# Each value is sourced from the module that owns it. Where a value
# only lives in a sub-package's params (not re-exported by the
# module's assembly.py), we mirror it explicitly with the path-of-
# truth comment — this is the same pattern m03/assembly.py uses for
# WALL_MIN.

# m05: top of the through-columns and top deck thickness — both live
# at module-level in m05_diverter/assembly.py.
_M05_TOP_VERT_Z: float = _M05_NS["TOP_VERT_Z"]
_M05_TOP_DECK_T: float = _M05_NS["TOP_DECK_T"]
M05_TOP_DECK_TOP_Z: float = _M05_TOP_VERT_Z + _M05_TOP_DECK_T   # 890

# m03: turret arm radius — m03/assembly.py imports it from
# turret.params, so the value is in m03's namespace.
M03_TURRET_ARM_R: float = _M03_NS["TURRET_ARM_R"]

# m03: shared bench Z (the floor of the camera_rig + turret 2020s).
# Used to compute scanner placement so the bench sits on the deck.
_M03_SHARED_BENCH_Z: float = _M03_NS["SHARED_BENCH_Z"]          # -220

# m02: rotor-base rim radius — the radial coordinate of the brick
# exit point in singulator-local coordinates. Mirrors
# cad/m02_c_channel_singulator/params.py:ROTOR_BASE_DIAMETER.
M02_EXIT_RADIUS: float = 180.0 / 2.0   # = ROTOR_BASE_DIAMETER / 2 = 90 mm


# --- Vertical handoff clearances --------------------------------------
#
# Free-fall gaps for brick handoffs. Placeholder values for V1 — the
# real values come with the dump-chute / load-chute design slices.
# Intentionally generous so geometry doesn't collide while we iterate
# the chutes.

# The scanner can't rest on the deck: the top floor's central column
# pokes 43 mm above the deck top face (column joint-plane at
# z_local=273 above floor 3's rail datum z_world=660 → z_world=933).
# Lift the scanner so the bench-level 2020s clear that column. Mirrors
# m05's ramp_mechanism column geometry (joint-plane Z + RAIL_DATUM
# offset); a drift here surfaces as a cross-module interference.
M05_TOP_COLUMN_TOP_Z = 933.0
SCANNER_BENCH_LIFT_OVER_COLUMN = 20.0   # clean-air gap above the column top
_M03_RAIL_HALF_BELOW_BENCH = 10.0       # half-side of a 20×20 2020 extrusion

# Singulator V-bottom (z=0 in singulator local) hovers this many mm
# above the scanner platform top (z=0 in scanner local). The brick
# falls through this gap onto the LOAD station's acrylic. The drive
# stem extends 50 mm below the V-bottom in m02-local; this gap also
# has to clear that — pick 100 mm so 50 mm of daylight remains under
# the stem.
SINGULATOR_DROP_OVER_LOAD = 100.0


# --- World poses ------------------------------------------------------

# m05 at world identity.
_M05_POSE = Location()

# m03 placement: scanner-local DUMP at (-TURRET_ARM_R, 0, 0) must land
# at world (0, 0, scanner_platform_top_z). The platform_top_z is set
# so the bench-level 2020s' bottom face sits
# `SCANNER_BENCH_LIFT_OVER_COLUMN` above the diverter's top column —
# not on the deck (the column protrusion above the deck makes a
# direct deck-rest impossible).
#
#   bench_bottom_z_world = scanner-local (SHARED_BENCH_Z − rail_half)
#                          + scanner_translation_z
#                        ≥ column_top_z + lift
#
# Solving for scanner_translation_z (= platform_top_z, since
# scanner-local z=0 is the platform top):
_M03_PLATFORM_TOP_Z = (
    M05_TOP_COLUMN_TOP_Z
    + SCANNER_BENCH_LIFT_OVER_COLUMN
    + _M03_RAIL_HALF_BELOW_BENCH
    - _M03_SHARED_BENCH_Z       # subtract negative = +220
)   # 933 + 20 + 10 + 220 = 1183
_M03_POSE = Pos(M03_TURRET_ARM_R, 0, _M03_PLATFORM_TOP_Z)

# m02 placement: singulator-local exit (+M02_EXIT_RADIUS, 0, 0) must
# sit over scanner LOAD (which is at world (2·ARM_R, 0, platform_z)),
# elevated by SINGULATOR_DROP_OVER_LOAD.
_SCANNER_LOAD_WORLD_X = 2 * M03_TURRET_ARM_R                      # 370
_SCANNER_LOAD_WORLD_Z = _M03_PLATFORM_TOP_Z                       # 1110
_M02_POSE = Pos(
    _SCANNER_LOAD_WORLD_X - M02_EXIT_RADIUS,                      # 370 - 90 = 280
    0,
    _SCANNER_LOAD_WORLD_Z + SINGULATOR_DROP_OVER_LOAD,             # 1210
)


# --- Compose ----------------------------------------------------------

_PREFIX = {
    "m02": "m02__",
    "m03": "m03__",
    "m05": "m05__",
}


def _splice(top: Assembly, mod: Assembly, base: Location, prefix: str) -> Assembly:
    """Re-add every placed part from ``mod`` into ``top`` with its
    location pre-multiplied by ``base`` and its name prefixed.
    Materials, colors, and parts pass through unchanged. Sub-assembly
    assertions are intentionally dropped — within-module checks are
    validated by running each module's own ``khana check``.
    """
    for p in mod.parts:
        top = top.add(
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
assembly = _splice(assembly, _M02_ASSY, _M02_POSE, _PREFIX["m02"])


# --- Cross-module no-interference -------------------------------------
#
# Every part-pair across module boundaries. Within-module pairs are
# already covered by each module's standalone check. Pairs are
# generated from the loaded source assemblies (pre-prefix) and the
# prefix is reapplied here.

_M02_NAMES = [f"{_PREFIX['m02']}{p.name}" for p in _M02_ASSY.parts]
_M03_NAMES = [f"{_PREFIX['m03']}{p.name}" for p in _M03_ASSY.parts]
_M05_NAMES = [f"{_PREFIX['m05']}{p.name}" for p in _M05_ASSY.parts]

for a in _M02_NAMES:
    for b in _M03_NAMES:
        assembly = assembly.assert_no_interference(a, b)
    for c in _M05_NAMES:
        assembly = assembly.assert_no_interference(a, c)
for b in _M03_NAMES:
    for c in _M05_NAMES:
        assembly = assembly.assert_no_interference(b, c)


if __name__ == "__main__":
    check(assembly, out="outputs")
