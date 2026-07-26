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

`cad/` is a package: each unit is imported as a plain module
(`cad.m05_diverter.assembly` etc. — no name collisions, no loader
machinery) and composes as a true subtree — `with_subassembly(name,
…, location=world_pose)` — so parts keep their tree paths under the
unit root (`m05.…`, `m03.…`, `m02_chain.…`) and every unit's joints
survive to the top level: the whole machine can animate from here.
World poses are *derived from the units' exported anchors* (m05
`deck_top`, m03 `dump_centroid` / `floor_bottom` / `load_top`), and
the chain's mirrored beliefs are asserted coincident with the live
units after composition — no mirrored placement constants live here.
Within-unit assertions ride along nested and are intentionally not
evaluated by this root's `check()` (root-only evaluate) — each unit's
own `khana check` validates them.
"""

from __future__ import annotations

from build123d import Location, Pos

from cad_khana.mechanism.assembly import Assembly

from .m02_chain.assembly import assembly as _M02_CHAIN_ASSY
from .m03_scanner.assembly import assembly as _M03_ASSY
from .m05_diverter.assembly import assembly as _M05_ASSY


# --- World poses (derived from the units' exported anchors) -----------

# m05 at world identity — its local frame IS the world frame.
_M05_POSE = Location()

# m03 placement, derived from anchors instead of mirrored constants:
#   - xy: the turret's DUMP centroid sits on the m05 hub axis (option
#     a', 2026-05-05) — m05's `deck_top` anchor carries the hub xy.
#   - z: the turret floor's bottom face rests on the m05 deck top
#     face — m03's `floor_bottom` meets m05's `deck_top` plane.
# Both constraints are pure translation, so the pose is componentwise
# arithmetic on the anchor readings (890 − (−140) = 1030 etc.).
_DECK_TOP = _M05_ASSY.anchor("deck_top").position
_DUMP = _M03_ASSY.anchor("dump_centroid").position
_FLOOR_BOTTOM = _M03_ASSY.anchor("floor_bottom").position
_M03_POSE = Pos(
    _DECK_TOP.X - _DUMP.X,
    _DECK_TOP.Y - _DUMP.Y,
    _DECK_TOP.Z - _FLOOR_BOTTOM.Z,
)

# m02 chain placement: the chain's `load_top` anchor must land on
# m03's live `load_top` (the LOAD platform top centroid the chain
# delivers onto). The chain's own mirrored beliefs are asserted
# against the live units after composition (see the anchor asserts
# below) instead of the retired namespace cross-checks.
_LOAD_TOP_WORLD = (_M03_POSE * _M03_ASSY.anchor("load_top")).position
_CHAIN_LOAD_LOCAL = _M02_CHAIN_ASSY.anchor("load_top").position
_M02_CHAIN_POSE = Pos(
    _LOAD_TOP_WORLD.X - _CHAIN_LOAD_LOCAL.X,
    _LOAD_TOP_WORLD.Y - _CHAIN_LOAD_LOCAL.Y,
    _LOAD_TOP_WORLD.Z - _CHAIN_LOAD_LOCAL.Z,
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


# --- Cross-unit interface anchors -------------------------------------
#
# Each unit exports where it believes the shared datums are — m05/m03
# from their live values, the chain from the constants it mirrors so
# it can build standalone. Composed, the beliefs must coincide; a
# drifted chain mirror fails here loudly instead of silently
# desyncing the machine. (m03 has nothing left to drift: its pose is
# *derived* from the live anchors above.)
assembly = assembly.assert_anchors_coincident(
    "m02_chain.deck_top", "m05.deck_top"
)
assembly = assembly.assert_anchors_coincident(
    "m02_chain.col1_base", "m05.outer_vert_0_top"
)
