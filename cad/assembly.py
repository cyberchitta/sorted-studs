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

from functools import cache

from build123d import Location, Pos

from cad_khana.mechanism.assembly import Assembly

from .design_systems import DEFAULT_SCHEMA
from .m02_chain.assembly import build_chain
from .m03_scanner.assembly import build_m03
from .m05_diverter.assembly import build_m05


# m05 at world identity — its local frame IS the world frame.
_M05_POSE = Location()


# Cached because the degenerate ``assembly`` memo below and any
# composing parent (``cad/assembly.py``, ``cad/render.py``) each ask
# for the same schema, and the build is seconds of OCCT work. Replaces
# import-time memoization: the memo line is now free, and a parent
# calling the builder gets the same object rather than rebuilding.
# Safe to share — ``Assembly`` is a frozen dataclass and every
# ``with_*`` returns a copy.
#
# Call it with the schema **explicit** everywhere, including the memo
# below: ``@cache`` keys on the call signature, so ``build_x()`` and
# ``build_x(DEFAULT_SCHEMA)`` are two entries that build identical
# geometry twice. That mistake is invisible — same output, double the
# work — so ``cache_info().misses`` is the only thing that shows it.
@cache
def build_top(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The whole machine: the three units composed at their derived
    world poses, with the cross-unit checks.

    ``schema`` threads down to each unit builder, which binds its own
    tag dicts and paints its own subtree — so this level never touches
    a palette. One process can build every schema in turn; nothing is
    captured at import.

    The unit poses derive from the units' exported anchors rather than
    mirrored constants, and those reads happen *here*, inside the
    builder, against the assemblies this call just built. They used to
    run at import against module-level objects, which silently tied the
    poses to whatever schema those objects were built under — harmless
    while there was one global schema, wrong the moment there are
    several in flight.
    """
    m05 = build_m05(schema)
    m03 = build_m03(schema)
    chain = build_chain(schema)

    # m03 placement:
    #   - xy: the turret's DUMP centroid sits on the m05 hub axis
    #     (option a', 2026-05-05) — m05's `deck_top` anchor carries the
    #     hub xy.
    #   - z: the turret floor's bottom face rests on the m05 deck top
    #     face — m03's `floor_bottom` meets m05's `deck_top` plane.
    # Both constraints are pure translation, so the pose is
    # componentwise arithmetic on the anchor readings (890 − (−140) =
    # 1030 etc.).
    deck_top = m05.anchor("deck_top").position
    dump = m03.anchor("dump_centroid").position
    floor_bottom = m03.anchor("floor_bottom").position
    m03_pose = Pos(
        deck_top.X - dump.X,
        deck_top.Y - dump.Y,
        deck_top.Z - floor_bottom.Z,
    )

    # m02 chain placement: the chain's `load_top` anchor must land on
    # m03's live `load_top` (the LOAD platform top centroid the chain
    # delivers onto). The chain's own mirrored beliefs are asserted
    # against the live units after composition (the anchor asserts
    # below) instead of the retired namespace cross-checks.
    load_top_world = (m03_pose * m03.anchor("load_top")).position
    chain_load_local = chain.anchor("load_top").position
    chain_pose = Pos(
        load_top_world.X - chain_load_local.X,
        load_top_world.Y - chain_load_local.Y,
        load_top_world.Z - chain_load_local.Z,
    )

    return (
        Assembly()
        .with_subassembly("m05", m05, location=_M05_POSE)
        .with_subassembly("m03", m03, location=m03_pose)
        .with_subassembly("m02_chain", chain, location=chain_pose)
        # Every cross-unit pair (m03 × m05, chain × m05, chain × m03),
        # each unit addressed by its subtree path. Within-unit pairs are
        # covered by each unit's own standalone `khana check` (nested
        # assertions are root-only and not evaluated here).
        .assert_no_interference_between("m03", "m05")
        .assert_no_interference_between("m02_chain", "m05")
        .assert_no_interference_between("m02_chain", "m03")
        # Each unit exports where it believes the shared datums are —
        # m05/m03 from their live values, the chain from the constants
        # it mirrors so it can build standalone. Composed, the beliefs
        # must coincide; a drifted chain mirror fails here loudly
        # instead of silently desyncing the machine. (m03 has nothing
        # left to drift: its pose is *derived* from the live anchors
        # above.)
        .assert_anchors_coincident("m02_chain.deck_top", "m05.deck_top")
        .assert_anchors_coincident(
            "m02_chain.col1_base", "m05.outer_vert_0_top"
        )
    )


# Degenerate memoized master, so ``khana check cad/assembly.py``
# resolves without a ``:factory``.
assembly = build_top(DEFAULT_SCHEMA)
