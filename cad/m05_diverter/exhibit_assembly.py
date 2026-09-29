"""
M05 diverter — exhibit variant for the subsite article pane.

A 3-floor tower with the production top-acrylic deck in place but
most baskets stripped, so the rotating column + ramp + trap-door
mechanism is visible *through* the structure. The production four-floor
tower (9–18 baskets a floor) is opaque from any useful camera angle; this
variant is what the article shows.

Reuses every per-floor composition helper from ``assembly.py`` via
``build_tower(...)`` — only the floor plan, per-floor basket layout,
and the ``with_drive_train`` flag differ. Same part functions, same
placement helpers, same assertions; no post-hoc stripping.

Per-floor baskets are drawn as a subset of ``FLOOR_LAYOUT`` so the
design-system tag coverage assertion in ``build_tower`` still
resolves every name; ``build_tower`` accepts a
``dict[floor_idx, layout]`` for per-floor overrides.
"""


from functools import cache

from cad_khana.mechanism.assembly import Assembly

from ..design_systems import DEFAULT_SCHEMA
from .assembly import build_tower


# Three floors: base + middle + top. Enough vertical extent to show
# the column-to-column joints and the top acrylic deck; one role of
# each.
EXHIBIT_FLOOR_PLAN = [
    (0, "base"),
    (1, "middle"),
    (2, "top"),
]

# The three basket sizes the layouts place (N=6, 12, 18 — no layout
# holds an N=9 since 2026-09-28), spread across the three floors at
# azimuths that don't stack over each other. Front-facing slots (azimuths within
# ~±60° of +X) are kept clear on the upper two floors — only the
# base floor carries the canonical N=6 at +X, so a viewer looking
# into the tower from the front sees through to the mechanism on the
# floors above. Each entry is drawn from its floor's ``FLOOR_LAYOUTS``
# entry, which keeps design-system tag coverage intact and gives each
# floor the same ramp variant as production (`aim.ramp_exit_w_for`).
EXHIBIT_BASKETS_BY_FLOOR = {
    0: [  # base — the canonical N=6 at +X, paired with one back N=12
        (  0.0,  6, {"left", "right"}),
        (225.0, 12, {"left"}),
    ],
    1: [  # middle — the small-parts floor: two N=18s around the back
        (140.0, 18, {"right"}),
        (180.0, 18, set()),
    ],
    2: [  # top — two N=12s on the back-right, clear of both below
        (285.0, 12, {"left"}),
        (315.0, 12, {"right"}),
    ],
}


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
def build_exhibit(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The static three-floor exhibit tower, painted under ``schema``."""
    return build_tower(
        EXHIBIT_FLOOR_PLAN,
        schema=schema,
        basket_layout=EXHIBIT_BASKETS_BY_FLOOR,
        with_top_deck=True,
        with_drive_train=False,
    )


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``. Privileged-stem rule: this file is not
# ``assembly.py``, so diagnostics land in ``outputs/exhibit_assembly/``.
assembly = build_exhibit(DEFAULT_SCHEMA)
