"""
M05 diverter — exhibit variant for the subsite article pane.

A stripped 2-floor tower with most baskets removed, so the rotating
column + ramp + trap-door mechanism is visible *through* the
structure. The production 4-floor × 11-basket assembly is opaque from
any useful camera angle; this variant is what the article shows.

Reuses every per-floor composition helper from ``assembly.py`` via
``build_tower(...)`` — only the floor plan, basket layout, and the
``with_top_deck`` / ``with_drive_train`` flags differ. Same part
functions, same placement helpers, same assertions; no post-hoc
stripping.

Basket layout is a subset of ``FLOOR_LAYOUT`` so the design-system
tag coverage assertion in ``build_tower`` still resolves every name.

Renders / animations are out of scope here — see
``subsite/diverter_exhibit.md`` for the output-format brief and the
chitra-cad ``rig`` pipeline that will read this assembly.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cad_khana.mechanism.check import check

from assembly import build_tower


# Floor plan: base + top, no middles. Two floors is enough to show the
# stacking and the cross-floor brick-fall-through; more floors would
# obscure the mechanism the exhibit is built to surface.
EXHIBIT_FLOOR_PLAN = [
    (0, "base"),
    (1, "top"),
]

# Two baskets per floor, drawn from FLOOR_LAYOUT so the
# design-system tag dict already covers them. The N=6 sits directly
# under the ramp's home azimuth (+X) — clearest "the brick lands
# here" diagram; the N=9 at 50° gives the reader a second target so
# the ramp's rotational choice is legible.
EXHIBIT_BASKETS = [
    (0.0, 6, {"left", "right"}),
    (50.0, 9, {"left"}),
]


assembly, _ = build_tower(
    EXHIBIT_FLOOR_PLAN,
    basket_layout=EXHIBIT_BASKETS,
    with_top_deck=False,
    with_drive_train=False,
)


if __name__ == "__main__":
    check(assembly, out="outputs/exhibit")
