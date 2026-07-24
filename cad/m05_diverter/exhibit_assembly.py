"""
M05 diverter — exhibit variant for the subsite article pane.

A 3-floor tower with the production top-acrylic deck in place but
most baskets stripped, so the rotating column + ramp + trap-door
mechanism is visible *through* the structure. The production 4-floor
× 11-basket assembly is opaque from any useful camera angle; this
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


from cad_khana.mechanism.check import check

from .assembly import build_tower


# Three floors: base + middle + top. Enough vertical extent to show
# the column-to-column joints and the top acrylic deck; one role of
# each.
EXHIBIT_FLOOR_PLAN = [
    (0, "base"),
    (1, "middle"),
    (2, "top"),
]

# All four basket sizes (N=6, 9, 12, 18) live in the exhibit, spread
# across the three floors so each size lands on its own level rather
# than stacking under itself. Front-facing slots (azimuths within
# ~±60° of +X) are kept clear on the upper two floors — only the
# base floor carries the canonical N=6 at +X, so a viewer looking
# into the tower from the front sees through to the mechanism on the
# floors above. Each entry is drawn from ``FLOOR_LAYOUT`` to keep
# design-system tag coverage intact.
EXHIBIT_BASKETS_BY_FLOOR = {
    0: [  # base — the canonical N=6 at +X, paired with one back N=18
        (  0.0,  6, {"left", "right"}),
        (240.0, 18, set()),
    ],
    1: [  # middle — N=9 + N=12 around the back, front clear
        (130.0,  9, {"right"}),
        (165.0, 12, {"left"}),
    ],
    2: [  # top — a second N=9 + a second N=18 on the back-right,
          # different azimuths from below so nothing stacks
        ( 90.0,  9, {"middle"}),
        (260.0, 18, {"right"}),
    ],
}


assembly = build_tower(
    EXHIBIT_FLOOR_PLAN,
    basket_layout=EXHIBIT_BASKETS_BY_FLOOR,
    with_top_deck=True,
    with_drive_train=False,
)


if __name__ == "__main__":
    check(assembly, out="outputs/exhibit")
