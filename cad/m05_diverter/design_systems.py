"""m05 diverter — per-module design-system tags.

Schema (``Role``, ``Module``, ``DesignSystem``, palettes, the palette
registry, ``paint``) lives in ``cad/design_systems.py``; this file
holds only m05's tag dicts, the ``design_system(schema)`` lookup, and
the ``color_for`` hook that carries the basket size ramp into
``paint``. ``assembly.py`` and ``single_floor.py`` paint their subtree
once on the way out; no ``.with_part(...)`` site sets a colour.
Material is a physical property, not a render axis — see
``materials.py``.

``basename_of`` lives here because ``materials.material_of`` needs it
to map a qualified tree path to its attribute-lookup leaf;
``materials.py`` imports it from this file. ``paint`` does not need it
— inside the tree a part's name is already its leaf.

One complication relative to m02 / m03:

* **Stand-in names in ``single_floor.py``.** ``lower_trap_servo`` is
  a stand-in for the floor below — same geometry as ``trap_servo``,
  but added under its own name so the cross-boundary check matrix
  reads cleanly. Tagged the same as the real servo.
"""

from __future__ import annotations

from build123d import Color

from ..design_systems import (
    DEFAULT_SCHEMA,
    DesignSystem,
    Module,
    Role,
    bind_all,
    blend,
    lookup,
)
from .params import all_basket_names


# --- Basket name expansion --------------------------------------------

def _basket_basenames() -> list[str]:
    return all_basket_names()


# --- m05 role tags ----------------------------------------------------
#
# Keys are *basenames* — the leaf part names the sub-assembly builders
# use. ``basename_of(...)`` below maps a qualified tree path to its
# leaf for lookup; instance identity (which floor) lives in the
# ``f{idx}`` subtree path, not the name.

ROLE_OF: dict[str, Role | None] = {
    # Frame — through-columns (tower-global)
    **{f"outer_vert_{k}": Role.PRIMARY_STRUCTURE for k in range(6)},
    **{f"inner_vert_{k}": Role.PRIMARY_STRUCTURE for k in range(6)},
    # Frame — per-floor horizontals
    **{f"outer_horiz_{k}": Role.PRIMARY_STRUCTURE for k in range(6)},
    **{f"inner_horiz_{k}": Role.PRIMARY_STRUCTURE for k in range(6)},
    **{f"radial_inner_{k}": Role.PRIMARY_STRUCTURE for k in range(6)},
    **{f"radial_outer_{k}": Role.PRIMARY_STRUCTURE for k in range(6)},
    # Frame — per-floor printed vertex brackets
    **{f"outer_vertex_bracket_{k}": Role.SECONDARY_STRUCTURE for k in range(6)},
    **{f"inner_vertex_bracket_{k}": Role.SECONDARY_STRUCTURE for k in range(6)},
    # Ramp mechanism
    "bearing_spider": Role.SECONDARY_STRUCTURE,
    # Passive PETG sleeve + grease. SECONDARY_STRUCTURE pairs it with
    # the spider (same shade); re-tag CONSUMABLE if a periodic sleeve-
    # swap workflow becomes routine.
    "ring_bearing": Role.SECONDARY_STRUCTURE,
    # The fifth sleeve, in the top deck's aperture — as ring_bearing.
    "ceiling_sleeve": Role.SECONDARY_STRUCTURE,
    "column": Role.MECHANISM,
    "hinge_pin": None,                        # steel pin — outside palette
    "ramp": Role.SECONDARY_STRUCTURE,
    "trap": Role.MECHANISM,
    "trap_coupler": Role.MECHANISM,           # printed pin-to-horn disc
    "servo_horn": None,                       # bought with the servo — outside palette
    "trap_servo": None,                       # bought hobby servo — outside palette
    "harness": None,                          # bought servo wire stand-in — outside palette
    # ``single_floor.py`` stand-in for the floor below's servo.
    "lower_trap_servo": None,
    # Baskets — textbook CONSUMABLE ("holds bricks; swapped often").
    **{name: Role.CONSUMABLE for name in _basket_basenames()},
    # Base-floor drive train
    "motor": None,                            # NEMA cast body — outside palette
    "pinion": None,                           # bought metal pinion — outside palette
    "ring_gear": Role.MECHANISM,              # printed drive gear
    "motor_mount": Role.SECONDARY_STRUCTURE,
    # Bottom catch — the tray holds bricks and is emptied like a basket.
    "catch_tray": Role.CONSUMABLE,
    "catch_cradle": Role.SECONDARY_STRUCTURE,
    # Top deck — clear acrylic, outside the palette (renders via material).
    "top_acrylic": None,
}


# --- m05 module tags --------------------------------------------------

MODULE_OF: dict[str, Module | None] = {
    name: (Module.DIVERTER if role is not None else None)
    for name, role in ROLE_OF.items()
}


# --- Bound design systems ---------------------------------------------

PALETTES: dict[str, DesignSystem] = bind_all(ROLE_OF, MODULE_OF)


def design_system(schema: str = DEFAULT_SCHEMA) -> DesignSystem:
    """m05's tag dicts bound to ``schema``.

    Callers pair it with ``color_for`` below when painting, so the
    basket size ramp is applied along with the plain tag reads.
    """
    return lookup(PALETTES, schema)


# --- Instance-name → basename helpers ---------------------------------

def basename_of(part_name: str) -> str:
    """Map a m05 part identity (qualified tree path or bare leaf
    name) to its attribute-lookup basename: the leaf segment of a
    dotted path. Root-level parts (``top_acrylic``, the
    ``single_floor.py`` stand-ins) pass through unchanged.
    """
    return part_name.rsplit(".", 1)[-1]


# --- Basket size ramp -------------------------------------------------
#
# All baskets share one tag on every axis (``Role.CONSUMABLE`` on the
# role axis, ``Module.DIVERTER`` on the module axis), so they resolve
# to a single palette color. To make basket sizes (N = sides per
# polygon) visually distinguishable, ramp luminance off that shared
# base — looked up through the design system's own axis-agnostic
# ``color_of`` path, never by indexing the palette with a hardcoded
# axis key (a ``palette[Role.…]`` reach crashes under the
# module-keyed bauhaus palette).
#
# The ramp is anchored to the page: smaller baskets blend *toward* the
# page background (less contrast), larger baskets shift *away* (more
# contrast). The direction therefore flips between light and dark
# palettes — in light mode smaller = lighter (toward the warm-paper
# page), in dark mode smaller = darker (toward the near-black page).
#
# ``_BASKET_TINT_BY_N`` values are signed magnitudes; positive means
# "toward the page direction", negative means "away". The palette's
# page direction is resolved at lookup time via ``_PAGE_SHIFT_SIGN``:
# +1 for a light page (blend toward white), -1 for a dark page (blend
# toward black). Net shift fed to ``blend`` is ``tint * sign``.
_BASKET_TINT_BY_N: dict[int, float] = {
    6:  +0.30,   # smallest — most blended with the page
    9:  +0.10,
    12: -0.10,
    18: -0.30,   # largest — most contrasted from the page
}

# Palettes whose page background is dark. Other palettes are assumed
# light-backgrounded (the historical default). Adding a new dark
# palette? Add its name here.
_DARK_PAGE_PALETTES: frozenset[str] = frozenset({"cyberchitta-dark", "midnight"})


def _page_shift_sign(ds: DesignSystem) -> int:
    return -1 if ds.name in _DARK_PAGE_PALETTES else +1


def basket_color_of(ds: DesignSystem, n_size: int) -> Color:
    """Per-size basket color: the baskets' shared base color under
    ``ds``, with a luminance shift keyed to ``n_size`` and that
    palette's page direction. Raises ``KeyError`` for an unknown size
    so a new basket variant surfaces loudly instead of silently
    inheriting the base color.
    """
    if n_size not in _BASKET_TINT_BY_N:
        raise KeyError(
            f"no basket-size tint for N={n_size} — add an entry to "
            f"_BASKET_TINT_BY_N in cad/m05_diverter/design_systems.py"
        )
    base = ds.color_of(_basket_basenames()[0])
    return blend(base, _BASKET_TINT_BY_N[n_size] * _page_shift_sign(ds))


def color_for(ds: DesignSystem, leaf: str) -> Color | None:
    """``paint``'s per-part hook for m05 — a plain tag read except for
    baskets, which take the size-keyed luminance ramp.

    ``n_size`` comes out of the leaf name (``basket_090_12`` → 12), the
    same field the builder put there, so the ramp survives colour
    moving out of the builders: the size is part of the part's identity
    rather than a build argument that would have to be threaded.
    """
    if leaf.startswith("basket_"):
        return basket_color_of(ds, int(leaf.rsplit("_", 1)[-1]))
    return ds.color_of(leaf)
