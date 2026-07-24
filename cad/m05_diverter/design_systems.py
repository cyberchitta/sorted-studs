"""m05 diverter — per-module design-system tags.

Schema (``Role``, ``Module``, ``DesignSystem``, palettes,
``DESIGN_SYSTEM_NAME`` switch) lives in
``cad/design_systems.py``; this file holds only m05's tag dicts and
binds the module-local ``DESIGN_SYSTEM`` + ``color_of`` shim that
``assembly.py`` and ``single_floor.py`` use at every ``.with_part(...)``
site. Material is a physical property, not a render axis — see
``materials.py``.

``basename_of`` lives here because both ``color_of`` and
``materials.material_of`` need it to map a qualified tree path to
its attribute-lookup leaf; ``materials.py`` imports it from this
file.

One complication relative to m02 / m03:

* **Stand-in names in ``single_floor.py``.** ``lower_trap_open`` is
  a stand-in for the floor below — same geometry as ``trap_open``,
  but added under its own name so the cross-boundary check matrix
  reads cleanly. Tagged the same as the real trap.
"""

from __future__ import annotations

from build123d import Color

from ..design_systems import (
    DesignSystem,
    Module,
    Role,
    bind_all,
    blend,
    resolve_name,
)
from .params import FLOOR_LAYOUT


# --- Basket name expansion --------------------------------------------

def _basket_basenames() -> list[str]:
    return [
        f"basket_{int(round(center_deg)):03d}_{n_size:02d}"
        for center_deg, n_size, _notches in FLOOR_LAYOUT
    ]


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
    "column": Role.MECHANISM,
    "hinge_pin": None,                        # steel pin — outside palette
    "ramp": Role.SECONDARY_STRUCTURE,
    "trap_closed": Role.MECHANISM,
    "trap_open": Role.MECHANISM,
    # ``single_floor.py`` stand-in for the floor below's trap-open.
    # Same geometry as ``trap_open``; tag matches.
    "lower_trap_open": Role.MECHANISM,
    # Baskets — textbook CONSUMABLE ("holds bricks; swapped often").
    **{name: Role.CONSUMABLE for name in _basket_basenames()},
    # Base-floor drive train
    "motor": None,                            # NEMA cast body — outside palette
    "pinion": None,                           # bought metal pinion — outside palette
    "ring_gear": Role.MECHANISM,              # printed drive gear
    "motor_mount": Role.SECONDARY_STRUCTURE,
    # Top deck — clear acrylic, outside the palette (renders via material).
    "top_acrylic": None,
}


# --- m05 module tags --------------------------------------------------

MODULE_OF: dict[str, Module | None] = {
    name: (Module.DIVERTER if role is not None else None)
    for name, role in ROLE_OF.items()
}


# --- Bound DesignSystem -----------------------------------------------

DESIGN_SYSTEM: DesignSystem = bind_all(ROLE_OF, MODULE_OF)[resolve_name("M05")]


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


def _page_shift_sign() -> int:
    return -1 if resolve_name("M05") in _DARK_PAGE_PALETTES else +1


def basket_color_of(n_size: int) -> Color:
    """Per-size basket color: the baskets' shared base color under the
    active design system, with a luminance shift keyed to ``n_size``
    and the active palette's page direction. Raises ``KeyError`` for
    an unknown size so a new basket variant surfaces loudly instead of
    silently inheriting the base color.
    """
    if n_size not in _BASKET_TINT_BY_N:
        raise KeyError(
            f"no basket-size tint for N={n_size} — add an entry to "
            f"_BASKET_TINT_BY_N in cad/m05_diverter/design_systems.py"
        )
    base = DESIGN_SYSTEM.color_of(_basket_basenames()[0])
    return blend(base, _BASKET_TINT_BY_N[n_size] * _page_shift_sign())


def color_of(part_name: str) -> Color | None:
    """Active design system's color for a m05 part instance.

    Accepts either a basename or a qualified tree path. Falls
    through to ``DESIGN_SYSTEM.color_of(...)``, which raises
    ``KeyError`` for an unknown basename — module-level coverage
    assertions in ``assembly.py`` / ``single_floor.py`` catch that
    earlier, but the explicit raise prevents silent off-palette
    renders.
    """
    return DESIGN_SYSTEM.color_of(basename_of(part_name))
