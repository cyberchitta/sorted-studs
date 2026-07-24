"""m02 c-channel singulator — per-module design-system tags.

Schema (``Role``, ``Module``, ``DesignSystem``, palettes,
``DESIGN_SYSTEM_NAME`` switch) lives in
``cad/design_systems.py``; this file holds only m02's tag dicts and
binds the module-local ``DESIGN_SYSTEM`` + ``color_of`` shim that
m02 sub-assemblies and the top-level assembly import. Material is a
physical property, not a render axis — see ``materials.py``.

m02 has four named parts: ``rotor`` (rotates, MECHANISM),
``stator`` (fixed structural shell, SECONDARY_STRUCTURE), ``stem``
(fixed support post hidden inside the rotor cavity,
SECONDARY_STRUCTURE), and ``motor`` (NEMA-17, outside the design
system — keeps its cast-aluminium intrinsic).
"""

from __future__ import annotations


from build123d import Color


from ..design_systems import (
    DesignSystem,
    Module,
    Role,
    bind_all,
    resolve_name,
)


# --- m02 role tags ----------------------------------------------------

ROLE_OF: dict[str, Role | None] = {
    "rotor":  Role.MECHANISM,
    # `stator` is the structural shell *and* the brick-contact surface.
    # SECONDARY_STRUCTURE wins today (not designed to be swapped during
    # normal operation); re-tag CONSUMABLE if a stator-swap workflow
    # for different brick mixes becomes a real thing.
    "stator": Role.SECONDARY_STRUCTURE,
    "stem":   Role.SECONDARY_STRUCTURE,
    "motor":  None,                          # NEMA-17 cast body — outside palette
}


# --- m02 module tags --------------------------------------------------

MODULE_OF: dict[str, Module | None] = {
    name: (Module.SINGULATOR if role is not None else None)
    for name, role in ROLE_OF.items()
}


# --- Bound DesignSystem -----------------------------------------------

DESIGN_SYSTEM: DesignSystem = bind_all(ROLE_OF, MODULE_OF)[resolve_name("M02")]


def color_of(part_name: str) -> Color | None:
    """Active design system's color for a named m02 part.

    Thin shim over ``DESIGN_SYSTEM.color_of(...)`` so call sites import
    one symbol regardless of which palette is active.
    """
    return DESIGN_SYSTEM.color_of(part_name)
