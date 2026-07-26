"""m02 c-channel singulator — per-module design-system tags.

Schema (``Role``, ``Module``, ``DesignSystem``, palettes, the palette
registry, ``paint``) lives in ``cad/design_systems.py``; this file
holds only m02's tag dicts and the ``design_system(schema)`` lookup
that ``assembly.build_m02`` binds when it paints. Sub-assembly
builders import nothing from here — they emit no colour at all.
Material is a physical property, not a render axis — see
``materials.py``.

m02 has four named parts: ``rotor`` (rotates, MECHANISM),
``stator`` (fixed structural shell, SECONDARY_STRUCTURE), ``stem``
(fixed support post hidden inside the rotor cavity,
SECONDARY_STRUCTURE), and ``motor`` (NEMA-17, outside the design
system — keeps its cast-aluminium intrinsic).
"""

from __future__ import annotations


from ..design_systems import (
    DEFAULT_SCHEMA,
    DesignSystem,
    Module,
    Role,
    bind_all,
    lookup,
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


# --- Bound design systems ---------------------------------------------

PALETTES: dict[str, DesignSystem] = bind_all(ROLE_OF, MODULE_OF)


def design_system(schema: str = DEFAULT_SCHEMA) -> DesignSystem:
    """m02's tag dicts bound to ``schema``.

    Called by ``assembly.build_m02`` to paint its subtree; there is no
    module-global "active" binding, so nothing here reads the
    environment at import.
    """
    return lookup(PALETTES, schema)
