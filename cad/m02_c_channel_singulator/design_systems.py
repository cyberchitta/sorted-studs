"""m02 c-channel singulator — per-module design-system tags.

Schema (``Role``, ``Module``, ``DesignSystem``, palettes,
``DESIGN_SYSTEM_NAME`` switch, ``compose`` helper) lives in
``cad/design_systems.py``; this file holds only m02's tag dicts and
binds the module-local ``DESIGN_SYSTEM`` + ``color_of`` shim that
m02 sub-assemblies import.

m02 has four named parts: ``rotor`` (rotates, MECHANISM),
``stator`` (fixed structural shell, SECONDARY_STRUCTURE), ``stem``
(fixed support post hidden inside the rotor cavity,
SECONDARY_STRUCTURE), and ``motor`` (NEMA-17, outside the design
system — keeps its cast-aluminium intrinsic).
"""

from __future__ import annotations

import sys
from pathlib import Path

from build123d import Color


sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from cad.design_systems import (  # noqa: E402
    BAUHAUS_COLORS,
    DESIGN_SYSTEM_NAME,
    DesignSystem,
    Module,
    Role,
    WORKSHOP_COLORS,
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


# --- Bound DesignSystems ----------------------------------------------

WORKSHOP = DesignSystem(name="workshop", tag_of=ROLE_OF, palette=WORKSHOP_COLORS)
BAUHAUS = DesignSystem(name="bauhaus", tag_of=MODULE_OF, palette=BAUHAUS_COLORS)

DESIGN_SYSTEM: DesignSystem = (
    {"workshop": WORKSHOP, "bauhaus": BAUHAUS}[DESIGN_SYSTEM_NAME]
)


def color_of(part_name: str) -> Color | None:
    """Active design system's color for a named m02 part.

    Thin shim over ``DESIGN_SYSTEM.color_of(...)`` so call sites import
    one symbol regardless of which palette is active.
    """
    return DESIGN_SYSTEM.color_of(part_name)
