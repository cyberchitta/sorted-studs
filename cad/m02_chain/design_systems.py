"""m02 chain — per-module design-system tags.

The chain unit composes m02 singulator(s) hanging off column
extensions above the m05 diverter's top deck. Only the chain's OWN
parts are tagged here — column extensions, yoke crossbars, stator
support plates, and LOAD ramp. The m02 singulator composes as the
`s1` subtree and carries m02's own color/material bindings (tagged
in `cad/m02_c_channel_singulator/design_systems.py`); the old
splice-time re-skin through mirrored `s1__*` entries retired with
the subtree composition.

Functional roles follow the same convention as m05:
- Column extensions + yokes (2020 aluminum): PRIMARY_STRUCTURE.
- Stator support plate + LOAD ramp (FDM printed): SECONDARY_STRUCTURE.

Module assignment (BAUHAUS palette): the columns + yokes are
diverter-frame extensions; tag them DIVERTER. The stator support
plate and ramp are clearly SINGULATOR. The chain-as-system is
bi-modal — that's accurate to the physical reality that the m02
singulator is structurally bolted to the m05 frame.
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


# --- Chain part role tags ---------------------------------------------
#
# Chain-own parts only (identity is the tree path; per-stage m02
# instances live in `s1`/`s2`/`s3` subtrees, tagged by m02 itself).
# `stator_plate` is tagged ahead of the deferred stator-mounting
# interface (see `params.py`).

ROLE_OF: dict[str, Role | None] = {
    "col1_ext":     Role.PRIMARY_STRUCTURE,
    "col2_ext":     Role.PRIMARY_STRUCTURE,
    "yoke":         Role.PRIMARY_STRUCTURE,
    "stator_plate": Role.SECONDARY_STRUCTURE,
    "load_ramp":    Role.SECONDARY_STRUCTURE,
}


# --- Chain part module tags -------------------------------------------

MODULE_OF: dict[str, Module | None] = {
    # Column extensions + yoke are physically m05 frame continuations
    # — they bolt onto the diverter's deck. Tag DIVERTER for the
    # bauhaus palette's wayfinding to read correctly (blue extrusions
    # rise out of the diverter blue base).
    "col1_ext":     Module.DIVERTER,
    "col2_ext":     Module.DIVERTER,
    "yoke":         Module.DIVERTER,
    # Stator support + ramp are singulator-side hardware.
    "stator_plate": Module.SINGULATOR,
    "load_ramp":    Module.SINGULATOR,
}


# --- Bound DesignSystem -----------------------------------------------

DESIGN_SYSTEM: DesignSystem = bind_all(ROLE_OF, MODULE_OF)[resolve_name("M02_CHAIN")]


def color_of(part_name: str) -> Color | None:
    """Active design system's color for a named chain part."""
    return DESIGN_SYSTEM.color_of(part_name)
