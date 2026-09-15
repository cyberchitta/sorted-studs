"""m02 chain — per-module design-system tags.

The chain unit composes m02 singulator(s) hanging off column
extensions above the m05 diverter's top deck. Only the chain's OWN
parts are tagged here — the gantry's 2020 members (prop bars, S2's
stem and T bar, the two braces), the stator support plate, and the
LOAD ramp. The m02 singulator composes as the
`s1` subtree and carries m02's own color/material bindings (tagged
in `cad/m02_c_channel_singulator/design_systems.py`); the old
splice-time re-skin through mirrored `s1__*` entries retired with
the subtree composition.

Functional roles follow the same convention as m05:
- Gantry members (2020 aluminum): PRIMARY_STRUCTURE.
- Stator support plate + LOAD ramp (FDM printed): SECONDARY_STRUCTURE.

Module assignment (BAUHAUS palette): the gantry members are
diverter-frame extensions (they bolt to m05's columns and deck); tag
them DIVERTER. The stator support
plate and ramp are clearly SINGULATOR. The chain-as-system is
bi-modal — that's accurate to the physical reality that the m02
singulator is structurally bolted to the m05 frame.
"""

from __future__ import annotations


from build123d import Color


from ..design_systems import (
    DEFAULT_SCHEMA,
    DesignSystem,
    Module,
    Role,
    bind_all,
    lookup,
)


# --- Chain part role tags ---------------------------------------------
#
# Chain-own parts only (identity is the tree path; per-stage m02
# instances live in `s1`/`s2`/`s3` subtrees, tagged by m02 itself).
# `stator_plate` is tagged ahead of the deferred stator-mounting
# interface (see `params.py`).

_GANTRY = (
    "s1_prop_bar", "s3_prop_bar", "s3_cross_a", "s3_cross_b",
    "s2_stem", "s2_tee", "brace_3", "brace_5",
)

ROLE_OF: dict[str, Role | None] = {
    **{name: Role.PRIMARY_STRUCTURE for name in _GANTRY},
    "stator_plate": Role.SECONDARY_STRUCTURE,
    "load_ramp":    Role.SECONDARY_STRUCTURE,
}


# --- Chain part module tags -------------------------------------------

MODULE_OF: dict[str, Module | None] = {
    # The gantry is physically an m05 frame continuation — it bolts
    # onto the diverter's columns and deck. Tag DIVERTER for the
    # bauhaus palette's wayfinding to read correctly (blue extrusions
    # rise out of the diverter blue base).
    **{name: Module.DIVERTER for name in _GANTRY},
    # Stator support + ramp are singulator-side hardware.
    "stator_plate": Module.SINGULATOR,
    "load_ramp":    Module.SINGULATOR,
}


# --- Bound design systems ---------------------------------------------

PALETTES: dict[str, DesignSystem] = bind_all(ROLE_OF, MODULE_OF)


def design_system(schema: str = DEFAULT_SCHEMA) -> DesignSystem:
    """The chain's tag dicts bound to ``schema``.

    Covers the chain's *own* parts only. ``assembly.build_chain``
    paints with ``skip=("s1",)`` because the composed singulator
    subtree carries m02's bindings, made against m02's tag dict.
    """
    return lookup(PALETTES, schema)
