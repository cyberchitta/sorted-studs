"""m02 chain — per-module design-system tags.

The chain unit composes m02 singulator(s) hanging off column
extensions above the m05 diverter's top deck. Only the chain's OWN
parts are tagged here — the gantry's 2020 members (prop bars, S3's
cross, S2's stem and T bar, the two braces), their corner brackets,
the printed joints, and the LOAD ramp. Each m02 singulator composes
as an `s1` / `s2` / `s3` subtree and carries m02's own color/material
bindings (tagged in `cad/m02_c_channel_singulator/design_systems.py`);
the old splice-time re-skin through mirrored `s1__*` entries retired
with the subtree composition.

Functional roles follow the same convention as m05:
- Gantry members (2020 aluminum): PRIMARY_STRUCTURE.
- Printed joints + LOAD ramp (FDM printed): SECONDARY_STRUCTURE.

Module assignment (BAUHAUS palette): the gantry members are
diverter-frame extensions (they bolt to m05's columns and deck); tag
them DIVERTER, and the printed joints and corner brackets with
them. The ramp is clearly SINGULATOR. The chain-as-system is
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

_GANTRY = (
    "s1_prop_bar", "s3_prop_bar", "s3_cross_a", "s3_cross_b",
    "s2_stem", "s2_tee", "brace_3", "brace_5",
)
# Bought 2020 corner brackets — frame hardware, with the members.
_BRACKETS = (
    "s2_stem_bracket_top", "s2_stem_bracket_left", "s2_stem_bracket_right",
    "s2_tee_bracket_left", "s2_tee_bracket_right",
    "s3_cross_bracket_a1", "s3_cross_bracket_a2", "s3_cross_bracket_b1", "s3_cross_bracket_b2",
)
# Printed joints (`joints.py`): the post seats, the brace collar and
# the brace feet.
_PRINTED_JOINTS = ("s1_post_seat", "s3_post_seat", "brace_head", "brace_foot_3", "brace_foot_5")

ROLE_OF: dict[str, Role | None] = {
    **{name: Role.PRIMARY_STRUCTURE for name in _GANTRY + _BRACKETS},
    **{name: Role.SECONDARY_STRUCTURE for name in _PRINTED_JOINTS},
    "load_ramp":    Role.SECONDARY_STRUCTURE,
    # Bought sensor envelopes, outside the palette.
    "load_beam_emitter": None,
    "load_beam_receiver": None,
}


# --- Chain part module tags -------------------------------------------

MODULE_OF: dict[str, Module | None] = {
    # The gantry is physically an m05 frame continuation — it bolts
    # onto the diverter's columns and deck. Tag DIVERTER for the
    # bauhaus palette's wayfinding to read correctly (blue extrusions
    # rise out of the diverter blue base).
    **{name: Module.DIVERTER for name in _GANTRY + _BRACKETS + _PRINTED_JOINTS},
    # The ramp is singulator-side hardware.
    "load_ramp":    Module.SINGULATOR,
    "load_beam_emitter": None,
    "load_beam_receiver": None,
}


# --- Bound design systems ---------------------------------------------

PALETTES: dict[str, DesignSystem] = bind_all(ROLE_OF, MODULE_OF)


def design_system(schema: str = DEFAULT_SCHEMA) -> DesignSystem:
    """The chain's tag dicts bound to ``schema``.

    Covers the chain's *own* parts only. ``assembly.build_chain``
    paints with the three stage subtrees skipped, because each carries
    m02's bindings, made against m02's tag dict.
    """
    return lookup(PALETTES, schema)
