"""m02 chain — per-module design-system tags.

The chain unit composes m02 singulator(s) hanging off column
extensions above the m05 diverter's top deck. Its own parts are the
column extensions, yoke crossbars, stator support plates, and LOAD
ramp; the spliced m02 parts (`s1__rotor`, `s1__stator`, ...) get
tagged here too so the chain's standalone coverage check passes.

Functional roles follow the same convention as m05:
- Column extensions + yokes (2020 aluminum): PRIMARY_STRUCTURE.
- Stator support plate + LOAD ramp (FDM printed): SECONDARY_STRUCTURE.
- Spliced m02 parts: re-use the m02 roles (rotor → MECHANISM, stator
  / stem → SECONDARY_STRUCTURE, motor → None).

Module assignment (BAUHAUS palette): the columns + yokes are
diverter-frame extensions; tag them DIVERTER. The stator support
plate, ramp, and the spliced m02 parts are clearly SINGULATOR. The
chain-as-system is bi-modal — that's accurate to the physical reality
that the m02 singulator is structurally bolted to the m05 frame.
"""

from __future__ import annotations

import sys
from pathlib import Path

from build123d import Color


sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from cad.design_systems import (  # noqa: E402
    DesignSystem,
    Module,
    Role,
    bind_all,
    resolve_name,
)


# --- Chain part role tags ---------------------------------------------

# Per-stage prefixes: `s1__`, `s2__`, `s3__`. Only S1 is wired up in
# this slice — S2 and S3 enter later.
_S1_PREFIX = "s1__"


_CHAIN_OWN_ROLES: dict[str, Role | None] = {
    "col1_ext":         Role.PRIMARY_STRUCTURE,
    "col2_ext":         Role.PRIMARY_STRUCTURE,
    f"{_S1_PREFIX}yoke":          Role.PRIMARY_STRUCTURE,
    f"{_S1_PREFIX}stator_plate":  Role.SECONDARY_STRUCTURE,
    f"{_S1_PREFIX}load_ramp":     Role.SECONDARY_STRUCTURE,
}

# Spliced m02 parts — keep in sync with m02's `ROLE_OF`.
_S1_M02_ROLES: dict[str, Role | None] = {
    f"{_S1_PREFIX}rotor":  Role.MECHANISM,
    f"{_S1_PREFIX}stator": Role.SECONDARY_STRUCTURE,
    f"{_S1_PREFIX}stem":   Role.SECONDARY_STRUCTURE,
    f"{_S1_PREFIX}motor":  None,
}


ROLE_OF: dict[str, Role | None] = {**_CHAIN_OWN_ROLES, **_S1_M02_ROLES}


# --- Chain part module tags -------------------------------------------

_CHAIN_OWN_MODULES: dict[str, Module | None] = {
    # Column extensions + yoke are physically m05 frame continuations
    # — they bolt onto the diverter's deck. Tag DIVERTER for the
    # bauhaus palette's wayfinding to read correctly (blue extrusions
    # rise out of the diverter blue base).
    "col1_ext":         Module.DIVERTER,
    "col2_ext":         Module.DIVERTER,
    f"{_S1_PREFIX}yoke":          Module.DIVERTER,
    # Stator support + ramp are singulator-side hardware.
    f"{_S1_PREFIX}stator_plate":  Module.SINGULATOR,
    f"{_S1_PREFIX}load_ramp":     Module.SINGULATOR,
}

_S1_M02_MODULES: dict[str, Module | None] = {
    name: (Module.SINGULATOR if role is not None else None)
    for name, role in _S1_M02_ROLES.items()
}


MODULE_OF: dict[str, Module | None] = {**_CHAIN_OWN_MODULES, **_S1_M02_MODULES}


# --- Bound DesignSystem -----------------------------------------------

DESIGN_SYSTEM: DesignSystem = bind_all(ROLE_OF, MODULE_OF)[resolve_name("M02_CHAIN")]


def color_of(part_name: str) -> Color | None:
    """Active design system's color for a named chain part."""
    return DESIGN_SYSTEM.color_of(part_name)
