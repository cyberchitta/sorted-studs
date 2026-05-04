"""m05 diverter — per-module design-system tags.

Schema (``Role``, ``Module``, ``DesignSystem``, palettes,
``DESIGN_SYSTEM_NAME`` switch, ``compose`` helper) lives in
``cad/design_systems.py``; this file holds only m05's tag dicts and
binds the module-local ``DESIGN_SYSTEM`` + ``color_of`` /
``material_of`` shims that ``assembly.py`` and ``single_floor.py``
use at every ``.add(...)`` site.

Two complications relative to m02 / m03:

* **Floor-prefixed instance names.** ``assembly.py`` adds parts as
  ``f{idx}_{role}__<basename>`` so the four-floor stack stays
  collision-free. ``basename_of(...)`` strips that prefix; tower-
  global parts (``outer_vert_k``) carry no prefix and pass through.

* **Stand-in names in ``single_floor.py``.** ``lower_trap_open`` is
  a stand-in for the floor below — same geometry as ``trap_open``,
  but added under its own name so the cross-boundary check matrix
  reads cleanly. Tagged the same as the real trap.
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

sys.path.insert(0, str(Path(__file__).resolve().parent))

from params import FLOOR_LAYOUT  # noqa: E402


# --- Basket name expansion --------------------------------------------

def _basket_basenames() -> list[str]:
    return [
        f"basket_{int(round(center_deg)):03d}_{n_size:02d}"
        for center_deg, n_size, _notches in FLOOR_LAYOUT
    ]


# --- m05 role tags ----------------------------------------------------
#
# Keys are *basenames* — the part names sub-assembly builders use,
# without any per-floor prefix. ``basename_of(...)`` below maps an
# instance name back to a basename for lookup.

ROLE_OF: dict[str, Role | None] = {
    # Frame — through-columns (tower-global, no floor prefix)
    **{f"outer_vert_{k}": Role.PRIMARY_STRUCTURE for k in range(6)},
    # Frame — per-floor horizontals
    **{f"outer_horiz_{k}": Role.PRIMARY_STRUCTURE for k in range(6)},
    **{f"inner_horiz_{k}": Role.PRIMARY_STRUCTURE for k in range(6)},
    **{f"radial_{k}": Role.PRIMARY_STRUCTURE for k in range(6)},
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
    # ``drive_shaft`` is obsolete (Lego falls through the column axis;
    # see NEXT_STEPS.md "Base-floor drive train"). Tag ``None`` until
    # it's removed from the assembly.
    "drive_shaft": None,
    "motor_mount": Role.SECONDARY_STRUCTURE,
}


# --- m05 module tags --------------------------------------------------

MODULE_OF: dict[str, Module | None] = {
    name: (Module.DIVERTER if role is not None else None)
    for name, role in ROLE_OF.items()
}


# --- Material lookup --------------------------------------------------
#
# Material is independent of the WORKSHOP/BAUHAUS swap — it's a
# physical property of the part, not a render-axis tag. Kept in this
# file for one-stop lookup at every ``.add(...)`` site.

MATERIAL_OF: dict[str, str | None] = {
    # 2020 extrusions — anodized aluminium
    **{f"outer_vert_{k}": "aluminium_anodized" for k in range(6)},
    **{f"outer_horiz_{k}": "aluminium_anodized" for k in range(6)},
    **{f"inner_horiz_{k}": "aluminium_anodized" for k in range(6)},
    **{f"radial_{k}": "aluminium_anodized" for k in range(6)},
    # Printed brackets / mechanism / mount — PLA matte
    **{f"outer_vertex_bracket_{k}": "pla_matte" for k in range(6)},
    **{f"inner_vertex_bracket_{k}": "pla_matte" for k in range(6)},
    "bearing_spider": "pla_matte",
    "ring_bearing": "pla_matte",
    "column": "pla_matte",
    "ramp": "pla_matte",
    "trap_closed": "pla_matte",
    "trap_open": "pla_matte",
    "lower_trap_open": "pla_matte",
    "motor_mount": "pla_matte",
    **{name: "pla_matte" for name in _basket_basenames()},
    # Hardware
    "hinge_pin": "steel",
    "motor": "cast_aluminium_dark",
    "drive_shaft": "steel",
}


# --- Bound DesignSystems ----------------------------------------------

WORKSHOP = DesignSystem(name="workshop", tag_of=ROLE_OF, palette=WORKSHOP_COLORS)
BAUHAUS = DesignSystem(name="bauhaus", tag_of=MODULE_OF, palette=BAUHAUS_COLORS)

DESIGN_SYSTEM: DesignSystem = (
    {"workshop": WORKSHOP, "bauhaus": BAUHAUS}[DESIGN_SYSTEM_NAME]
)


# --- Instance-name → basename helpers ---------------------------------

def basename_of(part_name: str) -> str:
    """Strip the ``f{idx}_{role}__`` floor prefix from a m05 instance
    name. Tower-global parts (``outer_vert_k``) and stand-ins used by
    ``single_floor.py`` carry no prefix and pass through unchanged.
    """
    sep = "__"
    return part_name.split(sep, 1)[1] if sep in part_name else part_name


def color_of(part_name: str) -> Color | None:
    """Active design system's color for a m05 part instance.

    Accepts either a basename or a floor-prefixed instance name. Falls
    through to ``DESIGN_SYSTEM.color_of(...)``, which raises
    ``KeyError`` for an unknown basename — module-level coverage
    assertions in ``assembly.py`` / ``single_floor.py`` catch that
    earlier, but the explicit raise prevents silent off-palette
    renders.
    """
    return DESIGN_SYSTEM.color_of(basename_of(part_name))


def material_of(part_name: str) -> str | None:
    """Material for a m05 part instance. Same instance-or-basename
    contract as ``color_of``. Raises ``KeyError`` for an unknown
    basename so a missing entry surfaces loudly.
    """
    bn = basename_of(part_name)
    if bn not in MATERIAL_OF:
        raise KeyError(
            f"part '{part_name}' (basename '{bn}') has no entry in "
            f"MATERIAL_OF — add one in cad/m05_diverter/design_systems.py"
        )
    return MATERIAL_OF[bn]
