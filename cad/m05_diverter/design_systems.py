"""m05 diverter — per-module design-system tags.

Schema (``Role``, ``Module``, ``DesignSystem``, palettes,
``DESIGN_SYSTEM_NAME`` switch, ``compose`` helper) lives in
``cad/design_systems.py``; this file holds only m05's tag dicts and
binds the module-local ``DESIGN_SYSTEM`` + ``color_of`` shim that
``assembly.py`` and ``single_floor.py`` use at every ``.with_part(...)``
site. Material is a physical property, not a render axis — see
``materials.py``.

``basename_of`` lives here because both ``color_of`` and
``materials.material_of`` need it to invert the floor-prefix naming
convention; ``materials.py`` imports it from this file.

Two complications relative to m02 / m03:

* **Floor-prefixed instance names.** ``assembly.py`` adds parts as
  ``f{idx}_{role}__<basename>`` so the four-floor stack stays
  collision-free. ``basename_of(...)`` strips that prefix; tower-
  global parts (``outer_vert_k`` / ``inner_vert_k``) carry no
  prefix and pass through.

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
    DesignSystem,
    Module,
    Role,
    bind_all,
    resolve_name,
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
    # ``drive_shaft`` is obsolete (Lego falls through the column axis;
    # see NEXT_STEPS.md "Base-floor drive train"). Tag ``None`` until
    # it's removed from the assembly.
    "drive_shaft": None,
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
    """Strip the ``f{idx}_{role}__`` floor prefix from a m05 instance
    name. Tower-global parts (``outer_vert_k`` / ``inner_vert_k``)
    and stand-ins used by ``single_floor.py`` carry no prefix and
    pass through unchanged.
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
