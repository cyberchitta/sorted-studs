"""Shared design-system schema for sorted-studs.

A "design system" is a binding from part name to color via some axis.
The same architecture supports multiple axes — functional role,
module, floor, sub-unit, etc. — by parameterising the axis enum and
its per-part tagging dict.

Every concrete design system is a ``DesignSystem(name, tag_of, palette)``:

  * ``tag_of`` — every named printed / extrusion part maps to an axis
    value (or ``None`` for parts outside the design system: hardware,
    optical glass, clear acrylic).
  * ``palette`` — each axis value maps to a ``Color``.

Two are wired up:

  * ``WORKSHOP`` — axis = functional role (what the part *does*).
    Graphite skeleton, signal-yellow mechanism, matte-black optical
    envelope, warm-off-white consumables.
  * ``BAUHAUS``  — axis = module (which subsystem the part lives in).
    Saturated primary-color wayfinding: m02 yellow, m03 red, m05 blue.

This file holds only the axis-agnostic machinery — enums, palettes,
the dataclass, the active-selection switch, and the ``compose``
helper. **Per-module tag dicts live in each module's own
``cad/m0X_*/design_systems.py``** so that a rename in a module's
assembly tree edits the same module's tag dict — no cross-directory
drift.

Active selection: edit ``DESIGN_SYSTEM_NAME`` below to swap the whole
repo's palette. Each module's ``DESIGN_SYSTEM`` reads this switch on
import; module-local coverage assertions catch a swap to a palette
that's missing a tag.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from build123d import Color


# --- Functional-role axis ---------------------------------------------

class Role(Enum):
    """Functional role of a part — what it does in the machine."""

    PRIMARY_STRUCTURE = "primary_structure"     # 2020 extrusions
    SECONDARY_STRUCTURE = "secondary_structure" # printed brackets / mounts
    MECHANISM = "mechanism"                     # rotating / hinged / pushing
    OPTICAL_ENVELOPE = "optical_envelope"       # inside camera FOV
    CONSUMABLE = "consumable"                   # holds bricks; swapped often


# --- Module axis ------------------------------------------------------

class Module(Enum):
    """Which top-level subsystem a part belongs to."""

    SINGULATOR = "singulator"   # m02
    SCANNER = "scanner"         # m03
    DIVERTER = "diverter"       # m05


# --- DesignSystem -----------------------------------------------------

@dataclass(frozen=True)
class DesignSystem:
    """A binding from part name → color via one named axis.

    ``tag_of`` covers every named printed / extrusion part; ``None``
    means the part is outside the system (hardware, optical glass,
    acrylic). ``palette`` maps each axis value to its color.
    """

    name: str
    tag_of: dict[str, Enum | None]
    palette: dict[Enum, Color]

    def color_of(self, part_name: str) -> Color | None:
        """Active color for a named part. ``None`` falls through to the
        material's intrinsic color. Raises ``KeyError`` for an untagged
        part — module-level coverage assertions catch this earlier, but
        the explicit raise prevents silent off-palette renders.
        """
        if part_name not in self.tag_of:
            raise KeyError(
                f"part '{part_name}' has no tag in design system "
                f"'{self.name}' — add it to the relevant module's "
                f"design_systems.py tag dict"
            )
        tag = self.tag_of[part_name]
        return self.palette[tag] if tag is not None else None


# --- Palettes ---------------------------------------------------------

WORKSHOP_COLORS: dict[Enum, Color] = {
    Role.PRIMARY_STRUCTURE:   Color(0.18, 0.18, 0.20),  # graphite anodized
    Role.SECONDARY_STRUCTURE: Color(0.18, 0.18, 0.20),  # graphite PLA
    Role.MECHANISM:           Color(0.95, 0.78, 0.10),  # signal yellow
    Role.OPTICAL_ENVELOPE:    Color(0.04, 0.04, 0.04),  # matte black
    Role.CONSUMABLE:          Color(0.92, 0.90, 0.84),  # warm off-white
}

BAUHAUS_COLORS: dict[Enum, Color] = {
    Module.SINGULATOR: Color(1.00, 0.85, 0.05),         # primary yellow
    Module.SCANNER:    Color(0.85, 0.10, 0.10),         # primary red
    Module.DIVERTER:   Color(0.08, 0.30, 0.75),         # primary blue
}


# --- Active selection -------------------------------------------------

# Single switch for the whole repo. Each module's ``design_systems.py``
# reads this on import to bind its local ``DESIGN_SYSTEM``.
DESIGN_SYSTEM_NAME: str = "workshop"   # "workshop" | "bauhaus"


# --- compose helper ---------------------------------------------------

def compose(
    *pairs: dict[str, Enum | None] | tuple[str, dict[str, Enum | None]],
) -> dict[str, Enum | None]:
    """Merge per-module tag dicts into one. Each arg is either a bare
    dict (no prefix) or a ``(prefix, dict)`` tuple. Raises ``KeyError``
    on collision after prefixing — collisions are bugs.

    Used by a future full-machine render driver:

        merged = compose(
            ("m02__", M02_ROLES),
            ("m03__", M03_ROLES),
            ("m05__", M05_ROLES),
        )
        DesignSystem("workshop", merged, WORKSHOP_COLORS)

    For module-local renders, per-module files don't go through
    ``compose`` — they bind their tag dict directly.
    """
    merged: dict[str, Enum | None] = {}
    for entry in pairs:
        if isinstance(entry, tuple):
            prefix, tag_dict = entry
        else:
            prefix, tag_dict = "", entry
        for name, tag in tag_dict.items():
            full = f"{prefix}{name}"
            if full in merged:
                raise KeyError(
                    f"design-system tag collision: '{full}' appears in "
                    f"more than one input dict"
                )
            merged[full] = tag
    return merged
