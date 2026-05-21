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

import os
from dataclasses import dataclass
from enum import Enum

from build123d import Color


def _srgb(r: float, g: float, b: float) -> Color:
    """Express a colour in sRGB 0–1 — the way CSS, web pickers, and
    design tools think about colour.

    The returned ``build123d.Color`` carries the sRGB-encoded values
    *as-is*; consumers that need linear light (Blender Cycles via
    ``chitra-cad.scene._color_to_rgb``) convert at their input
    boundary. Consumers that emit sRGB (cad-khana's glTF export, which
    labels OCP colours as ``Quantity_TOC_sRGB``) use the values
    directly — double-linearization would crush them to near-black,
    which is what was happening before this helper stopped doing the
    conversion itself.
    """
    return Color(r, g, b)


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
    Role.PRIMARY_STRUCTURE:   _srgb(0.18, 0.18, 0.20),  # graphite anodized
    Role.SECONDARY_STRUCTURE: _srgb(0.18, 0.18, 0.20),  # graphite PLA
    Role.MECHANISM:           _srgb(0.95, 0.78, 0.10),  # signal yellow
    Role.OPTICAL_ENVELOPE:    _srgb(0.04, 0.04, 0.04),  # matte black
    Role.CONSUMABLE:          _srgb(0.92, 0.90, 0.84),  # warm off-white
}

BAUHAUS_COLORS: dict[Enum, Color] = {
    Module.SINGULATOR: _srgb(1.00, 0.85, 0.05),         # primary yellow
    Module.SCANNER:    _srgb(0.85, 0.10, 0.10),         # primary red
    Module.DIVERTER:   _srgb(0.08, 0.30, 0.75),         # primary blue
}

CLEAN_COLORS: dict[Enum, Color] = {
    Role.PRIMARY_STRUCTURE:   _srgb(0.72, 0.70, 0.67),  # warm light grey (natural aluminium)
    Role.SECONDARY_STRUCTURE: _srgb(0.94, 0.91, 0.86),  # cream white printed
    Role.MECHANISM:           _srgb(0.80, 0.48, 0.10),  # warm amber
    Role.OPTICAL_ENVELOPE:    _srgb(0.10, 0.14, 0.32),  # deep navy
    Role.CONSUMABLE:          _srgb(0.96, 0.94, 0.90),  # off-white
}

MIDNIGHT_COLORS: dict[Enum, Color] = {
    Role.PRIMARY_STRUCTURE:   _srgb(0.08, 0.10, 0.14),  # near-black blue-grey
    Role.SECONDARY_STRUCTURE: _srgb(0.13, 0.15, 0.21),  # dark slate
    Role.MECHANISM:           _srgb(0.12, 0.42, 0.92),  # electric blue
    Role.OPTICAL_ENVELOPE:    _srgb(0.02, 0.02, 0.03),  # pure black
    Role.CONSUMABLE:          _srgb(0.76, 0.82, 0.94),  # pale blue-white
}

SIGNAL_COLORS: dict[Enum, Color] = {
    Role.PRIMARY_STRUCTURE:   _srgb(0.15, 0.15, 0.15),  # near-black charcoal
    Role.SECONDARY_STRUCTURE: _srgb(0.22, 0.22, 0.22),  # dark grey
    Role.MECHANISM:           _srgb(0.95, 0.40, 0.02),  # hi-vis orange
    Role.OPTICAL_ENVELOPE:    _srgb(0.72, 0.06, 0.06),  # signal red
    Role.CONSUMABLE:          _srgb(0.97, 0.96, 0.93),  # safety white
}

# CyberChitta brand palette — warm gold/bronze/earth tones from
# www.cyberchitta.cc light theme (--color-neutral / base-300 / gold-decorative / bronze / base-100)
CYBERCHITTA_COLORS: dict[Enum, Color] = {
    Role.PRIMARY_STRUCTURE:   _srgb(0.290, 0.271, 0.251),  # neutral  #4a4540 — warm charcoal
    Role.SECONDARY_STRUCTURE: _srgb(0.878, 0.867, 0.847),  # base-300 #e0ddd8 — warm cream-grey
    Role.MECHANISM:           _srgb(0.824, 0.667, 0.196),  # gold-dec #d2aa32 — bright gold
    Role.OPTICAL_ENVELOPE:    _srgb(0.804, 0.373, 0.176),  # bronze   #cd5f2d — warm bronze
    Role.CONSUMABLE:          _srgb(0.969, 0.965, 0.957),  # base-100 #f7f6f4 — warm off-white
}


# --- Active selection -------------------------------------------------

# Repo-wide default palette. Resolves in three layers, highest-priority
# first:
#
#   1. Per-module env var ``SORTED_STUDS_DS_<MODULE>`` (e.g.
#      ``SORTED_STUDS_DS_M03``) — a module's own switch, useful for
#      one-pane subsite refreshes where the rest of the assemblies
#      stay on the global palette.
#   2. Global env var ``SORTED_STUDS_DS`` — captured here into
#      ``DESIGN_SYSTEM_NAME`` at import time.
#   3. The fallback default below.
#
# Each module's ``design_systems.py`` calls ``resolve_name("M0X")`` to
# pick its palette; layers 2 and 3 share the ``DESIGN_SYSTEM_NAME``
# state (so ``cad/render.py``'s mutate-and-reload schema-switch dance
# keeps working). Valid palette names match the keys returned by
# ``bind_all`` below — keep them in sync when adding a palette.
DESIGN_SYSTEM_NAME: str = os.environ.get("SORTED_STUDS_DS", "cyberchitta")


def resolve_name(module: str | None = None) -> str:
    """Resolve the active palette name for ``module`` (e.g. ``"M03"``).

    Layered per the docstring above ``DESIGN_SYSTEM_NAME``:
    per-module env var → global env var (captured into
    ``DESIGN_SYSTEM_NAME``) → repo default.
    """
    if module:
        per_module = os.environ.get(f"SORTED_STUDS_DS_{module.upper()}")
        if per_module:
            return per_module
    return DESIGN_SYSTEM_NAME


# --- Bind every palette for a module ----------------------------------

def bind_all(
    role_of: dict[str, Role | None],
    module_of: dict[str, Module | None],
) -> dict[str, DesignSystem]:
    """Bind every known palette to a module's tag dicts.

    Each module's ``design_systems.py`` declares two tag dicts
    (``ROLE_OF`` and ``MODULE_OF``) and then picks the active palette
    with ``bind_all(ROLE_OF, MODULE_OF)[DESIGN_SYSTEM_NAME]``. Role-
    keyed palettes (workshop / clean / midnight / signal / cyberchitta)
    consume ``role_of``; the module-keyed ``bauhaus`` palette consumes
    ``module_of``. Adding a new palette is one line here — modules
    pick it up automatically on the next import.
    """
    return {
        "workshop":    DesignSystem("workshop",    role_of,   WORKSHOP_COLORS),
        "bauhaus":     DesignSystem("bauhaus",     module_of, BAUHAUS_COLORS),
        "clean":       DesignSystem("clean",       role_of,   CLEAN_COLORS),
        "midnight":    DesignSystem("midnight",    role_of,   MIDNIGHT_COLORS),
        "signal":      DesignSystem("signal",      role_of,   SIGNAL_COLORS),
        "cyberchitta": DesignSystem("cyberchitta", role_of,   CYBERCHITTA_COLORS),
    }


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
