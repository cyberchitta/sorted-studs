"""Shared design-system schema for sorted-studs.

A "design system" is a binding from part name to color via some axis.
The same architecture supports multiple axes — functional role,
module, floor, sub-unit, etc. — by parameterising the axis enum and
its per-part tagging dict.

Every concrete design system is a
``DesignSystem(name, axis, tag_of, palette)``:

  * ``tag_of`` — every named printed / extrusion part maps to an axis
    value (or ``None`` for parts outside the design system: hardware,
    optical glass, clear acrylic).
  * ``palette`` — each axis value maps to a ``Color``.

Two axes are wired up:

  * ``Axis.ROLE`` — what the part *does*. ``WORKSHOP`` is the
    reference: graphite skeleton, signal-yellow mechanism, matte-black
    optical envelope, warm-off-white consumables.
  * ``Axis.MODULE`` — which subsystem the part lives in. ``BAUHAUS``
    is the only one: saturated primary-color wayfinding, m02 yellow,
    m03 red, m05 blue.

This file holds only the axis-agnostic machinery — enums, palettes,
the dataclass, the palette registry, and the ``paint`` walk.
**Per-module tag dicts live in each module's own
``cad/m0X_*/design_systems.py``** so that a rename in a module's
assembly tree edits the same module's tag dict — no cross-directory
drift. Cross-module composition needs no tag-dict merging: the top
level composes units as subtrees, and each unit root paints its own
subtree before handing it up.

**A design system is a value, not module state.** There is no active
selection here to edit and no environment variable read at import.
Builders take a ``schema`` name, bind their module's tag dicts to it,
and paint on the way out — pure functions from schema to assembly. Entry
points (render drivers, export scripts) resolve the name, via
``schema_from_env`` if they want the env layers. See ``paint``.
"""

from __future__ import annotations

import os
from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import Enum

from build123d import Color

from cad_khana.mechanism.assembly import Assembly, PlacedPart


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


# --- Which tag dict a palette keys on ---------------------------------

class Axis(Enum):
    """Which per-module tag dict a palette's keys are drawn from.

    Carried on the ``DesignSystem`` so axis-dependent decisions can
    dispatch on it explicitly instead of probing the palette for a
    ``Role`` key — see ``neutral_of``.
    """

    ROLE = "role"        # keys are ``Role``   — what the part does
    MODULE = "module"    # keys are ``Module`` — which subsystem it's in


# --- DesignSystem -----------------------------------------------------

@dataclass(frozen=True)
class DesignSystem:
    """A binding from part name → color via one named axis.

    ``tag_of`` covers every named printed / extrusion part; ``None``
    means the part is outside the system (hardware, optical glass,
    acrylic). ``palette`` maps each axis value to its color.
    """

    name: str
    axis: Axis
    tag_of: dict[str, Enum | None]
    palette: dict[Enum, Color]

    def color_of(self, part_name: str) -> Color | None:
        """Active color for a named part. ``None`` falls through to the
        material's intrinsic color. Raises ``KeyError`` for an untagged
        part — ``paint`` checks coverage over a whole assembly and
        reports every offender at once, but the explicit raise prevents
        a silent off-palette render for anyone calling this directly.
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

# CyberChitta brand palettes — warm gold/bronze/earth tones from the
# supramental-gold theme tokens (www.cyberchitta.cc).
#
# Paired light/dark variants: the subsite serves whichever matches the
# visitor's ``prefers-color-scheme``, so every entry must hold an edge
# against its target background. Sticking to L ≈ 0.40–0.70 also lets
# either palette degrade gracefully on the opposite background.
#
# The original ``cyberchitta`` palette mapped SECONDARY_STRUCTURE and
# CONSUMABLE to ``--color-base-300`` / ``--color-base-100`` — which are
# *page-background* tokens and by design blend into the page. The
# retuned values below use mid-luminance neutrals and a clay accent
# (``--color-link``) so painted parts read against the page.
CYBERCHITTA_LIGHT_COLORS: dict[Enum, Color] = {
    Role.PRIMARY_STRUCTURE:   _srgb(0.290, 0.271, 0.251),  # neutral  #4a4540 — warm charcoal
    Role.SECONDARY_STRUCTURE: _srgb(0.478, 0.443, 0.416),  # mid-grey #7a716a — warm mid grey
    Role.MECHANISM:           _srgb(0.824, 0.667, 0.196),  # gold-dec #d2aa32 — bright gold
    Role.OPTICAL_ENVELOPE:    _srgb(0.804, 0.373, 0.176),  # bronze   #cd5f2d — warm bronze
    Role.CONSUMABLE:          _srgb(0.722, 0.329, 0.125),  # link     #b85420 — deep clay
}

# Mirrors CYBERCHITTA_LIGHT across the luminance band:
#   neutral / mid-grey shift to lighter warm greys (foreground on dark);
#   gold and bronze use the dark-theme variants (slightly brighter so
#   they pop against ``--color-base-100`` #1e1c18).
# CONSUMABLE re-uses ``--color-link`` dark-theme value, which equals
# bronze-dark in this theme — fine for m05 because OPTICAL_ENVELOPE is
# unused there. If a future module mixes both, split them.
CYBERCHITTA_DARK_COLORS: dict[Enum, Color] = {
    Role.PRIMARY_STRUCTURE:   _srgb(0.722, 0.690, 0.643),  # neutral  #b8b0a4 — light warm grey
    Role.SECONDARY_STRUCTURE: _srgb(0.541, 0.510, 0.471),  # mid-grey #8a8278 — slightly brighter mid grey
    Role.MECHANISM:           _srgb(0.831, 0.659, 0.227),  # gold     #d4a83a — gold (dark theme, --color-gold/--color-primary)
    Role.OPTICAL_ENVELOPE:    _srgb(0.878, 0.502, 0.290),  # bronze   #e0804a — bronze (dark theme)
    Role.CONSUMABLE:          _srgb(0.878, 0.502, 0.290),  # link     #e0804a — clay (dark theme)
}


# --- Palette registry -------------------------------------------------

# One entry per palette: which tag dict its keys come from, and the
# colors. ``bind_all`` and ``PALETTE_NAMES`` both derive from this, so
# adding a palette is one line here and nothing else.
PALETTES: dict[str, tuple[Axis, dict[Enum, Color]]] = {
    "workshop":          (Axis.ROLE,   WORKSHOP_COLORS),
    "bauhaus":           (Axis.MODULE, BAUHAUS_COLORS),
    "clean":             (Axis.ROLE,   CLEAN_COLORS),
    "midnight":          (Axis.ROLE,   MIDNIGHT_COLORS),
    "signal":            (Axis.ROLE,   SIGNAL_COLORS),
    "cyberchitta-light": (Axis.ROLE,   CYBERCHITTA_LIGHT_COLORS),
    "cyberchitta-dark":  (Axis.ROLE,   CYBERCHITTA_DARK_COLORS),
}

PALETTE_NAMES: tuple[str, ...] = tuple(PALETTES)

# The schema a builder uses when its caller doesn't name one, so
# ``khana check <unit>/assembly.py`` resolves without any ambient
# state. Entry points that want the env layers call
# ``schema_from_env`` and pass the result down explicitly.
DEFAULT_SCHEMA: str = "cyberchitta-light"


def schema_from_env(module: str | None = None) -> str:
    """Resolve a palette name from the environment — for **entry
    points only**: a render driver, an export script, a CLI.

    Layers, highest first: ``SORTED_STUDS_DS_<MODULE>`` (e.g.
    ``SORTED_STUDS_DS_M03``, one module's own switch) → the global
    ``SORTED_STUDS_DS`` → ``DEFAULT_SCHEMA``.

    Declaration modules must not call this. A builder that reads the
    environment is not a function of its arguments, which is the whole
    defect this module's rewrite retired: the schema used to be
    captured into a module global at import, so switching it meant
    evicting ``cad.m0*`` from ``sys.modules`` and re-importing
    (``cad/render.py``) or spawning a subprocess per schema
    (``m03_scanner/render_all_ds.py``). Take a ``schema`` parameter and
    let your caller decide where it came from.
    """
    if module:
        per_module = os.environ.get(f"SORTED_STUDS_DS_{module.upper()}")
        if per_module:
            return per_module
    return os.environ.get("SORTED_STUDS_DS", DEFAULT_SCHEMA)


# --- Bind every palette for a module ----------------------------------

def bind_all(
    role_of: dict[str, Role | None],
    module_of: dict[str, Module | None],
) -> dict[str, DesignSystem]:
    """Bind every palette in ``PALETTES`` to a module's tag dicts.

    Each module's ``design_systems.py`` declares two tag dicts
    (``ROLE_OF`` and ``MODULE_OF``) and exposes a
    ``design_system(schema)`` lookup over the result. Role-keyed
    palettes consume ``role_of``; the module-keyed ``bauhaus`` palette
    consumes ``module_of``. Adding a palette is one line in
    ``PALETTES`` — modules pick it up with no edit of their own.
    """
    tags: dict[Axis, dict[str, Enum | None]] = {
        Axis.ROLE: dict(role_of),
        Axis.MODULE: dict(module_of),
    }
    return {
        name: DesignSystem(name, axis, tags[axis], colors)
        for name, (axis, colors) in PALETTES.items()
    }


def lookup(palettes: dict[str, DesignSystem], schema: str) -> DesignSystem:
    """``palettes[schema]`` with an error that names the alternatives.

    A typo'd schema otherwise surfaces as a bare ``KeyError`` inside a
    module's ``design_system()`` with no hint of what is valid.
    """
    try:
        return palettes[schema]
    except KeyError:
        raise KeyError(
            f"unknown design system {schema!r} — have {list(PALETTE_NAMES)}"
        ) from None


# --- Luminance blend helper -------------------------------------------

def blend(color: Color, t: float) -> Color:
    """Blend ``color`` toward black (``t < 0``) or white (``t > 0``) by
    fraction ``|t|``. Used to derive monotonic luminance ramps off a
    palette base — e.g. m05's basket-size colouring picks a darker
    shade for the smallest basket and a lighter shade for the largest.
    Alpha passes through unchanged.
    """
    r, g, b, a = tuple(color)
    target = 0.0 if t < 0 else 1.0
    f = abs(t)
    return Color(
        r + (target - r) * f,
        g + (target - g) * f,
        b + (target - b) * f,
        a,
    )


# --- Painting an assembly ---------------------------------------------
#
# Colour is applied to a *finished* assembly rather than set at each
# ``with_part`` site. Builders stay palette-free: they take no colour
# argument, import nothing from a design system, and produce the same
# geometry whatever the schema. The unit root — the level that owns the
# module's tag dict — paints its own subtree on the way out.
#
# Why a walk rather than a parameter threaded down every builder: the
# tag dicts are keyed on **leaf part name**, and inside the tree
# ``PlacedPart.name`` is already the leaf (``placed_parts`` is what
# qualifies it into a dotted path). So a recursive walk has exactly the
# key it needs at every level, with no plumbing through the builders in
# between.

ColorFor = Callable[[DesignSystem, str], Color | None]


def _by_tag(ds: DesignSystem, leaf: str) -> Color | None:
    return ds.color_of(leaf)


def paint(
    assembly: Assembly,
    ds: DesignSystem,
    *,
    skip: tuple[str, ...] = (),
    color_for: ColorFor = _by_tag,
) -> Assembly:
    """Return ``assembly`` with every part's ``color`` set from ``ds``.

    ``skip`` names **root-level sub-assemblies** to leave untouched —
    for a foreign unit composed as a subtree whose own unit root has
    already painted it against its own tag dict (the m02 chain's
    ``s1``). Without it, this walk would look the singulator's part
    names up in the chain's tag dict and report them all as untagged.

    ``color_for`` overrides the per-part lookup for a module whose
    colour isn't a plain tag read — m05 ramps basket luminance off
    ``n_size``. It is called only for parts the tag dict covers, so a
    coverage gap is still reported rather than silently absorbed.

    Every part whose leaf name is missing from ``ds.tag_of`` is
    collected, and **one** ``KeyError`` names all of them with their
    tree paths. That replaces the four hand-rolled module-level
    coverage asserts that walked ``placed_parts`` at import time: same
    guarantee (nothing renders off-palette), one error listing every
    offender instead of a bare assert per module, and it fires when the
    unit is *built* rather than when it is imported.
    """
    untagged: list[str] = []
    out = _painted(assembly, ds, frozenset(skip), color_for, untagged, "")
    if untagged:
        raise KeyError(
            f"{len(untagged)} part(s) have no tag in design system "
            f"'{ds.name}' — add them to the owning module's "
            f"design_systems.py tag dict: {sorted(untagged)}"
        )
    return out


def _painted(
    assembly: Assembly,
    ds: DesignSystem,
    skip: frozenset[str],
    color_for: ColorFor,
    untagged: list[str],
    prefix: str,
) -> Assembly:
    parts = tuple(
        _paint_part(p, ds, color_for, untagged, prefix)
        for p in assembly.parts
    )
    subs = tuple(
        s if s.name in skip else replace(
            s,
            assembly=_painted(
                s.assembly, ds, skip, color_for, untagged,
                f"{prefix}{s.name}.",
            ),
        )
        for s in assembly.subassemblies
    )
    return replace(assembly, parts=parts, subassemblies=subs)


def _paint_part(
    p: PlacedPart,
    ds: DesignSystem,
    color_for: ColorFor,
    untagged: list[str],
    prefix: str,
) -> PlacedPart:
    """One part's colour, or the part unchanged.

    A part tagged ``None`` is *outside* the design system, so the
    system has nothing to say about it and the builder's own colour
    stands — that is what ``color_of``'s "falls through to the
    material's intrinsic color" means. The one part that relies on it
    is m03's ``acrylic``, which carries a translucent baseColor with a
    real alpha channel so the shape-truth GLB renders through it;
    assigning ``None`` over that would silently make it opaque.
    """
    if p.name not in ds.tag_of:
        untagged.append(f"{prefix}{p.name}")
        return p
    color = color_for(ds, p.name)
    return p if color is None else replace(p, color=color)


# --- GLB-export neutral fill ------------------------------------------

# Warm mid-grey, luminance-matched to the role palettes'
# SECONDARY_STRUCTURE band so a module-axis GLB reads like the others.
_MODULE_AXIS_NEUTRAL: Color = _srgb(0.478, 0.443, 0.416)


def neutral_of(ds: DesignSystem) -> Color:
    """The palette's mid-luminance neutral, for ``fill_palette_neutrals``.

    Owns the per-axis decision that four ``export_glb*.py`` call sites
    used to make by indexing ``palette[Role.SECONDARY_STRUCTURE]``
    directly — a reach that raises ``KeyError`` under the module-keyed
    ``bauhaus`` palette, which has no ``Role`` keys. (Same defect class
    as the ``basket_color_of`` crash fixed 2026-07-24; this was the
    surviving instance, tracked as "Latent: GLB-export neutral fill
    hardcodes the role axis" in ``_notes/state/cad-top-level.md``.)

    On the role axis the answer is the printed-bracket shade. On the
    module axis there is none: a wayfinding palette encodes *which
    subsystem*, not *how light*, and picking one module's colour would
    tint every untagged part with a subsystem it doesn't belong to.
    Hence a fixed warm mid-grey — which is what "mid-luminance neutral"
    means once the axis carries no luminance information.
    """
    if ds.axis is Axis.ROLE:
        return ds.palette[Role.SECONDARY_STRUCTURE]
    return _MODULE_AXIS_NEUTRAL


def fill_palette_neutrals(assembly: Assembly, neutral: Color) -> Assembly:
    """Recursively assign ``neutral`` to every PlacedPart whose ``color``
    is ``None``. Used by static GLB exports only.

    cad-khana's ``export_glb`` is the geometry-truth path: it writes
    only per-part ``Color`` and ignores ``material``. Parts tagged
    ``None`` in a module's ``ROLE_OF`` (clear acrylic, glass, FR4, …)
    therefore default to glTF's flat white baseColor — which has no
    contrast against ``--color-base-100`` ``#f7f6f4`` in the light
    palette. Filling with the palette's mid-luminance neutral
    (``neutral_of``) reads cleanly against both themes.

    Photo-real chitra-cad renders are unaffected: they go through a
    different driver (``cad/render.py``) that builds its own Assembly
    per palette pass, so this in-place-style return value doesn't leak
    into Cycles' material catalog.
    """
    new_parts = tuple(
        replace(p, color=neutral) if p.color is None else p
        for p in assembly.parts
    )
    new_subs = tuple(
        replace(s, assembly=fill_palette_neutrals(s.assembly, neutral))
        for s in assembly.subassemblies
    )
    return replace(assembly, parts=new_parts, subassemblies=new_subs)
