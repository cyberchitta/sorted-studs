"""m02 c-channel singulator — per-module design-system tags.

Schema (``Role``, ``Module``, ``DesignSystem``, palettes, the palette
registry, ``paint``) lives in ``cad/design_systems.py``; this file
holds only m02's tag dicts and the ``design_system(schema)`` lookup
that ``assembly.build_m02`` binds when it paints. Sub-assembly
builders import nothing from here — they emit no colour at all.
Material is a physical property, not a render axis — see
``materials.py``.

m02's named parts: ``rotor`` (rotates, MECHANISM), ``stator_0..2``
(the liner's thirds — the brick-contact surface, CONSUMABLE),
``exit_floor`` (the tongue under the exit arc, SECONDARY_STRUCTURE),
``bowl_0..2`` (the thirds of the floor and skirt the rotor bearing,
motor and stator liner sit on, SECONDARY_STRUCTURE),
``pinion`` (rotates, MECHANISM), and two bought parts outside the
design system — ``bearing`` (aluminium turntable) and ``motor``
(NEMA-17) — and, on a stage whose liner the exit opening cuts
through, ``arc_screw_pos`` / ``arc_screw_neg`` (the bought screws
fastening the two arcs' free ends), which keep their material
intrinsics.
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


# --- m02 role tags ----------------------------------------------------

ROLE_OF: dict[str, Role | None] = {
    "rotor":  Role.MECHANISM,
    # `stator` is the structural shell *and* the brick-contact surface.
    # CONSUMABLE marks what the bricks touch, as on m05's baskets — and
    # since the S3 hopper wall the liner is the singulator's largest
    # visible surface, so it carries the machine's accent colour. Not a
    # claim that stators are swapped in operation.
    "stator_0":   Role.CONSUMABLE,
    "stator_1":   Role.CONSUMABLE,
    "stator_2":   Role.CONSUMABLE,
    "exit_floor": Role.SECONDARY_STRUCTURE,
    "bowl_0":     Role.SECONDARY_STRUCTURE,
    "bowl_1":     Role.SECONDARY_STRUCTURE,
    "bowl_2":     Role.SECONDARY_STRUCTURE,
    "pinion":     Role.MECHANISM,
    "bearing":    None,                      # bought turntable — outside palette
    "motor":      None,                      # NEMA-17 cast body — outside palette
    "arc_screw_pos": None,                   # bought M3 countersunk screw — outside palette
    "arc_screw_neg": None,
}


# --- m02 module tags --------------------------------------------------

MODULE_OF: dict[str, Module | None] = {
    name: (Module.SINGULATOR if role is not None else None)
    for name, role in ROLE_OF.items()
}


# --- Bound design systems ---------------------------------------------

PALETTES: dict[str, DesignSystem] = bind_all(ROLE_OF, MODULE_OF)


# --- Liner shade -------------------------------------------------------
#
# The liner shares CONSUMABLE with m05's baskets, whose colour is a
# luminance ramp off the same base (``m05_diverter/design_systems.py``,
# ``_BASKET_TINT_BY_N``) — the unshifted base sits mid-ramp, between the
# N=9 and N=12 baskets. A luminance step would land on some basket size,
# so the liner moves in hue instead: part-way toward the rotor's own
# colour, which no basket carries. Read through ``color_of`` so it stays
# axis-agnostic (under a module-keyed palette both are the same colour
# and the mix is a no-op).
LINER_TOWARD_ROTOR = 0.30


def _mix(a: Color, b: Color, f: float) -> Color:
    """``a`` moved fraction ``f`` of the way toward ``b``; alpha from ``a``."""
    ra, ga, ba, aa = tuple(a)
    rb, gb, bb, _ = tuple(b)
    return Color(ra + (rb - ra) * f, ga + (gb - ga) * f, ba + (bb - ba) * f, aa)


def color_for(ds: DesignSystem, leaf: str) -> Color | None:
    """``paint``'s per-part hook for m02 — a plain tag read except for
    the liner thirds, which take the shade above."""
    if leaf.startswith("stator_"):
        return _mix(ds.color_of(leaf), ds.color_of("rotor"), LINER_TOWARD_ROTOR)
    return ds.color_of(leaf)


def design_system(schema: str = DEFAULT_SCHEMA) -> DesignSystem:
    """m02's tag dicts bound to ``schema``.

    Called by ``assembly.build_m02`` to paint its subtree; there is no
    module-global "active" binding, so nothing here reads the
    environment at import.
    """
    return lookup(PALETTES, schema)
