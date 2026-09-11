"""m02 c-channel singulator — per-part materials.

Material is a physical property of the part, independent of the
WORKSHOP/BAUHAUS render-axis swap in ``design_systems.py``.
Centralised here so a material change is one edit; every
``.with_part(...)`` site reads ``material_of(name)``.
"""

from __future__ import annotations


MATERIAL_OF: dict[str, str | None] = {
    "rotor":      "pla_matte",
    "stator":     "pla_matte",
    "base_plate": "pla_matte",
    "pinion":     "pla_matte",
    "bearing":    "aluminium",
    "motor":      "cast_aluminium_dark",
}


def material_of(part_name: str) -> str | None:
    """Material for a named m02 part. Raises ``KeyError`` for an
    unknown name so a missing entry surfaces loudly.
    """
    if part_name not in MATERIAL_OF:
        raise KeyError(
            f"part '{part_name}' has no entry in MATERIAL_OF — add one "
            f"in cad/m02_c_channel_singulator/materials.py"
        )
    return MATERIAL_OF[part_name]
