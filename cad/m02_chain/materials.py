"""m02 chain — per-part materials.

Chain-own parts only: column extensions and yokes are 2020 aluminum
extrusion; the stator support plate and the LOAD ramp are
FDM-printed. The `s1` m02 subtree carries m02's own materials
(bound in `cad/m02_c_channel_singulator/materials.py`).
"""

from __future__ import annotations


MATERIAL_OF: dict[str, str | None] = {
    "col1_ext":     "aluminium_anodized",
    "col2_ext":     "aluminium_anodized",
    "yoke":         "aluminium_anodized",
    "stator_plate": "pla_matte",
    "load_ramp":    "pla_matte",
}


def material_of(part_name: str) -> str | None:
    """Material for a named chain part. Raises ``KeyError`` for an
    unknown name so a missing entry surfaces loudly."""
    if part_name not in MATERIAL_OF:
        raise KeyError(
            f"part '{part_name}' has no entry in MATERIAL_OF — add one "
            f"in cad/m02_chain/materials.py"
        )
    return MATERIAL_OF[part_name]
