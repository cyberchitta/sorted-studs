"""m02 chain — per-part materials.

Column extensions and yokes are 2020 aluminum extrusion. The stator
support plate and the LOAD ramp are FDM-printed. Spliced m02 parts
inherit m02's materials (PLA matte for printed parts, cast aluminum
for the NEMA-17 motor body).
"""

from __future__ import annotations


_S1_PREFIX = "s1__"


MATERIAL_OF: dict[str, str | None] = {
    "col1_ext":                    "aluminium_anodized",
    "col2_ext":                    "aluminium_anodized",
    f"{_S1_PREFIX}yoke":           "aluminium_anodized",
    f"{_S1_PREFIX}stator_plate":   "pla_matte",
    f"{_S1_PREFIX}load_ramp":      "pla_matte",
    # Spliced m02 parts — mirror cad/m02_c_channel_singulator/materials.py.
    f"{_S1_PREFIX}rotor":          "pla_matte",
    f"{_S1_PREFIX}stator":         "pla_matte",
    f"{_S1_PREFIX}stem":           "pla_matte",
    f"{_S1_PREFIX}motor":          "cast_aluminium_dark",
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
