"""m05 diverter — per-part materials.

Material is a physical property of the part, independent of the
WORKSHOP/BAUHAUS render-axis swap in ``design_systems.py``.
Centralised here so a material change is one edit; every
``.with_part(...)`` site reads ``material_of(name)``.

Keys are *basenames* — the leaf part names the sub-assembly builders
use. ``material_of(...)`` accepts either a basename or a qualified
tree path and uses ``design_systems.basename_of`` to take the leaf
segment before lookup; instance identity (which floor) lives in the
subtree path, not the name.
"""

from __future__ import annotations


from .design_systems import basename_of
from .params import all_basket_names


# --- Basket name expansion --------------------------------------------

def _basket_basenames() -> list[str]:
    return all_basket_names()


MATERIAL_OF: dict[str, str | None] = {
    # 2020 extrusions — anodized aluminium
    **{f"outer_vert_{k}": "aluminium_anodized" for k in range(6)},
    **{f"inner_vert_{k}": "aluminium_anodized" for k in range(6)},
    **{f"outer_horiz_{k}": "aluminium_anodized" for k in range(6)},
    **{f"inner_horiz_{k}": "aluminium_anodized" for k in range(6)},
    **{f"radial_inner_{k}": "aluminium_anodized" for k in range(6)},
    **{f"radial_outer_{k}": "aluminium_anodized" for k in range(6)},
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
    "ring_gear": "pla_matte",
    **{name: "pla_matte" for name in _basket_basenames()},
    # Hardware
    "hinge_pin": "steel",
    "motor": "cast_aluminium_dark",
    "pinion": "steel",
    # Top deck
    "top_acrylic": "acrylic_clear",
}


def material_of(part_name: str) -> str | None:
    """Material for a m05 part instance. Same instance-or-basename
    contract as ``color_of``. Raises ``KeyError`` for an unknown
    basename so a missing entry surfaces loudly.
    """
    bn = basename_of(part_name)
    if bn not in MATERIAL_OF:
        raise KeyError(
            f"part '{part_name}' (basename '{bn}') has no entry in "
            f"MATERIAL_OF — add one in cad/m05_diverter/materials.py"
        )
    return MATERIAL_OF[bn]
