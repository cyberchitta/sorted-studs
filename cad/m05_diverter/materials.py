"""m05 diverter — per-part materials.

Material is a physical property of the part, independent of the
WORKSHOP/BAUHAUS render-axis swap in ``design_systems.py``.
Centralised here so a material change is one edit; every
``.add(...)`` site reads ``material_of(name)``.

Keys are *basenames* — the part names sub-assembly builders use,
without any per-floor prefix. ``material_of(...)`` accepts either a
basename or a floor-prefixed instance name and uses
``design_systems.basename_of`` to strip the prefix before lookup.
"""

from __future__ import annotations

import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))

from design_systems import basename_of  # noqa: E402
from params import FLOOR_LAYOUT  # noqa: E402


# --- Basket name expansion --------------------------------------------

def _basket_basenames() -> list[str]:
    return [
        f"basket_{int(round(center_deg)):03d}_{n_size:02d}"
        for center_deg, n_size, _notches in FLOOR_LAYOUT
    ]


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
