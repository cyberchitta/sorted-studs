"""m03 scanner — per-part materials.

Material is a physical property of the part, independent of the
WORKSHOP/BAUHAUS render-axis swap in ``design_systems.py``.
Centralised here so a material change is one edit; every
``.add(...)`` site reads ``material_of(name)``.

Several parts are deliberately omitted (``cam_*_pcb``, ``cam_*_lens``,
``homing_sensor``): their settled material is unset at the ``.add(...)``
site and ``material_variations.py`` applies a render-time override.
Calling ``material_of(...)`` on them raises, by design.

Sub-assembly-local stub fixtures (``hub_shaft_stub``, ``cross_rail_stub``,
``cam_stub``, ``rail_*_stub``, the standalone ``"gusset"``) are not in
``ROLE_OF`` and not here either — they keep their inline material
strings at the standalone-test ``.add(...)`` sites.
"""

from __future__ import annotations


MATERIAL_OF: dict[str, str | None] = {
    # camera_rig.arch — Phase-2 integrated frame (2020 extrusions)
    "arch_post_pos_x": "aluminium_anodized",
    "arch_post_neg_x": "aluminium_anodized",
    "arch_stub_pos_x": "aluminium_anodized",
    "arch_stub_neg_x": "aluminium_anodized",
    "rail_d": "aluminium_anodized",
    # camera_rig: four ESP32-CAM mounts
    "cam_front_low_clamp_mount": "pla_matte",
    "cam_back_upper_clamp_mount": "pla_matte",
    "cam_below_left_clamp_mount": "pla_matte",
    "cam_below_right_clamp_mount": "pla_matte",
    # cam_*_pcb / cam_*_lens — see docstring; not in this dict.
    # turret.frame — 2020 floor + outriggers
    "floor_back_x": "aluminium_anodized",
    "floor_left_y": "aluminium_anodized",
    "floor_right_y": "aluminium_anodized",
    "floor_cross_x": "aluminium_anodized",
    "floor_outrig_pos_x": "aluminium_anodized",
    "floor_outrig_neg_x": "aluminium_anodized",
    # turret.turret_hub
    "hub_housing": "pla_matte",
    "hub_shaft": "steel",
    "hub_bearing_top": "steel",
    "hub_bearing_bottom": "steel",
    # turret.turret_arm
    "spider": "pla_matte",
    "pin_image": "steel",
    "pin_load": "steel",
    "pin_idle": "steel",
    "pin_dump": "steel",
    # turret.turret_platform — 4 stations × {acrylic, frame}
    **{
        f"platform_{station}_{sub}": mat
        for station in ("image", "load", "idle", "dump")
        for sub, mat in (
            ("acrylic", "acrylic_clear"),
            ("frame", "pla_matte"),
        )
    },
    # turret.turret_dump_cam
    "dump_cam": "pla_matte",
    # turret.turret_dump_kicker
    "kicker_mount": "pla_matte",
    "kicker_lever": "pla_matte",
    "kicker_pin": "steel",
    # turret.turret_drive
    "drive_pulley_shaft": "aluminium_anodized",
    "drive_pulley_motor": "aluminium_anodized",
    "drive_motor": "cast_aluminium_dark",
    "drive_bracket": "pla_matte",
    # turret.turret_homing
    "homing_flag": "pla_matte",
    # homing_sensor — see docstring; not in this dict.
    "homing_bracket": "pla_matte",
}


def material_of(part_name: str) -> str | None:
    """Material for a named m03 part. Raises ``KeyError`` for an
    unknown name so a missing entry surfaces loudly.
    """
    if part_name not in MATERIAL_OF:
        raise KeyError(
            f"part '{part_name}' has no entry in MATERIAL_OF — add one "
            f"in cad/m03_scanner/materials.py"
        )
    return MATERIAL_OF[part_name]
