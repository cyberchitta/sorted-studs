"""M03 scanner — photo-real render via chitra-cad.

Imports the composed top-level assembly (already module-level in
``assembly.py``) and assigns a material to every named part. Lighting
and camera framing come from chitra-cad's defaults; do not author them
here. If a material is missing, add it to
``../../chitra-cad/src/chitra_cad/materials.py``.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from assembly import assembly  # noqa: E402

from chitra_cad.render import render  # noqa: E402
from chitra_cad.scene import Scene  # noqa: E402


MATERIALS: dict[str, str] = {
    # --- Camera rig: 2020 V-slot skeleton (aluminium) -------------------
    "rail_a": "aluminium_anodized",
    "rail_b": "aluminium_anodized",
    "rail_c": "aluminium_anodized",
    "rail_d": "aluminium_anodized",
    "brace_bc": "aluminium_anodized",
    "strut_d": "aluminium_anodized",
    # --- Camera rig: printed gussets -----------------------------------
    "gusset_t_ab": "plastic_matte",
    "gusset_l_cd": "plastic_matte",
    # --- Camera bracket triplet ----------------------------------------
    "camera_plate": "plastic_matte",
    "camera_pcb": "plastic_matte",  # TODO: confirm material — green PCB, no FR4 material in v0 table
    "camera_lens": "plastic_matte",  # TODO: confirm material — lens stub; chitra-cad has no glass material yet
    # --- Mirrors: printed brackets + first-surface mirror glass --------
    "mirror_back_upper_bracket": "plastic_matte",
    "mirror_back_upper_glass": "plastic_matte",  # TODO: confirm material — first-surface mirror; no mirror material in v0 table
    "mirror_below_left_bracket": "plastic_matte",
    "mirror_below_left_glass": "plastic_matte",  # TODO: confirm material — first-surface mirror; no mirror material in v0 table
    "mirror_below_right_bracket": "plastic_matte",
    "mirror_below_right_glass": "plastic_matte",  # TODO: confirm material — first-surface mirror; no mirror material in v0 table
    # --- Camera-rig printed pad clamps ---------------------------------
    "pad_clamp_below_left": "plastic_matte",
    "pad_clamp_below_right": "plastic_matte",
    "pad_clamp_back_upper": "plastic_matte",
    "pad_clamp_camera": "plastic_matte",
    # --- Turret frame: 5-member 2020 floor frame ------------------------
    "floor_back_x": "aluminium_anodized",
    "floor_front_x": "aluminium_anodized",
    "floor_left_z": "aluminium_anodized",
    "floor_right_z": "aluminium_anodized",
    "floor_cross_x": "aluminium_anodized",
    # --- Turret hub: printed pillow-block, steel shaft + bearings -------
    "hub_housing": "plastic_matte",
    "hub_shaft": "steel",
    "hub_bearing_top": "steel",
    "hub_bearing_bottom": "steel",
    # --- Turret arm: spider + 4 stock pins ------------------------------
    "spider": "plastic_matte",  # TODO: confirm material — printed vs. aluminium plate
    "pin_image": "steel",
    "pin_load": "steel",
    "pin_return": "steel",
    "pin_dump": "steel",
    # --- Turret platforms (×4 stations): acrylic + printed brackets -----
    "platform_image_acrylic": "plastic_glossy",  # TODO: confirm material — clear acrylic; chitra-cad has no acrylic/transparent material yet
    "platform_image_bracket_pos": "plastic_matte",
    "platform_image_bracket_neg": "plastic_matte",
    "platform_image_ring": "plastic_matte",
    "platform_image_kicker_tab": "plastic_matte",
    "platform_load_acrylic": "plastic_glossy",  # TODO: confirm material — clear acrylic; chitra-cad has no acrylic/transparent material yet
    "platform_load_bracket_pos": "plastic_matte",
    "platform_load_bracket_neg": "plastic_matte",
    "platform_load_ring": "plastic_matte",
    "platform_load_kicker_tab": "plastic_matte",
    "platform_dump_acrylic": "plastic_glossy",  # TODO: confirm material — clear acrylic; chitra-cad has no acrylic/transparent material yet
    "platform_dump_bracket_pos": "plastic_matte",
    "platform_dump_bracket_neg": "plastic_matte",
    "platform_dump_ring": "plastic_matte",
    "platform_dump_kicker_tab": "plastic_matte",
    "platform_return_acrylic": "plastic_glossy",  # TODO: confirm material — clear acrylic; chitra-cad has no acrylic/transparent material yet
    "platform_return_bracket_pos": "plastic_matte",
    "platform_return_bracket_neg": "plastic_matte",
    "platform_return_ring": "plastic_matte",
    "platform_return_kicker_tab": "plastic_matte",
    # --- Turret active-push tilt: printed cam + lever, steel pin --------
    "dump_cam": "plastic_matte",
    "kicker_mount": "plastic_matte",
    "kicker_lever": "plastic_matte",
    "kicker_pin": "steel",
    # --- Turret drive: NEMA17 + GT2 belt + printed L-bracket ------------
    "drive_pulley_shaft": "aluminium_anodized",
    "drive_pulley_motor": "aluminium_anodized",
    "drive_motor": "motor_casing",
    "drive_bracket": "plastic_matte",
    # --- Turret homing: printed flag + bracket, opto-interrupter --------
    "homing_flag": "plastic_matte",
    "homing_sensor": "plastic_matte",  # TODO: confirm material — slotted opto-interrupter (black plastic housing + LED/photodiode)
    "homing_bracket": "plastic_matte",
}


scene = Scene.from_assembly(assembly).with_materials(MATERIALS)


if __name__ == "__main__":
    render(scene, out="renders", resolution=(1920, 1080))
