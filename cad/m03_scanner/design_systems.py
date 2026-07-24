"""m03 scanner — per-module design-system tags.

Schema (``Role``, ``Module``, ``DesignSystem``, palettes,
``DESIGN_SYSTEM_NAME`` switch) lives in
``cad/design_systems.py``; this file holds only m03's tag dicts and
binds the module-local ``DESIGN_SYSTEM`` + ``color_of`` shim that
m03 sub-assemblies import.
"""

from __future__ import annotations


from build123d import Color


from ..design_systems import (
    DesignSystem,
    Module,
    Role,
    bind_all,
    resolve_name,
)


# --- m03 role tags ----------------------------------------------------

# Intrinsic — what the part DOES, not how it looks. Shared across every
# role-based palette. ``None`` means the part is outside the design
# system (hardware, optical glass, clear acrylic) and keeps its
# material's intrinsic color.
ROLE_OF: dict[str, Role | None] = {
    # --- camera_rig.arch (Phase-2 integrated frame) -------------------
    "arch_post_pos_x": Role.PRIMARY_STRUCTURE,
    "arch_post_neg_x": Role.PRIMARY_STRUCTURE,
    "arch_stub_pos_x": Role.PRIMARY_STRUCTURE,
    "arch_stub_neg_x": Role.PRIMARY_STRUCTURE,
    "rail_d": Role.PRIMARY_STRUCTURE,
    # --- camera_rig: four ESP32-CAM mounts ----------------------------
    "cam_front_low_clamp_mount": Role.OPTICAL_ENVELOPE,
    "cam_back_upper_clamp_mount": Role.OPTICAL_ENVELOPE,
    "cam_below_left_clamp_mount": Role.OPTICAL_ENVELOPE,
    "cam_below_right_clamp_mount": Role.OPTICAL_ENVELOPE,
    "cam_front_low_pcb": None,                          # FR4 — outside palette
    "cam_back_upper_pcb": None,
    "cam_below_left_pcb": None,
    "cam_below_right_pcb": None,
    "cam_front_low_lens": None,                         # glass — outside palette
    "cam_back_upper_lens": None,
    "cam_below_left_lens": None,
    "cam_below_right_lens": None,
    # --- turret.frame (6-member 2020 floor) ---------------------------
    "floor_back_x": Role.PRIMARY_STRUCTURE,
    "floor_left_y": Role.PRIMARY_STRUCTURE,
    "floor_right_y": Role.PRIMARY_STRUCTURE,
    "floor_cross_x": Role.PRIMARY_STRUCTURE,
    "floor_outrig_pos_x": Role.PRIMARY_STRUCTURE,
    "floor_outrig_neg_x": Role.PRIMARY_STRUCTURE,
    # --- turret.turret_hub --------------------------------------------
    "hub_housing": Role.SECONDARY_STRUCTURE,
    "hub_shaft": None,                                  # steel
    "hub_bearing_top": None,                            # steel bearing
    "hub_bearing_bottom": None,                         # steel bearing
    # --- turret.turret_arm (spider + 4 hinge pins) --------------------
    "spider": Role.MECHANISM,
    "pin_image": None,                                  # steel
    "pin_load": None,
    "pin_idle": None,
    "pin_dump": None,
    # --- turret.rotor.platform_<station> — shared cartridge leaves ----
    # Station identity is the subtree path; tags key on the leaf name.
    # Acrylic insert stays None (clear); the integrated printed frame
    # tilts with the platform during dump → MECHANISM.
    "acrylic": None,
    "frame": Role.MECHANISM,
    # --- turret.dump_servo --------------------------------------------
    "servo_bracket": Role.SECONDARY_STRUCTURE,
    "servo_arm": Role.MECHANISM,
    "servo_body": None,                                 # bought hobby servo
    # --- turret.turret_drive ------------------------------------------
    "drive_pulley_shaft": None,                         # GT2 pulley, hardware
    "drive_pulley_motor": None,                         # GT2 pulley, hardware
    "drive_motor": None,                                # NEMA17 cast body
    "drive_bracket": Role.SECONDARY_STRUCTURE,
    # --- turret.turret_homing -----------------------------------------
    "homing_flag": Role.MECHANISM,
    "homing_sensor": None,                              # opto-interrupter housing
    "homing_bracket": Role.SECONDARY_STRUCTURE,
}


# --- m03 module tags --------------------------------------------------

# Every part tagged in ROLE_OF is also tagged here. Parts whose
# ROLE_OF is ``None`` (hardware / optical / acrylic) are also ``None``
# here — design-system axes share the "outside the system" set.
MODULE_OF: dict[str, Module | None] = {
    name: (Module.SCANNER if role is not None else None)
    for name, role in ROLE_OF.items()
}


# --- Bound DesignSystem -----------------------------------------------

DESIGN_SYSTEM: DesignSystem = bind_all(ROLE_OF, MODULE_OF)[resolve_name("M03")]


def color_of(part_name: str) -> Color | None:
    """Active design system's color for a named m03 part.

    Thin shim over ``DESIGN_SYSTEM.color_of(...)`` so call sites import
    one symbol regardless of which palette is active.
    """
    return DESIGN_SYSTEM.color_of(part_name)
