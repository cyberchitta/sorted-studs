"""m03 scanner — per-module design-system tags.

Schema (``Role``, ``Module``, ``DesignSystem``, palettes,
``DESIGN_SYSTEM_NAME`` switch, ``compose`` helper) lives in
``cad/design_systems.py``; this file holds only m03's tag dicts and
binds the module-local ``DESIGN_SYSTEM`` + ``color_of`` shim that
m03 sub-assemblies import.
"""

from __future__ import annotations

import sys
from pathlib import Path

from build123d import Color


sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from cad.design_systems import (  # noqa: E402
    BAUHAUS_COLORS,
    DESIGN_SYSTEM_NAME,
    DesignSystem,
    Module,
    Role,
    WORKSHOP_COLORS,
)


# --- m03 role tags ----------------------------------------------------

# Intrinsic — what the part DOES, not how it looks. Shared across every
# role-based palette. ``None`` means the part is outside the design
# system (hardware, optical glass, clear acrylic) and keeps its
# material's intrinsic color.
ROLE_OF: dict[str, Role | None] = {
    # --- camera_rig.skeleton_t_frame (4 rails + 1 mitered brace) ------
    "rail_a": Role.PRIMARY_STRUCTURE,
    "rail_b": Role.PRIMARY_STRUCTURE,
    "rail_c": Role.PRIMARY_STRUCTURE,
    "rail_d": Role.PRIMARY_STRUCTURE,
    "brace_bc": Role.PRIMARY_STRUCTURE,
    # --- camera_rig: gussets + four ESP32-CAM mounts ------------------
    "gusset_t_ab": Role.SECONDARY_STRUCTURE,
    "gusset_l_cd_pos_x": Role.SECONDARY_STRUCTURE,
    "gusset_l_cd_neg_x": Role.SECONDARY_STRUCTURE,
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
    # --- turret.turret_frame (5-member 2020 floor) --------------------
    "floor_back_x": Role.PRIMARY_STRUCTURE,
    "floor_front_x": Role.PRIMARY_STRUCTURE,
    "floor_left_y": Role.PRIMARY_STRUCTURE,
    "floor_right_y": Role.PRIMARY_STRUCTURE,
    "floor_cross_x": Role.PRIMARY_STRUCTURE,
    # --- turret.turret_hub --------------------------------------------
    "hub_housing": Role.SECONDARY_STRUCTURE,
    "hub_shaft": None,                                  # steel
    "hub_bearing_top": None,                            # steel bearing
    "hub_bearing_bottom": None,                         # steel bearing
    # --- turret.turret_arm (spider + 4 hinge pins) --------------------
    "spider": Role.MECHANISM,
    "pin_image": None,                                  # steel
    "pin_load": None,
    "pin_dump": None,
    "pin_return": None,
    # --- turret.turret_platform × 4 stations × 2 sub-parts ------------
    # Acrylic insert stays None (clear); the integrated printed frame
    # tilts with the platform during dump → MECHANISM.
    **{
        f"platform_{station}_{sub}": role
        for station in ("image", "load", "dump", "return")
        for sub, role in (
            ("acrylic", None),
            ("frame", Role.MECHANISM),
        )
    },
    # --- turret.turret_dump_cam ---------------------------------------
    "dump_cam": Role.MECHANISM,
    # --- turret.turret_dump_kicker ------------------------------------
    "kicker_mount": Role.SECONDARY_STRUCTURE,
    "kicker_lever": Role.MECHANISM,
    "kicker_pin": None,                                 # steel
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


# --- Bound DesignSystems ----------------------------------------------

WORKSHOP = DesignSystem(name="workshop", tag_of=ROLE_OF, palette=WORKSHOP_COLORS)
BAUHAUS = DesignSystem(name="bauhaus", tag_of=MODULE_OF, palette=BAUHAUS_COLORS)

DESIGN_SYSTEM: DesignSystem = (
    {"workshop": WORKSHOP, "bauhaus": BAUHAUS}[DESIGN_SYSTEM_NAME]
)


def color_of(part_name: str) -> Color | None:
    """Active design system's color for a named m03 part.

    Thin shim over ``DESIGN_SYSTEM.color_of(...)`` so call sites import
    one symbol regardless of which palette is active.
    """
    return DESIGN_SYSTEM.color_of(part_name)
