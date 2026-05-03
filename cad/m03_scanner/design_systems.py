"""Functional-role design system for the M03 scanner.

Two layers, kept separate so palette swaps don't touch the parts and
part renames don't touch the palette:

  * ``ROLE_OF`` — intrinsic. Maps every named printed / extrusion part
    to its functional role. Hardware (steel pins, bearings, motor body),
    optical glass, and clear acrylic map to ``None`` — they keep their
    intrinsic material color and stay outside the design system.
  * ``WORKSHOP`` — palette. Maps each role to a ``Color``.
  * ``DESIGN_SYSTEM`` — the active palette binding. Re-point this to
    swap looks; nothing in any ``assembly.py`` changes.

Call sites read the active binding via ``color_of(name)``, passed
alongside ``material=`` at each ``.add(...)``. Coverage is asserted at
the top-level: every part in ``CAMERA_RIG_PART_NAMES + TURRET_PART_NAMES``
must appear in ``ROLE_OF`` (``None`` allowed). Adding a part without a
role tag fails ``khana check``.

Scope today is M03 only. M02 / M05 add their own role tags when they
migrate; promote shared schema (``Role``, ``color_of``) to ``cad/`` when
the second module lands.
"""

from __future__ import annotations

from enum import Enum

from build123d import Color


class Role(Enum):
    """Functional role of a part in the machine.

    The taxonomy answers "what does this part do?", not "what does it
    look like?". A palette decides the second question.
    """

    PRIMARY_STRUCTURE = "primary_structure"     # 2020 extrusions
    SECONDARY_STRUCTURE = "secondary_structure" # printed brackets / mounts
    MECHANISM = "mechanism"                     # rotating / hinged / pushing
    OPTICAL_ENVELOPE = "optical_envelope"       # inside camera FOV (locked black)
    CONSUMABLE = "consumable"                   # holds bricks; swapped often


# --- Functional role tagging (intrinsic; shared across palettes) -------
#
# ``None`` means the part is outside the design system: hardware (steel,
# motor body), optical glass (mirrors, lens, PCB), or clear acrylic.
# Those keep their intrinsic material color from chitra-cad's catalog.

ROLE_OF: dict[str, Role | None] = {
    # --- camera_rig.skeleton_t_frame (4 rails + 2 mitered braces) -----
    "rail_a": Role.PRIMARY_STRUCTURE,
    "rail_b": Role.PRIMARY_STRUCTURE,
    "rail_c": Role.PRIMARY_STRUCTURE,
    "rail_d": Role.PRIMARY_STRUCTURE,
    "brace_bc": Role.PRIMARY_STRUCTURE,
    "strut_d": Role.PRIMARY_STRUCTURE,
    # --- camera_rig: gussets, camera bracket, mirrors -----------------
    "gusset_t_ab": Role.SECONDARY_STRUCTURE,
    "gusset_l_cd": Role.SECONDARY_STRUCTURE,
    "camera_clamp_bracket": Role.OPTICAL_ENVELOPE,
    "camera_pcb": None,                                 # FR4 — outside palette
    "camera_lens": None,                                # glass — outside palette
    "mirror_below_left_clamp_mount": Role.OPTICAL_ENVELOPE,
    "mirror_below_right_clamp_mount": Role.OPTICAL_ENVELOPE,
    "mirror_back_upper_clamp_mount": Role.OPTICAL_ENVELOPE,
    "mirror_below_left_glass": None,                    # first-surface mirror
    "mirror_below_right_glass": None,
    "mirror_back_upper_glass": None,
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
    # --- turret.turret_platform × 4 stations × 5 sub-parts ------------
    # acrylic stays None (clear); the printed brackets / ring / kicker
    # tab tilt with the platform during dump → MECHANISM.
    **{
        f"platform_{station}_{sub}": role
        for station in ("image", "load", "dump", "return")
        for sub, role in (
            ("acrylic", None),
            ("bracket_pos", Role.MECHANISM),
            ("bracket_neg", Role.MECHANISM),
            ("ring", Role.MECHANISM),
            ("kicker_tab", Role.MECHANISM),
        )
    },
    # --- turret.turret_dump_cam ---------------------------------------
    "dump_cam": Role.MECHANISM,
    # --- turret.turret_dump_kicker ------------------------------------
    "kicker_mount": Role.SECONDARY_STRUCTURE,           # bench-mounted, fixed
    "kicker_lever": Role.MECHANISM,                     # rocks under cam
    "kicker_pin": None,                                 # steel
    # --- turret.turret_drive ------------------------------------------
    "drive_pulley_shaft": None,                         # GT2 pulley, hardware
    "drive_pulley_motor": None,                         # GT2 pulley, hardware
    "drive_motor": None,                                # NEMA17 cast body
    "drive_bracket": Role.SECONDARY_STRUCTURE,          # printed L-bracket
    # --- turret.turret_homing -----------------------------------------
    "homing_flag": Role.MECHANISM,                      # spins with shaft
    "homing_sensor": None,                              # opto-interrupter housing
    "homing_bracket": Role.SECONDARY_STRUCTURE,
}


# --- Palettes ----------------------------------------------------------

WORKSHOP: dict[Role, Color] = {
    Role.PRIMARY_STRUCTURE:   Color(0.18, 0.18, 0.20),  # graphite anodized
    Role.SECONDARY_STRUCTURE: Color(0.18, 0.18, 0.20),  # graphite PLA
    Role.MECHANISM:           Color(0.95, 0.78, 0.10),  # signal yellow
    Role.OPTICAL_ENVELOPE:    Color(0.04, 0.04, 0.04),  # matte black (stray-light)
    Role.CONSUMABLE:          Color(0.92, 0.90, 0.84),  # warm off-white
}


# --- Active selection --------------------------------------------------

DESIGN_SYSTEM: dict[Role, Color] = WORKSHOP


def color_of(name: str) -> Color | None:
    """Active palette's color for a named part. Returns ``None`` for
    parts whose role is ``None`` (hardware / optical / acrylic) so the
    placement falls back to its material's intrinsic base color.

    Raises ``KeyError`` for unknown names — the top-level coverage
    assertion catches missing tags before this would, but the explicit
    raise here means a typo in a call site fails loudly instead of
    silently rendering the wrong color.
    """
    if name not in ROLE_OF:
        raise KeyError(
            f"part '{name}' has no design-system role tag — add it to "
            f"ROLE_OF in cad/m03_scanner/design_systems.py"
        )
    role = ROLE_OF[name]
    return DESIGN_SYSTEM[role] if role is not None else None
