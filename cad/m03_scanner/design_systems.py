"""Parameterised design systems for sorted-studs.

A "design system" is a binding from part name to color via some axis.
The same architecture supports multiple axes — functional role,
module, floor, sub-unit, etc. — by parameterising the axis enum and
its per-part tagging dict.

Every concrete design system is a ``DesignSystem(name, tag_of, palette)``:

  * ``tag_of`` — every named printed / extrusion part maps to an axis
    value (or ``None`` for parts outside the design system: hardware,
    optical glass, clear acrylic).
  * ``palette`` — each axis value maps to a ``Color``.

Today two are wired up:

  * ``WORKSHOP`` — axis = functional role (what the part *does*).
    Graphite skeleton, signal-yellow mechanism, matte-black optical
    envelope, warm-off-white consumables.
  * ``BAUHAUS``  — axis = module (which subsystem the part lives in).
    Saturated primary-color wayfinding: m02 yellow, m03 red, m05 blue.
    Today only m03 is tagged, so a Bauhaus render reads as monochrome
    red until m02 / m05 migrate.

Re-point ``DESIGN_SYSTEM`` to swap the active look. The top-level
coverage assertion validates against ``DESIGN_SYSTEM.tag_of``, so a
swap to a palette with missing tags fails ``khana check`` cleanly.

Scope today is M03 only. M02 / M05 add their own tags to both
``ROLE_OF`` and ``MODULE_OF`` when they migrate; promote the schema
to ``cad/`` when the second module lands.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from build123d import Color


# --- Functional-role axis ---------------------------------------------

class Role(Enum):
    """Functional role of a part — what it does in the machine."""

    PRIMARY_STRUCTURE = "primary_structure"     # 2020 extrusions
    SECONDARY_STRUCTURE = "secondary_structure" # printed brackets / mounts
    MECHANISM = "mechanism"                     # rotating / hinged / pushing
    OPTICAL_ENVELOPE = "optical_envelope"       # inside camera FOV
    CONSUMABLE = "consumable"                   # holds bricks; swapped often


# Intrinsic — what the part DOES, not how it looks. Shared across
# every role-based palette. ``None`` means the part is outside the
# design system (hardware, optical glass, clear acrylic) and keeps
# its material's intrinsic color.
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
    # Acrylic stays None (clear); printed sub-parts tilt with the
    # platform during dump → MECHANISM.
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


# --- Module axis ------------------------------------------------------

class Module(Enum):
    """Which top-level subsystem a part belongs to."""

    SINGULATOR = "singulator"   # m02
    SCANNER = "scanner"         # m03
    DIVERTER = "diverter"       # m05


# Every part tagged in ROLE_OF is also tagged here. Today only m03 is
# wired, so every active entry is ``SCANNER``; m02 / m05 add their
# parts when those modules migrate. Parts whose ROLE_OF is ``None``
# (hardware / optical / acrylic) are also ``None`` here — design-system
# axes share the "outside the system" set.
MODULE_OF: dict[str, Module | None] = {
    name: (Module.SCANNER if role is not None else None)
    for name, role in ROLE_OF.items()
}


# --- DesignSystem -----------------------------------------------------

# Type alias: the tag axis is any Enum the design system chooses (Role,
# Module, future Floor / Unit / etc.). Python's enum hierarchy is too
# loose to constrain this to a TypeVar cleanly, so we accept ``Enum``.
@dataclass(frozen=True)
class DesignSystem:
    """A binding from part name → color via one named axis.

    ``tag_of`` covers every named printed / extrusion part; ``None``
    means the part is outside the system (hardware, optical glass,
    acrylic). ``palette`` maps each axis value to its color.
    """

    name: str
    tag_of: dict[str, Enum | None]
    palette: dict[Enum, Color]

    def color_of(self, part_name: str) -> Color | None:
        """Active color for a named part. ``None`` falls through to the
        material's intrinsic color. Raises ``KeyError`` for an untagged
        part — the top-level coverage assertion catches this earlier,
        but the explicit raise prevents silent off-palette renders.
        """
        if part_name not in self.tag_of:
            raise KeyError(
                f"part '{part_name}' has no tag in design system "
                f"'{self.name}' — add it to the relevant tagging dict "
                f"in cad/m03_scanner/design_systems.py"
            )
        tag = self.tag_of[part_name]
        return self.palette[tag] if tag is not None else None


# --- Palettes ---------------------------------------------------------

WORKSHOP_COLORS: dict[Enum, Color] = {
    Role.PRIMARY_STRUCTURE:   Color(0.18, 0.18, 0.20),  # graphite anodized
    Role.SECONDARY_STRUCTURE: Color(0.18, 0.18, 0.20),  # graphite PLA
    Role.MECHANISM:           Color(0.95, 0.78, 0.10),  # signal yellow
    Role.OPTICAL_ENVELOPE:    Color(0.04, 0.04, 0.04),  # matte black
    Role.CONSUMABLE:          Color(0.92, 0.90, 0.84),  # warm off-white
}

BAUHAUS_COLORS: dict[Enum, Color] = {
    Module.SINGULATOR: Color(1.00, 0.85, 0.05),         # primary yellow
    Module.SCANNER:    Color(0.85, 0.10, 0.10),         # primary red
    Module.DIVERTER:   Color(0.08, 0.30, 0.75),         # primary blue
}


WORKSHOP = DesignSystem(name="workshop", tag_of=ROLE_OF, palette=WORKSHOP_COLORS)
BAUHAUS = DesignSystem(name="bauhaus", tag_of=MODULE_OF, palette=BAUHAUS_COLORS)


# --- Active selection -------------------------------------------------

DESIGN_SYSTEM: DesignSystem = WORKSHOP


def color_of(part_name: str) -> Color | None:
    """Active design system's color for a named part.

    Thin shim over ``DESIGN_SYSTEM.color_of(...)`` so call sites import
    one symbol regardless of which palette is active.
    """
    return DESIGN_SYSTEM.color_of(part_name)
