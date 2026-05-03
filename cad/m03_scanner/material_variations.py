"""Render-only material overrides for the M03 scanner.

Settled materials live on the assembly itself (set at each
``.add(name=..., material=...)`` site). This module only carries
*unsettled* assignments — parts whose real-world material isn't yet
representable in chitra-cad's material table (transparent acrylic,
first-surface mirror glass, FR4 PCB) or whose answer is genuinely open
(opto-interrupter housing finish).

When a TODO is resolved upstream:
  * promote the material into the part's ``.add(...)`` call in the
    relevant ``assembly.py`` (intrinsic) or its parent ``.add()`` site
    (placement-varying), and
  * delete its entry here.

Add experimental bundles (e.g. ``STEEL_RAILS = {"rail_a": "steel", ...}``)
when sweeping for comparison renders or future FEA.
"""

from __future__ import annotations

DEFAULT_OVERRIDES: dict[str, str] = {
    # Camera bracket: PCB (FR4) and lens (glass) — chitra-cad has
    # neither material yet.
    "camera_pcb": "pla_matte",
    "camera_lens": "pla_matte",
    # Mirror glass: first-surface mirror — no mirror material in v0 table.
    "mirror_back_upper_glass": "pla_matte",
    "mirror_below_left_glass": "pla_matte",
    "mirror_below_right_glass": "pla_matte",
    # Turret platforms: clear acrylic — no transparent material in v0.
    "platform_image_acrylic": "pla_glossy",
    "platform_load_acrylic": "pla_glossy",
    "platform_dump_acrylic": "pla_glossy",
    "platform_return_acrylic": "pla_glossy",
    # Slotted opto-interrupter housing — black plastic vs generic
    # pla_matte not committed.
    "homing_sensor": "pla_matte",
}
