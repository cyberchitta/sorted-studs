"""Render-only material overrides for the M03 scanner.

Settled materials live on the assembly itself (set at each
``.with_part(name=..., material=...)`` site). This module only carries
*unsettled* assignments — parts whose real-world material isn't yet
representable in chitra-cad's material table (transparent acrylic,
first-surface mirror glass, FR4 PCB) or whose answer is genuinely open
(opto-interrupter housing finish).

When a TODO is resolved upstream:
  * promote the material into the part's ``.with_part(...)`` call in the
    relevant ``assembly.py`` (intrinsic) or its parent ``.with_part()`` site
    (placement-varying), and
  * delete its entry here.

Add experimental bundles (e.g. ``STEEL_RAILS = {"rail_a": "steel", ...}``)
when sweeping for comparison renders or future FEA.
"""

from __future__ import annotations

DEFAULT_OVERRIDES: dict[str, str] = {
    # Camera mounts: PCB (FR4) and lens (glass) for each of the four
    # tetrahedral cameras — chitra-cad has neither material yet.
    "cam_front_low_pcb": "pla_matte",
    "cam_back_upper_pcb": "pla_matte",
    "cam_below_left_pcb": "pla_matte",
    "cam_below_right_pcb": "pla_matte",
    "cam_front_low_lens": "pla_matte",
    "cam_back_upper_lens": "pla_matte",
    "cam_below_left_lens": "pla_matte",
    "cam_below_right_lens": "pla_matte",
    # Slotted opto-interrupter housing — black plastic vs generic
    # pla_matte not committed.
    "homing_sensor": "pla_matte",
}
