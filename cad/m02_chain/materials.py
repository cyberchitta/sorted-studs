"""m02 chain — per-part materials.

Chain-own parts only: the gantry members are 2020 aluminum
extrusion, their corner brackets die-cast; the joint parts that take
the compound angles and the LOAD ramp are FDM-printed. The
`s1` / `s2` / `s3` m02 subtrees carry m02's own materials
(bound in `cad/m02_c_channel_singulator/materials.py`).
"""

from __future__ import annotations


MATERIAL_OF: dict[str, str | None] = {
    **{name: "aluminium_anodized" for name in (
        "s1_prop_bar", "s3_prop_bar", "s3_cross_a", "s3_cross_b",
        "s2_stem", "s2_tee", "brace_3", "brace_5",
    )},
    # Bought 2020 corner brackets (die-cast).
    **{name: "cast_aluminium_dark" for name in (
        "s2_stem_bracket_top", "s2_stem_bracket_left", "s2_stem_bracket_right",
        "s2_tee_bracket_left", "s2_tee_bracket_right",
        "s3_cross_bracket_a1", "s3_cross_bracket_a2", "s3_cross_bracket_b1", "s3_cross_bracket_b2",
    )},
    # Printed joints (`joints.py`).
    **{name: "pla_matte" for name in (
        "s1_post_seat", "s3_post_seat", "brace_head", "brace_foot_3", "brace_foot_5",
    )},
    "load_ramp":    "pla_matte",
    # V_3's foot bolt and washer (bought steel).
    "foot_bolt_3": "steel",
    "foot_washer_3": "steel",
    # LOAD beam-break halves (bought sensor envelopes) — left to the default.
    "load_beam_emitter": None,
    "load_beam_receiver": None,
    # The receiver guard (printed) and its two M3 screws (bought steel).
    "load_beam_guard": "pla_matte",
    "load_beam_guard_screw_a": "steel",
    "load_beam_guard_screw_b": "steel",
}


def material_of(part_name: str) -> str | None:
    """Material for a named chain part. Raises ``KeyError`` for an
    unknown name so a missing entry surfaces loudly."""
    if part_name not in MATERIAL_OF:
        raise KeyError(
            f"part '{part_name}' has no entry in MATERIAL_OF — add one "
            f"in cad/m02_chain/materials.py"
        )
    return MATERIAL_OF[part_name]
