"""
M03 scanner — detail-tier check module.

A **check module**: it imports the product assembly, applies the
render-time detail bundle, and exposes the result so `khana check`
evaluates the claims that only a detailed build can answer.

    .venv/bin/khana check cad/m03_scanner/check_detail.py
    # → cad/m03_scanner/outputs/check_detail/mechanism.json

The cheap `assembly.py` check (and every parent that composes it,
`cad/assembly.py` included) skips the drive-foot M5 bolt assertions as
`passed: null`: the bolts exist only once
`detail_variations.apply_default_detail` adds them.

**One factory, not two.** There used to be a second —
`build_drive_detailed` — because the bracket-side bolt claims were
declared in `drive/assembly.py`'s *standalone* fixture, which no
composed run ever calls, and the additions were keyed bare so they
landed at whatever root the override was applied to. Both sides moved
on 2026-09-20: the claims to the levels that own them
(`build_turret_drive_static` for bolt-vs-bracket and bolt-vs-motor,
`assembly.py:build_turret` for bolt-vs-rail), and the additions into
the drive unit by path. All six now qualify and evaluate at any root
that applies the detail — this one, and `cad/check_detail.py` for the
whole machine.

The detail bundle also swaps each 2020 `Box` for its real V-slot
profile, so every other m03 claim is re-evaluated against the hollow
rails too. That is a strict subset of the solid box, so it cannot turn
a cheap-green claim red — only the fastener claims are new information.

Nothing imports this module.
"""

from cad_khana.mechanism.assembly import Assembly

from ..design_systems import DEFAULT_SCHEMA
from .assembly import build_m03
from .detail_variations import apply_default_detail


def assembly(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The m03 unit with the default detail bundle applied — evaluates
    all six foot-bolt claims."""
    return apply_default_detail(build_m03(schema))
