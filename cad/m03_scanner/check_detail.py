"""
M03 scanner — detail-tier check module.

A **check module**: it imports the product assembly, applies the
render-time detail bundle, and exposes the result so `khana check`
evaluates the claims that only a detailed build can answer.

    .venv/bin/khana check cad/m03_scanner/check_detail.py
    # → cad/m03_scanner/outputs/check_detail/mechanism.json
    .venv/bin/khana check cad/m03_scanner/check_detail.py:build_drive_detailed
    # → cad/m03_scanner/outputs/check_detail-build_drive_detailed/mechanism.json

The cheap `assembly.py` check (and every parent that composes it,
`cad/assembly.py` included) skips the drive-foot M5 bolt assertions as
`passed: null`: the bolts exist only once
`detail_variations.apply_default_detail` adds them, and nothing else
runs a check on that tree. The rail side of the joint is declared in
`assembly.py:build_m03`, the bracket side in
`drive/assembly.py:build_turret_drive_standalone` — hence two
factories here, one per declaring level.

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
from .drive.assembly import build_turret_drive_standalone


def assembly(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The m03 unit with the default detail bundle applied — evaluates
    the rail-side foot-bolt claims."""
    return apply_default_detail(build_m03(schema))


def build_drive_detailed() -> Assembly:
    """The drive standalone fixture with the default detail bundle
    applied — evaluates the bracket-side foot-bolt claims."""
    return apply_default_detail(build_turret_drive_standalone())
