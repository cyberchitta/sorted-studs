"""
Whole machine — detail-tier check module.

A **check module**: it composes the top-level assembly and applies
m03's detail bundle to the *whole-machine* root, so the claims that
only a detailed build can answer are evaluated in the frame the
machine is actually assembled in.

    .venv/bin/khana check cad/check_detail.py
    # → cad/outputs/check_detail/mechanism.json

This is the expensive one (~3 min, 34k assertions) — the cheap
`khana check cad/assembly.py` covers everything here except the six
foot-bolt claims, and reports them as `absent_part` skips. Run this
after a change to the drive-foot joint, the detail bundle, or
`detail_variations`' keying; not on every slice.

**Why it exists.** A detail addition lands at the level its key names,
so `apply_default_detail` keys the two M5 foot bolts into the static
drive unit (`m03.turret.drive` from here). The claims about them are
declared one and two levels down — bolt-vs-bracket and bolt-vs-motor
in `m03_scanner/drive/assembly.py`, bolt-vs-rail in
`m03_scanner/assembly.py:build_turret` — and qualify up to this root.
`skipped_counts` going to zero here is the check: a nonzero
`absent_part` means an addition was keyed to the wrong level, or a
claim was parked somewhere a composed run never reaches.

Only m03 carries fastener additions. m05's bundle is swap-only (V-slot
profiles for cheap boxes), so applying it would cost tessellation and
answer no new claim; it is deliberately not applied here.

Nothing imports this module.
"""

from cad_khana.mechanism.assembly import Assembly

from .assembly import build_top
from .design_systems import DEFAULT_SCHEMA
from .m03_scanner.detail_variations import apply_default_detail


def assembly(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The whole machine with m03's detail bundle applied at this
    root — evaluates all six foot-bolt claims in machine coordinates."""
    return apply_default_detail(build_top(schema))
