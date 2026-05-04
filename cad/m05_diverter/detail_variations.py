"""Detail-fidelity geometry overrides for the M05 hex tower.

One layer for now:

  * **Extrusions.** Every named 2020 in the frame — through-columns
    (``outer_vert_k``), outer hex edges (``outer_horiz_k``), inner
    hex edges (``inner_horiz_k``), and radials (``radial_k``) — is
    currently a solid ``Box`` in part-local axes. ``default_detail``
    swaps each for a real V-slot profile from ``bd_warehouse`` at
    the same part-local pose; the existing ``PlacedPart.location``
    (which carries the per-floor + per-azimuth rotation) flows
    through untouched.

The 4-floor tower instances every horizontal/radial under a
``f{idx}_{role}__`` floor prefix; through-columns are tower-global
and carry no prefix. ``basename_of(...)`` strips the prefix so the
target match is one set of basenames regardless of floor count.

Fasteners (vertex-bracket bolts, motor mount bolts, joint hardware)
are deferred to a follow-up slice — see ``NEXT_STEPS.md``.

The bbox-derived V-slot helper is intentionally duplicated from
m03's ``detail_variations.py``. When a third module needs detailed
geometry, lift the helper into ``cad-khana`` rather than chaining
cross-module imports.
"""

from __future__ import annotations

import sys
from pathlib import Path

from bd_warehouse.open_builds import VSlotLinearRail
from build123d import Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

sys.path.insert(0, str(Path(__file__).resolve().parent))

from design_systems import basename_of  # noqa: E402


_MEMBER = 20.0
_AXIS_TOL = 0.5

_STRAIGHT_EXTRUSION_BASENAMES = frozenset(
    [f"outer_vert_{k}" for k in range(6)]
    + [f"outer_horiz_{k}" for k in range(6)]
    + [f"inner_horiz_{k}" for k in range(6)]
    + [f"radial_{k}" for k in range(6)]
)


def _vslot_for_axis_aligned_box(part: Part) -> Part:
    """Build a ``VSlotLinearRail("20x20", length=L)`` at the same
    part-local pose as a 20×20×L cheap Box. Long axis auto-detected
    from the part's bounding box; the V-slot cross-section is 4-fold
    symmetric so slot orientation is unconstrained for rendering.

    Operates in part-local frame — the caller's ``PlacedPart.location``
    carries the world placement (per-floor z, per-vertex azimuth)
    through unchanged.
    """
    bb = part.bounding_box()
    sx, sy, sz = bb.size.X, bb.size.Y, bb.size.Z
    cx, cy, cz = bb.center().X, bb.center().Y, bb.center().Z
    short_x = abs(sx - _MEMBER) < _AXIS_TOL
    short_y = abs(sy - _MEMBER) < _AXIS_TOL
    short_z = abs(sz - _MEMBER) < _AXIS_TOL
    if short_y and short_z and not short_x:
        length, axis_rot = sx, Rot(0, 90, 0)        # native +Z → +X
    elif short_x and short_z and not short_y:
        length, axis_rot = sy, Rot(-90, 0, 0)       # native +Z → +Y
    elif short_x and short_y and not short_z:
        length, axis_rot = sz, Rot(0, 0, 0)         # native +Z (no swap)
    else:
        raise ValueError(
            f"detail_variations: bbox ({sx:.1f}, {sy:.1f}, {sz:.1f}) is "
            "not a 20×20×L axis-aligned extrusion"
        )
    return (
        Pos(cx, cy, cz)
        * axis_rot
        * Pos(0, 0, -length / 2)
        * VSlotLinearRail("20x20", length=length)
    )


def default_detail(assembly: Assembly) -> dict[str, Part]:
    """Swap-only override map for every 2020 extrusion in the supplied
    assembly. Names are matched by basename (floor prefix stripped),
    so the same bundle covers ``single_floor.py`` (no prefix) and the
    4-floor tower (``f{idx}_{role}__`` prefix) without enumeration.
    Names whose basename is absent from the target set pass through.
    """
    overrides: dict[str, Part] = {}
    for placed in assembly.parts:
        if basename_of(placed.name) in _STRAIGHT_EXTRUSION_BASENAMES:
            overrides[placed.name] = _vslot_for_axis_aligned_box(placed.part)
    return overrides


def apply_default_detail(assembly: Assembly) -> Assembly:
    """Apply the full detail bundle (extrusion swap only for now).
    Use at render time and any consumer that wants the high-fidelity
    geometry. Cheap-geometry consumers (``assembly.py``'s ``check()``)
    ignore this entirely — V-slot 2020 is a strict subset of solid
    20×20, so any clearance the cheap model passed, the detailed one
    passes too.
    """
    return assembly.with_detailed_geometry(default_detail(assembly))
