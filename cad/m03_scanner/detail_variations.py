"""Detail-fidelity geometry overrides for the M03 scanner.

Two layers compose at the call site:

  * **Extrusions.** Each named 2020 stub is currently a solid
    ``Box(20, 20, L)`` — the right primitive for the geometric-
    iteration loop (interference / clearance) but visually
    indistinguishable from MDF. ``default_detail`` swaps each for
    a real V-slot profile from ``bd_warehouse`` at the same pose.
    Straight rails are detected from their axis-aligned bbox; after
    the Phase-2 integrate (2026-05-06) every 2020 in the m03 unit
    is axis-aligned (the rev-3 mitered ``brace_bc`` was demolished
    with the rest of the rig T-frame), so no `body_factory` hook is
    needed.
  * **Fasteners.** Visible bolts at high-value joints. Currently the
    two M5 SHCS that bolt the drive bracket foot to the cross-rail's
    top T-slot. Added as new placed parts (no cheap-model counterpart).

Use ``apply_default_detail(assembly)`` in render-time consumers — it
runs both layers.

GT2 metal pulleys (``drive_pulley_shaft``, ``drive_pulley_motor``)
are deliberately not in the extrusion target set — they're cylindrical
drive parts modelled as cheap stand-ins, not extrusions.

The per-fastener no-interference assertions live at the level that
owns each side of the joint: bolt-vs-bracket and bolt-vs-motor in
``drive/assembly.py``'s static builder, bolt-vs-rail in
``assembly.py``'s ``build_turret`` where the drive and frame subtrees
meet. cad-khana skips assertions whose parts are absent from the run
(``passed: null``) instead of KeyErroring, so the cheap checks tolerate
them; a detail-applied ``check()`` evaluates them **at any root**,
because the bolts are keyed into the drive unit rather than added bare
at the root the override was applied to (see ``_drive_foot_bolts``).
"""

from __future__ import annotations


from bd_warehouse.fastener import SocketHeadCapScrew
from bd_warehouse.open_builds import VSlotLinearRail
from build123d import Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly, DetailOverride


from .drive.assembly import BRACKET_BOLT_XS, BRACKET_FOOT_Z
from .params import (
    TURRET_FRAME_CROSS_Y,
    TURRET_FRAME_FLOOR_Z,
)


_MEMBER = 20.0
_AXIS_TOL = 0.5

_STRAIGHT_EXTRUSION_TARGETS = frozenset(
    {
        # Phase-2 integrated rig (`arch/`)
        "arch_post_pos_x", "arch_post_neg_x",
        "arch_stub_pos_x", "arch_stub_neg_x",
        "rail_d",
        # Turret floor frame (`frame/`)
        "floor_back_x", "floor_left_y", "floor_right_y",
        "floor_cross_x",
        "floor_outrig_pos_x", "floor_outrig_neg_x",
    }
)


def _vslot_for_axis_aligned_box(part: Part) -> Part:
    """Build a ``VSlotLinearRail("20x20", length=L)`` at the same
    axis-aligned pose as a 20×20×L cheap Box. Long axis auto-detected
    from the part's bounding box; the V-slot cross-section is 4-fold
    symmetric so slot orientation is unconstrained for rendering.
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
    assembly. Straight rails are matched by leaf name (the target set
    is placement-agnostic) but keyed by full path — the form
    ``with_detailed_geometry`` addresses. Leaves absent from the
    assembly are silently skipped, so the bundle stays robust to slice
    changes that drop or rename a member.
    """
    overrides: dict[str, Part] = {}
    for placed in assembly.placed_parts:
        if placed.name.rsplit(".", 1)[-1] in _STRAIGHT_EXTRUSION_TARGETS:
            overrides[placed.name] = _vslot_for_axis_aligned_box(placed.part)
    return overrides


# --- Fasteners ---------------------------------------------------------

# Foot-bolt geometry. The drive bracket foot top face sits at z = -162
# (rail top z = -170 + foot thickness 8). M5×16 SHCS heads land on the
# foot top face; the threaded shank descends through the foot (8 mm)
# and into the rail's top T-slot (8 mm of engagement) where a t-nut
# captures it. Bolt origin (head/shank interface in bd_warehouse's
# canonical pose) lands on z = -162. Default Cylinder/SHCS axis is +Z,
# matches the bolt's "head up" orientation here without rotation.
_FOOT_BOLT_HEAD_Z = TURRET_FRAME_FLOOR_Z + _MEMBER / 2 + BRACKET_FOOT_Z   # = -162
_FOOT_BOLT_Y = TURRET_FRAME_CROSS_Y                                        # bolts on the cross-rail centerline
# Bolt X positions come from the bracket's own hole pattern
# (`BRACKET_BOLT_XS`, inner then outer) — one source, no mirror.

# Bolt length: 8 mm foot pass-through + 6 mm slot engagement = 14 mm.
# Stock M5×14 SHCS. The 6 mm slot-side reach is sized to fit within the
# V-slot opening's clear depth (no contact with the rib region) — a
# real assembly would catch a t-nut here (not modelled). The bolt is
# therefore zero-overlap against both the bracket clearance hole and
# the V-slot rail's hollow interior.
_FOOT_BOLT_LENGTH_MM = 14.0


def _drive_foot_bolts(unit_path: str) -> dict[str, DetailOverride]:
    """Two M5×14 SHCS bolts at the drive bracket foot, keyed into the
    static drive unit at ``unit_path`` and located in that unit's own
    frame.

    The drive unit sits at identity inside the turret, so its frame is
    turret-local — the same frame ``BRACKET_BOLT_XS`` and the two
    module constants above are written in. Keying the addition by the
    unit's path (rather than bare) is what lets the unit's own bolt
    claims evaluate at *any* composed root: the claims qualify to
    ``<unit path>.drive_foot_bolt_*`` there, and so does the addition.
    A bare key would land the bolts at whatever root the override was
    applied to and leave those claims skipped forever.
    """
    prefix = f"{unit_path}." if unit_path else ""
    overrides: dict[str, DetailOverride] = {}
    for bolt_x, label in zip(BRACKET_BOLT_XS, ("inner", "outer")):
        overrides[f"{prefix}drive_foot_bolt_{label}"] = DetailOverride(
            part=SocketHeadCapScrew(
                size="M5-0.8",
                length=_FOOT_BOLT_LENGTH_MM,
                fastener_type="iso4762",
                simple=True,
            ),
            location=Pos(bolt_x, _FOOT_BOLT_Y, _FOOT_BOLT_HEAD_Z),
            material="steel",
        )
    return overrides


def apply_default_detail(assembly: Assembly) -> Assembly:
    """Apply the full detail bundle (extrusion swap + fastener adds).
    Use at render time and any consumer that wants the high-fidelity
    geometry. Cheap-geometry consumers (``assembly.py``'s ``check()``)
    ignore this entirely. The fastener interference assertions are
    declared in ``drive/assembly.py`` beside the joint.

    Root-agnostic: the static drive unit is located by its
    ``drive_bracket`` leaf, so the same bundle applies to the drive
    standalone, the m03 unit, or the whole machine, and the bolts land
    in the drive unit every time.
    """
    overrides: dict[str, object] = dict(default_detail(assembly))

    # Drive-bracket foot bolts — only emitted if the bracket itself is
    # in the assembly (lets a sub-assembly slice that omits the drive
    # use the same helper without a missing-key crash). The bracket's
    # dotted prefix *is* the drive unit's path from this root.
    bracket_path = next(
        (
            p.name
            for p in assembly.placed_parts
            if p.name.rsplit(".", 1)[-1] == "drive_bracket"
        ),
        None,
    )
    if bracket_path is not None:
        unit_path = bracket_path.rpartition(".")[0]
        overrides.update(_drive_foot_bolts(unit_path))

    return assembly.with_detailed_geometry(overrides)
