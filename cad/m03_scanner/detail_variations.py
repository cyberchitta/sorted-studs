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

The per-fastener no-interference assertions live in
``drive/assembly.py``, where the bolted joint lives — cad-khana
(2043f01) skips assertions whose parts are absent from the run
(``passed: null``) instead of KeyErroring, so the cheap standalone
check tolerates them and a detail-applied ``check()`` evaluates them.
"""

from __future__ import annotations


from bd_warehouse.fastener import SocketHeadCapScrew
from bd_warehouse.open_builds import VSlotLinearRail
from build123d import Location, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly, DetailOverride


from .drive.assembly import (
    BRACKET_BOLT_SPACING_X,
    BRACKET_FOOT_Z,
    BRACKET_WALL_T,
    BRACKET_WALL_X_MIN,
)
from .params import (
    TURRET_ARM_R,
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
_FOOT_BOLT_CTR_X = BRACKET_WALL_X_MIN + BRACKET_WALL_T / 2                # bracket foot centerline
_FOOT_BOLT_DX = BRACKET_BOLT_SPACING_X / 2                                 # = 17.5
_FOOT_BOLT_Y = TURRET_FRAME_CROSS_Y                                        # bolts on the cross-rail centerline

# Bolt length: 8 mm foot pass-through + 6 mm slot engagement = 14 mm.
# Stock M5×14 SHCS. The 6 mm slot-side reach is sized to fit within the
# V-slot opening's clear depth (no contact with the rib region) — a
# real assembly would catch a t-nut here (not modelled). The bolt is
# therefore zero-overlap against both the bracket clearance hole and
# the V-slot rail's hollow interior.
_FOOT_BOLT_LENGTH_MM = 14.0


def _drive_foot_bolts(
    drive_bracket_location: Location,
) -> dict[str, DetailOverride]:
    """Two M5×14 SHCS bolts at the drive bracket foot. The bolts ride
    in the same Location as ``drive_bracket`` so the
    ``Pos(0, -ARM_R, 0)`` turret-base placement carries through
    without duplication here.
    """
    overrides: dict[str, DetailOverride] = {}
    for sx, label in ((-1, "inner"), (+1, "outer")):
        bolt = (
            Pos(_FOOT_BOLT_CTR_X + sx * _FOOT_BOLT_DX, _FOOT_BOLT_Y, _FOOT_BOLT_HEAD_Z)
            * SocketHeadCapScrew(
                size="M5-0.8",
                length=_FOOT_BOLT_LENGTH_MM,
                fastener_type="iso4762",
                simple=True,
            )
        )
        overrides[f"drive_foot_bolt_{label}"] = DetailOverride(
            part=bolt,
            location=drive_bracket_location,
            material="steel",
        )
    return overrides


def apply_default_detail(assembly: Assembly) -> Assembly:
    """Apply the full detail bundle (extrusion swap + fastener adds).
    Use at render time and any consumer that wants the high-fidelity
    geometry. Cheap-geometry consumers (``assembly.py``'s ``check()``)
    ignore this entirely. The fastener interference assertions are
    declared in ``drive/assembly.py`` beside the joint.
    """
    # Leaf-keyed lookup: both leaves are unique across the m03 tree,
    # and matching by leaf keeps this helper placement-agnostic.
    parts_by_leaf = {
        p.name.rsplit(".", 1)[-1]: p for p in assembly.placed_parts
    }
    overrides: dict[str, object] = dict(default_detail(assembly))

    # Drive-bracket foot bolts — only emitted if the bracket itself is
    # in the assembly (lets a sub-assembly slice that omits the drive
    # use the same helper without a missing-key crash).
    if "drive_bracket" in parts_by_leaf:
        overrides.update(
            _drive_foot_bolts(parts_by_leaf["drive_bracket"].location)
        )

    return assembly.with_detailed_geometry(overrides)
