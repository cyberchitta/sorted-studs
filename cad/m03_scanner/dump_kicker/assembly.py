"""
M03 scanner — rev-4 turret dump kicker (slice f2 of the 4-station turret).

Standalone bench-mounted 3D L-lever pivoted on a printed clevis bracket
bolted to the slice (a) cross-rail's top T-slot. The lever's hub-end
(in the cam plane Y=0) picks off the slice (f1) shaft cam at heading
180° (the -X side, between hub and IDLE); a -Y cantilever (option a',
2026-05-05) then carries the lifter pad to turret (-30, -25) so the
pad lands under the DUMP-station platform frame's hub-ward extension
tip (slice d) at the new firing heading 270°. At every indexed dwell
a cam lobe rotates into heading 180° (cam follower heading is
independent of firing-station heading; the cam is 4-fold symmetric)
and pushes the follower outward by 5 mm (BASE_R 25 → PEAK_R 30); the
lever rotates around the pivot pin and the lifter pad rises against
the extension tip at DUMP. The lift force acts at cart-frame Y=-100,
40 mm hub-side of the hinge at cart Y=-60, giving a clean single-
direction outer-DOWN torque about the hinge — the platform tilts so
the brick slides off the OUTER edge into the diverter inlet below.
Active-push: at rest (cam at valley) the lifter is retracted, so a
stalled mechanism leaves all platforms level (no spurious dumps); the
per-hinge torsion spring holds them at level when the kicker isn't
engaged.

This slice ships REST geometry only (cam at valley, follower at radial
BASE_R, lifter retracted). Cam-follower kinematics under shaft rotation
— actual lever angle vs cam profile, follower roller geometry, peak-
stroke contact with the platform tab, and verification that cam push
rotates the lever in the direction that LIFTS the pad — are deferred
to slices f3/f4 (per-arm hinge stop + platform tab) and slice f5
(top-level integrate). The static slice locks placement and clearances;
kinematic verification lands when the mating parts arrive.

Members (3 total — 2 printed + 1 stock):
  * kicker_mount  — printed clevis bracket: foot sitting on the cross-
                    rail's top face (z = -200) with two M5 vertical
                    bolt-clearance holes into the rail's top T-slot,
                    plus TWO pillar pieces (one at Y∈[-15,-7], one at
                    Y∈[+7,+15]) rising to a horizontal Ø5.4 pivot-pin
                    bore at (TURRET_DUMP_KICKER_PIVOT_X, *,
                    PIVOT_Z). The pillar gap at Y∈[-7,+7] hosts the
                    lever's hub disc.
  * kicker_lever  — printed 3D L-lever: hub disc at the pivot in cam
                    plane Y∈[-6,+6], hub-arm + hub-end block in the
                    same Y∈[-6,+6] plane (so the cam follower roller
                    contacts the cam OD directly, no Y-bridge), riser
                    rising from the hub disc to a "shoulder" Z above
                    the pillar tops, bridge-Y cantilevering in -Y to
                    Y=-25 (option a', 2026-05-05), and a lifter beam
                    spanning the bridge-Y end through the platform
                    extension drop footprint at the new DUMP heading
                    270°. Top face of bridge-Y / lifter beam at Z = -45
                    = TURRET_DUMP_KICKER_LIFTER_TOP_Z.
  * kicker_pin    — stock M5 round pin (Ø5), axis along ±Y, captures
                    the lever between the two mount pillars. Pin spans
                    pillar A outboard face to pillar B outboard face
                    plus 1 mm overhang each side.

For-standalone-only stubs (1 each, duplicated rather than sibling-
imported per `cad/CLAUDE.md` principle 2):
  * cross_rail_stub — 80 mm slice of the slice (a) cross-rail under
                      the mount, for the bolt-line + foot-tangent
                      check.
  * cam_stub        — slice (f1) cam disc, for the lever-vs-cam
                      clearance check at the hub-end block (now in
                      the cam plane Y=0).

Coordinate frame = turret-local frame (matches `../params.py`):
    origin = hub axis at the cross-rail's top face Z plane
    +X     = operator's right            (parallel to world +X)
    +Y     = away from operator          (parallel to world +Y)
    +Z     = up                          (parallel to world +Z)

Print orientation (both printed parts): bottom face on the build plate,
+Z up (build123d default). The mount has only vertical bolt holes
(clean through) and a horizontal pivot bore (Ø5.4 ceiling bridges fine
in FDM at this diameter). The lever's only horizontal feature is the
same Ø5.4 pivot bore; the rest is rectangular/cylindrical with no
overhangs beyond 90°.
"""

import math
import sys
from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    TURRET_DUMP_CAM_BASE_R,
    TURRET_DUMP_CAM_CENTER_Z,
    TURRET_DUMP_CAM_HEIGHT,
    TURRET_DUMP_CAM_PEAK_R,
    TURRET_DUMP_KICKER_LIFTER_TOP_Z,
    TURRET_DUMP_KICKER_LIFTER_TURRET_X,
    TURRET_DUMP_KICKER_LIFTER_TURRET_Y,
    TURRET_DUMP_KICKER_PIVOT_X,
    TURRET_DUMP_KICKER_PIVOT_Y,
    TURRET_DUMP_KICKER_PIVOT_Z,
    TURRET_FRAME_CROSS_Y,
    TURRET_FRAME_FLOOR_Z,
    TURRET_HINGE_PIN_D,
    WALL_MIN,
)
from design_systems import color_of  # noqa: E402
from materials import material_of  # noqa: E402


# --- Cross-rail interface (private mirrors of slice (a)) ---------------
#
# Cross-rail along ±X at y = TURRET_FRAME_CROSS_Y = 0, top face at
# z = -170. Mount foot bolts into the rail's top T-slot at y = 0.

MEMBER = 20.0                      # 2020 extrusion
RAIL_TOP_Z = TURRET_FRAME_FLOOR_Z + MEMBER / 2                     # = -120 after c7


# --- Pivot pin ---------------------------------------------------------

PIVOT_PIN_D = TURRET_HINGE_PIN_D                                   # = 5
PIVOT_BORE_D = PIVOT_PIN_D + 0.4                                    # = 5.4


# --- Mount geometry (printed clevis bracket on cross-rail) -------------

# Foot dimensions (sits on rail top face). Foot footprint matches the
# pre-corner-move design (X_LEN=60, X∈[-140,-80] with pivot at -110)
# so the slice (g) homing-bracket KICKER_FOOT_X_INNER mirror (= -80)
# stays valid without coordinated update.
MOUNT_FOOT_X = 60.0
MOUNT_FOOT_Y = 30.0    # depth (across rail) — full clevis width
MOUNT_FOOT_Z = 8.0     # vertical foot height
_MOUNT_FOOT_BOTTOM_Z = RAIL_TOP_Z                                  # = -120 after c7
_MOUNT_FOOT_TOP_Z = _MOUNT_FOOT_BOTTOM_Z + MOUNT_FOOT_Z            # = -112
_MOUNT_FOOT_CENTER_Z = (_MOUNT_FOOT_BOTTOM_Z + _MOUNT_FOOT_TOP_Z) / 2

# M5 bolt clearance, two holes along the rail T-slot (at y = 0).
MOUNT_BOLT_HOLE_D = 5.5
MOUNT_BOLT_SPACING_X = 35.0                                        # along rail
_MOUNT_BOLT_HALF_X = MOUNT_BOLT_SPACING_X / 2

# Clevis pillar pieces (one each side of the lever's hub disc). Each
# pillar is MOUNT_PILLAR_Y_THICK thick in Y, with a 1 mm air gap
# between the pillar inner face and the lever hub disc face.
LEVER_T = 12.0                                                     # lever Y-thickness
_LEVER_AIR_GAP_Y = 1.0
_LEVER_HALF_Y = LEVER_T / 2                                        # = 6
_PILLAR_INNER_Y = _LEVER_HALF_Y + _LEVER_AIR_GAP_Y                 # = 7
_PILLAR_OUTER_Y = MOUNT_FOOT_Y / 2                                 # = 15
MOUNT_PILLAR_Y_THICK = _PILLAR_OUTER_Y - _PILLAR_INNER_Y           # = 8
_PILLAR_NEG_Y_CENTER = -(_PILLAR_INNER_Y + MOUNT_PILLAR_Y_THICK / 2)
_PILLAR_POS_Y_CENTER = +(_PILLAR_INNER_Y + MOUNT_PILLAR_Y_THICK / 2)

# Pillar X / Z extents — sized to enclose the pivot bore with WALL_MIN.
MOUNT_PILLAR_X = 25.0
_MOUNT_PILLAR_BOTTOM_Z = _MOUNT_FOOT_TOP_Z                         # = -112 after c7
_PILLAR_CAP_ABOVE_BORE = WALL_MIN + 0.5                            # = 2 mm wall above bore
_MOUNT_PILLAR_TOP_Z = (
    TURRET_DUMP_KICKER_PIVOT_Z + PIVOT_BORE_D / 2 + _PILLAR_CAP_ABOVE_BORE
)                                                                  # ≈ -94.3 after c7
MOUNT_PILLAR_Z = _MOUNT_PILLAR_TOP_Z - _MOUNT_PILLAR_BOTTOM_Z      # ≈ 17.7 after c7
_MOUNT_PILLAR_CENTER_Z = (_MOUNT_PILLAR_BOTTOM_Z + _MOUNT_PILLAR_TOP_Z) / 2


# --- Lever geometry (printed 3D L-lever) -------------------------------

# Lever Y range — single contiguous body in the cam plane (Y∈[-6,+6])
# from hub disc through the riser, then cantilevering in -Y (option a',
# 2026-05-05) past the clevis to the lifter pad at Y=-25.
LEVER_Y_INNER = -_LEVER_HALF_Y                                     # = -6
LEVER_Y_OUTER_AT_HUB = +_LEVER_HALF_Y                              # = +6 (at cam-plane segments)
LEVER_Y_LIFTER = TURRET_DUMP_KICKER_LIFTER_TURRET_Y                # = -25

# Hub disc (at pivot bore). OD chosen to leave WALL_MIN around bore.
LEVER_HUB_DISC_OD = 24.0
_LEVER_HUB_DISC_R = LEVER_HUB_DISC_OD / 2

# Hub-arm beam: rectangular beam in the X-Z plane at Y∈[-6,+6] from
# pivot to hub-end center. Width perpendicular to the arm direction in
# the X-Z plane.
LEVER_ARM_W = 14.0

# Hub-end block: hosts the cam follower roller (slice f5, deferred).
# At REST/DUMP the cam is in dwell orientation with a lobe peak at
# heading 180° (cam OD = PEAK_R = 30 at heading 180°). The block's
# inboard X face must clear PEAK_R + AIR_GAP. The block lives in the
# cam plane (Y∈[-6,+6]); the cam disc Z-band [-81,-69] partially
# overlaps the block Z range, so corner clearance is checked below.
LEVER_HUB_END_X = -45.0
LEVER_HUB_END_Z = TURRET_DUMP_CAM_CENTER_Z                         # = -44 after c7
LEVER_HUB_END_X_LEN = 18.0
LEVER_HUB_END_Z_LEN = 14.0
_HUB_END_INBOARD_X = LEVER_HUB_END_X + LEVER_HUB_END_X_LEN / 2     # = -36

# Riser: vertical post from hub disc center (Z=-99 after c7) up to
# "shoulder" Z above the pillar top, in the cam plane Y∈[-6,+6]. The
# shoulder carries bridge-Y. Riser bottom extends down past the disc
# top so the union with the disc is clean (no overhang gap on the
# disc's upper sides at the riser's outer X corners).
LEVER_RISER_X_CENTER = TURRET_DUMP_KICKER_PIVOT_X                  # = -110
LEVER_RISER_X_LEN = 14.0
LEVER_RISER_BOTTOM_Z = TURRET_DUMP_KICKER_PIVOT_Z                  # = -99 (disc center) after c7
# Shoulder Z = lifter top face minus a half pad thickness keeps the
# riser top, bridge-Y top, and lifter pad top coplanar.
LIFTER_BEAM_Z_LEN = 8.0                                            # = pad Z thickness
LIFTER_BEAM_TOP_Z = TURRET_DUMP_KICKER_LIFTER_TOP_Z                # = -45
LIFTER_BEAM_BOTTOM_Z = LIFTER_BEAM_TOP_Z - LIFTER_BEAM_Z_LEN       # = -53
LIFTER_BEAM_CENTER_Z = (LIFTER_BEAM_TOP_Z + LIFTER_BEAM_BOTTOM_Z) / 2
LEVER_RISER_TOP_Z = LIFTER_BEAM_TOP_Z                              # = -45
LEVER_RISER_Z_LEN = LEVER_RISER_TOP_Z - LEVER_RISER_BOTTOM_Z       # = 54 after c7
_LEVER_RISER_CENTER_Z = (LEVER_RISER_BOTTOM_Z + LEVER_RISER_TOP_Z) / 2

# Bridge-Y: horizontal beam at the lifter Z band, extending from the
# riser top (Y=0) in -Y (option a', 2026-05-05) to the lifter pad zone
# (Y=-25). Cross-section in X-Z = 14 × 8 (matches riser X-extent and
# lifter beam Z-extent).
BRIDGE_Y_X_LEN = LEVER_RISER_X_LEN                                 # = 14
BRIDGE_Y_X_CENTER = LEVER_RISER_X_CENTER                           # = -110
BRIDGE_Y_Y_START = 0.0
BRIDGE_Y_Y_END = LEVER_Y_LIFTER                                    # = -25
BRIDGE_Y_Y_LEN = abs(BRIDGE_Y_Y_END - BRIDGE_Y_Y_START)
_BRIDGE_Y_Y_CENTER = (BRIDGE_Y_Y_START + BRIDGE_Y_Y_END) / 2

# Lifter beam: rectangular pad block sized to cover both the bridge-Y
# endpoint (at X = LEVER_RISER_X_CENTER = -110, where the Y-cantilever
# meets the pad zone) AND the platform extension drop block footprint
# at the new past-hinge position. Option a' (2026-05-05): extension
# drop at the new DUMP (heading 270°, Rz(+180)) lands in turret at
# X ∈ [-39, -21], Y ∈ [-36, -14] (= cart X ∈ [+21, +39], cart Y ∈
# [-111, -89] under Rz(+180) + centroid translate (0, -125)). Beam
# X ∈ [-117, -19] spans from the bridge-Y outboard X face (-117) past
# the extension's hub-most tip with a 2 mm overhang at X-inner. Beam
# Y ∈ [-36, -14] is exactly flush with the extension Y span — 0 mm
# overhang each side, tight; LIFTER_BEAM_Y_LEN = 22 = extension Y
# span. The bridge-Y endpoint at (X=-110, Y=-25) lies inside the beam.
_LIFTER_BEAM_X_INNER = -19.0                                       # +X end (toward hub)
_LIFTER_BEAM_X_OUTER = -117.0                                      # -X end (toward bench)
LIFTER_BEAM_X_LEN = _LIFTER_BEAM_X_INNER - _LIFTER_BEAM_X_OUTER     # = 98
_LIFTER_BEAM_X_CENTER = (_LIFTER_BEAM_X_INNER + _LIFTER_BEAM_X_OUTER) / 2  # = -68
LIFTER_BEAM_Y_LEN = 22.0
_LIFTER_BEAM_Y_CENTER = LEVER_Y_LIFTER                              # = -25


# --- Pivot pin dimensions ----------------------------------------------

# Pin spans the clevis: from pillar A outboard face (Y=-15) to pillar
# B outboard face (Y=+15) plus 1 mm overhang each side.
_PIN_OVERHANG = 1.0
_PIN_Y_INSIDE = -(MOUNT_FOOT_Y / 2) - _PIN_OVERHANG                # = -16
_PIN_Y_OUTSIDE = MOUNT_FOOT_Y / 2 + _PIN_OVERHANG                  # = +16
PIN_LENGTH = _PIN_Y_OUTSIDE - _PIN_Y_INSIDE                        # = 32
_PIN_Y_CENTER = (_PIN_Y_INSIDE + _PIN_Y_OUTSIDE) / 2               # = 0


# --- Sanity asserts ----------------------------------------------------

# Mount: bolt-hole edge to pivot-bore edge wall (X direction). Bolt
# holes (axis ±Z) at x = pivot ± _MOUNT_BOLT_HALF_X. Pivot bore (axis
# ±Y) at pivot X.
_bolt_to_bore_x = (
    _MOUNT_BOLT_HALF_X - MOUNT_BOLT_HOLE_D / 2 - PIVOT_BORE_D / 2
)
assert _bolt_to_bore_x >= WALL_MIN, (
    f"bolt-hole edge to pivot-bore edge wall {_bolt_to_bore_x:.2f} mm "
    f"< WALL_MIN ({WALL_MIN}); raise MOUNT_BOLT_SPACING_X or shrink "
    f"PIVOT_BORE_D"
)

# Mount: bolt-hole edge to foot X-end face wall.
_bolt_to_foot_edge = MOUNT_FOOT_X / 2 - _MOUNT_BOLT_HALF_X - MOUNT_BOLT_HOLE_D / 2
assert _bolt_to_foot_edge >= WALL_MIN, (
    f"bolt-hole to foot X-edge wall {_bolt_to_foot_edge:.2f} mm < "
    f"WALL_MIN ({WALL_MIN}); raise MOUNT_FOOT_X or shrink "
    f"MOUNT_BOLT_SPACING_X"
)

# Mount pillar: pivot-bore-to-pillar-X-edge wall.
_bore_to_pillar_x = MOUNT_PILLAR_X / 2 - PIVOT_BORE_D / 2
assert _bore_to_pillar_x >= WALL_MIN, (
    f"pivot-bore to pillar X-edge wall {_bore_to_pillar_x:.2f} mm < "
    f"WALL_MIN ({WALL_MIN}); raise MOUNT_PILLAR_X"
)

# Mount pillar: pivot-bore-to-pillar-top wall (Z direction).
_bore_to_pillar_top = (
    _MOUNT_PILLAR_TOP_Z - TURRET_DUMP_KICKER_PIVOT_Z - PIVOT_BORE_D / 2
)
assert _bore_to_pillar_top >= WALL_MIN, (
    f"pivot-bore to pillar top wall {_bore_to_pillar_top:.2f} mm < "
    f"WALL_MIN ({WALL_MIN}); raise _PILLAR_CAP_ABOVE_BORE"
)

# Clevis pillar Y-thickness ≥ WALL_MIN.
assert MOUNT_PILLAR_Y_THICK >= WALL_MIN, (
    f"clevis pillar Y-thickness {MOUNT_PILLAR_Y_THICK:.2f} mm < "
    f"WALL_MIN ({WALL_MIN}); widen MOUNT_FOOT_Y or shrink LEVER_T"
)

# Lever: hub-disc-to-bore wall.
_disc_wall = _LEVER_HUB_DISC_R - PIVOT_BORE_D / 2
assert _disc_wall >= WALL_MIN, (
    f"hub-disc wall around pivot bore {_disc_wall:.2f} mm < WALL_MIN "
    f"({WALL_MIN}); raise LEVER_HUB_DISC_OD"
)

# Mount-lever Y air gap.
assert _LEVER_AIR_GAP_Y > 0, (
    f"lever-mount Y air gap {_LEVER_AIR_GAP_Y} mm ≤ 0"
)

# Lifter pad top Z must clear the platform underside (slice d bracket
# bottom Z = -27.3 = -PLATFORM_ACRYLIC_T - BOND_GAP - BRACKET_BLOCK_Z).
_PLATFORM_BRACKET_BOTTOM_Z = -27.3
_lifter_to_platform = _PLATFORM_BRACKET_BOTTOM_Z - TURRET_DUMP_KICKER_LIFTER_TOP_Z
assert _lifter_to_platform > 0, (
    f"lifter top Z ({TURRET_DUMP_KICKER_LIFTER_TOP_Z}) ≥ platform "
    f"bracket bottom ({_PLATFORM_BRACKET_BOTTOM_Z}); rest position "
    f"would crash the platform"
)

# Lever hub-end block clears the cam OD at the dwell orientation (cam
# peak at heading 180°, cam_r = PEAK_R = 30 at Y=0). Block inboard X
# face at _HUB_END_INBOARD_X, Y∈[-LEVER_HALF_Y, +LEVER_HALF_Y]. At each
# corner (inboard X, ±LEVER_HALF_Y) check radial > cam_r at that
# heading.
_HALF_PEAK_DEG = 10.0   # mirror of slice (f1) LOBE_PEAK_DEG / 2
_RAMP_DEG = 10.0        # mirror of slice (f1) LOBE_RAMP_DEG


def _cam_r_at(theta_deg: float) -> float:
    delta = (theta_deg + 360.0) % 90.0
    if delta > 45.0:
        delta = 90.0 - delta
    if delta <= _HALF_PEAK_DEG:
        return TURRET_DUMP_CAM_PEAK_R
    if delta <= _HALF_PEAK_DEG + _RAMP_DEG:
        return TURRET_DUMP_CAM_PEAK_R - (delta - _HALF_PEAK_DEG) / _RAMP_DEG * (
            TURRET_DUMP_CAM_PEAK_R - TURRET_DUMP_CAM_BASE_R
        )
    return TURRET_DUMP_CAM_BASE_R


for _y in (-_LEVER_HALF_Y, +_LEVER_HALF_Y):
    _r = math.hypot(_HUB_END_INBOARD_X, _y)
    _heading_deg = math.degrees(math.atan2(_y, _HUB_END_INBOARD_X))
    _cam_r_here = _cam_r_at(_heading_deg)
    assert _r > _cam_r_here, (
        f"hub-end inboard corner at ({_HUB_END_INBOARD_X}, {_y}) radial "
        f"{_r:.2f} mm ≤ cam OD {_cam_r_here:.2f} mm at heading "
        f"{_heading_deg:.1f}°; lever crashes the cam. Move "
        f"LEVER_HUB_END_X further outboard or shrink LEVER_HUB_END_X_LEN."
    )


# Lifter beam covers the platform extension drop-block footprint. Drop
# block in turret-local at DUMP (option a', 2026-05-05): cart (X=+30,
# Y=-100) ± (9, 11) under Rz(+180) + centroid translate (0, -125) maps
# to turret X ∈ [-39, -21] (= -cart_x), Y ∈ [-36, -14] (= -cart_y -
# 125). Lifter beam X ∈ [-117, -19], Y ∈ [-36, -14]. Drop X inside
# beam X with 2 mm overhang at X-inner (-19 vs -21) and 78 mm overhang
# at X-outer (-117 vs -39). Drop Y exactly flush with beam Y (0 mm
# overhang each side — tight). Beam wraps the bridge-Y endpoint at
# X ∈ [-117, -103]. Lever arm = 40 mm hub-side of the hinge — clean
# single-direction outer-DOWN torque (no competing brick-side push;
# see TURRET_DUMP_KICKER_LIFTER_TURRET_X comment in `../params.py`).
_EXT_DROP_TURRET_X_INNER_AT_DUMP = -21.0   # drop hub-most (least negative X)
_EXT_DROP_TURRET_X_OUTER_AT_DUMP = -39.0   # drop bench-most (most negative X)
_EXT_DROP_TURRET_Y_NEG_AT_DUMP = -36.0
_EXT_DROP_TURRET_Y_POS_AT_DUMP = -14.0
assert _LIFTER_BEAM_X_INNER >= _EXT_DROP_TURRET_X_INNER_AT_DUMP + 2.0 - 1e-9, (
    f"lifter beam X-inner {_LIFTER_BEAM_X_INNER} too close to extension "
    f"drop X-inner {_EXT_DROP_TURRET_X_INNER_AT_DUMP} (overhang < 2 mm)"
)
assert _LIFTER_BEAM_X_OUTER <= _EXT_DROP_TURRET_X_OUTER_AT_DUMP, (
    f"lifter beam X-outer {_LIFTER_BEAM_X_OUTER} doesn't reach extension "
    f"drop X-outer {_EXT_DROP_TURRET_X_OUTER_AT_DUMP}; bridge-Y endpoint "
    f"may be unsupported"
)
_LIFTER_BEAM_Y_INNER = _LIFTER_BEAM_Y_CENTER - LIFTER_BEAM_Y_LEN / 2
_LIFTER_BEAM_Y_OUTER = _LIFTER_BEAM_Y_CENTER + LIFTER_BEAM_Y_LEN / 2
assert _LIFTER_BEAM_Y_INNER <= _EXT_DROP_TURRET_Y_NEG_AT_DUMP + 1e-9, (
    f"lifter beam Y-inner {_LIFTER_BEAM_Y_INNER} > extension drop Y-neg "
    f"{_EXT_DROP_TURRET_Y_NEG_AT_DUMP} (beam must reach ≤ extension's "
    f"more-negative Y face)"
)
assert _LIFTER_BEAM_Y_OUTER >= _EXT_DROP_TURRET_Y_POS_AT_DUMP - 1e-9, (
    f"lifter beam Y-outer {_LIFTER_BEAM_Y_OUTER} < extension drop Y-pos "
    f"{_EXT_DROP_TURRET_Y_POS_AT_DUMP} (beam must reach ≥ extension's "
    f"less-negative Y face)"
)


# --- Part functions ----------------------------------------------------

def kicker_mount():
    """Printed clevis bracket: foot + two pillars with a horizontal
    pivot bore through both. Centered on (TURRET_DUMP_KICKER_PIVOT_X,
    0, *) in turret-local; bottom face on cross-rail top
    (z = RAIL_TOP_Z)."""
    foot = (
        Pos(TURRET_DUMP_KICKER_PIVOT_X, 0, _MOUNT_FOOT_CENTER_Z)
        * Box(MOUNT_FOOT_X, MOUNT_FOOT_Y, MOUNT_FOOT_Z)
    )
    # Two M5 vertical bolt clearance holes at y = 0, x = pivot ± half.
    for sx in (-1, 1):
        foot -= (
            Pos(
                TURRET_DUMP_KICKER_PIVOT_X + sx * _MOUNT_BOLT_HALF_X,
                0,
                _MOUNT_FOOT_CENTER_Z,
            )
            * Cylinder(MOUNT_BOLT_HOLE_D / 2, MOUNT_FOOT_Z * 3)
        )

    pillar_neg = (
        Pos(
            TURRET_DUMP_KICKER_PIVOT_X,
            _PILLAR_NEG_Y_CENTER,
            _MOUNT_PILLAR_CENTER_Z,
        )
        * Box(MOUNT_PILLAR_X, MOUNT_PILLAR_Y_THICK, MOUNT_PILLAR_Z)
    )
    pillar_pos = (
        Pos(
            TURRET_DUMP_KICKER_PIVOT_X,
            _PILLAR_POS_Y_CENTER,
            _MOUNT_PILLAR_CENTER_Z,
        )
        * Box(MOUNT_PILLAR_X, MOUNT_PILLAR_Y_THICK, MOUNT_PILLAR_Z)
    )

    body = foot + pillar_neg + pillar_pos

    # Pivot bore — axis along ±Y, drilled through both pillars.
    body -= (
        Pos(
            TURRET_DUMP_KICKER_PIVOT_X,
            0,
            TURRET_DUMP_KICKER_PIVOT_Z,
        )
        * Rot(90, 0, 0)
        * Cylinder(PIVOT_BORE_D / 2, MOUNT_FOOT_Y * 3)
    )

    return body


def kicker_lever():
    """Printed 3D L-lever: hub disc + hub-arm + hub-end block (in cam
    plane Y∈[-6,+6]) + riser (vertical from disc to lifter Z) +
    bridge-Y (horizontal beam in -Y to Y=-25, option a' 2026-05-05)
    + lifter beam (horizontal beam wrapping the bridge endpoint and
    spanning the platform extension drop footprint at the new DUMP
    heading 270°, top face at LIFTER_TOP_Z)."""
    # Hub disc at pivot. Axis along ±Y.
    hub_disc = (
        Pos(
            TURRET_DUMP_KICKER_PIVOT_X,
            0,
            TURRET_DUMP_KICKER_PIVOT_Z,
        )
        * Rot(90, 0, 0)
        * Cylinder(_LEVER_HUB_DISC_R, LEVER_T)
    )

    # Hub arm: rectangular beam in X-Z plane at Y=0 (centered),
    # Y-thickness LEVER_T. Length spans pivot to hub-end center.
    hub_arm = _hub_arm_beam(
        x0=TURRET_DUMP_KICKER_PIVOT_X,
        z0=TURRET_DUMP_KICKER_PIVOT_Z,
        x1=LEVER_HUB_END_X,
        z1=LEVER_HUB_END_Z,
    )

    hub_end_block = (
        Pos(LEVER_HUB_END_X, 0, LEVER_HUB_END_Z)
        * Box(LEVER_HUB_END_X_LEN, LEVER_T, LEVER_HUB_END_Z_LEN)
    )

    # Riser: vertical block from disc center up to lifter Z, in cam
    # plane Y∈[-6,+6].
    riser = (
        Pos(LEVER_RISER_X_CENTER, 0, _LEVER_RISER_CENTER_Z)
        * Box(LEVER_RISER_X_LEN, LEVER_T, LEVER_RISER_Z_LEN)
    )

    # Bridge-Y: horizontal beam at lifter Z, X-thickness 14, Y from 0
    # to -25 (option a', 2026-05-05), Z-thickness LIFTER_BEAM_Z_LEN.
    # Cross-section in X-Z = 14 × 8; runs in Y.
    bridge_y = (
        Pos(BRIDGE_Y_X_CENTER, _BRIDGE_Y_Y_CENTER, LIFTER_BEAM_CENTER_Z)
        * Box(BRIDGE_Y_X_LEN, BRIDGE_Y_Y_LEN, LIFTER_BEAM_Z_LEN)
    )

    # Lifter beam: horizontal beam at Y=-25 (centered), Y-thickness 22,
    # X from -19 (past extension hub-most tip with 2 mm overhang) to
    # -117 (wraps the bridge-Y endpoint). Top face at LIFTER_TOP_Z.
    lifter_beam = (
        Pos(_LIFTER_BEAM_X_CENTER, _LIFTER_BEAM_Y_CENTER, LIFTER_BEAM_CENTER_Z)
        * Box(LIFTER_BEAM_X_LEN, LIFTER_BEAM_Y_LEN, LIFTER_BEAM_Z_LEN)
    )

    body = hub_disc + hub_arm + hub_end_block + riser + bridge_y + lifter_beam

    # Pivot bore through hub disc — axis along ±Y.
    body -= (
        Pos(
            TURRET_DUMP_KICKER_PIVOT_X,
            0,
            TURRET_DUMP_KICKER_PIVOT_Z,
        )
        * Rot(90, 0, 0)
        * Cylinder(PIVOT_BORE_D / 2, LEVER_T * 3)
    )

    return body


def _hub_arm_beam(x0: float, z0: float, x1: float, z1: float):
    """Rectangular hub-arm beam in the X-Z plane at Y=0 (centered),
    Y-thickness LEVER_T, centered at the midpoint of (x0, z0)→(x1, z1)
    and rotated to align its long axis along that direction. Width
    LEVER_ARM_W perpendicular to the arm in the X-Z plane.

    Box default long axis +X. Ry(α) maps +X = (1,0,0) onto
    (cos α, 0, -sin α); to land on (dx/L, 0, dz/L) we need
    cos α = dx/L and sin α = -dz/L, giving α = atan2(-dz, dx).
    """
    dx = x1 - x0
    dz = z1 - z0
    arm_len = math.hypot(dx, dz)
    arm_angle_deg = math.degrees(math.atan2(-dz, dx))
    return (
        Pos((x0 + x1) / 2, 0, (z0 + z1) / 2)
        * Rot(0, arm_angle_deg, 0)
        * Box(arm_len, LEVER_T, LEVER_ARM_W)
    )


def kicker_pin(length: float = PIN_LENGTH, d: float = PIVOT_PIN_D):
    """Stock M5 round pivot pin. Default Cylinder axis +Z; caller
    rotates with Rot(90, 0, 0) to align with the pivot bore (axis ±Y)."""
    return Cylinder(d / 2, length)


# --- Stubs for the standalone (duplicated, not imported) ---------------

def _cross_rail_stub(length: float = MOUNT_FOOT_X + 30.0):
    """Slice (a) cross-rail stub under the mount."""
    return (
        Pos(TURRET_DUMP_KICKER_PIVOT_X, TURRET_FRAME_CROSS_Y, TURRET_FRAME_FLOOR_Z)
        * Box(length, MEMBER, MEMBER)
    )


def _cam_stub():
    """Slice (f1) cam disc geometry — used here only for the lever-vs-
    cam clearance check at REST. 4-lobed profile rebuilt locally per
    cad/CLAUDE.md principle 2 (no sibling imports). Cam pinned to the
    dwell orientation: a lobe peak sits at heading 180° (the follower
    face)."""
    from build123d import Polygon, extrude

    HALF_PEAK = 10.0
    RAMP = 10.0
    BASE_R = TURRET_DUMP_CAM_BASE_R
    PEAK_R = TURRET_DUMP_CAM_PEAK_R

    def _r_at(theta_deg: float) -> float:
        delta = (theta_deg + 360.0) % 90.0
        if delta > 45.0:
            delta = 90.0 - delta
        if delta <= HALF_PEAK:
            return PEAK_R
        if delta <= HALF_PEAK + RAMP:
            ramp_t = (delta - HALF_PEAK) / RAMP
            return PEAK_R - ramp_t * (PEAK_R - BASE_R)
        return BASE_R

    n = 360
    pts = []
    for i in range(n):
        theta_deg = i
        r = _r_at(theta_deg)
        pts.append((r * math.cos(math.radians(theta_deg)),
                    r * math.sin(math.radians(theta_deg))))

    profile = Polygon(*pts, align=None)
    body = extrude(profile, amount=TURRET_DUMP_CAM_HEIGHT)
    body = Pos(0, 0, -TURRET_DUMP_CAM_HEIGHT / 2) * body
    return Pos(0, 0, TURRET_DUMP_CAM_CENTER_Z) * body


# --- Assembly builder --------------------------------------------------

KICKER_PART_NAMES = ("kicker_mount", "kicker_lever", "kicker_pin")


def add_turret_dump_kicker(a: Assembly, base=Pos(0, 0, 0)) -> Assembly:
    """Place the kicker mount + lever + pin in turret-local. `base`
    (default: identity) is pre-multiplied to lift the assembly into a
    parent frame (e.g. world)."""
    a = a.add(
        "kicker_mount", kicker_mount(), location=base,
        material=material_of("kicker_mount"), color=color_of("kicker_mount"),
    )
    a = a.add(
        "kicker_lever", kicker_lever(), location=base,
        material=material_of("kicker_lever"), color=color_of("kicker_lever"),
    )
    # Pin axis = pivot bore axis = ±Y. Rotate default Cylinder (+Z) to
    # ±Y, then place centered along the bore axis.
    a = a.add(
        "kicker_pin",
        kicker_pin(),
        location=base * Pos(
            TURRET_DUMP_KICKER_PIVOT_X,
            _PIN_Y_CENTER,
            TURRET_DUMP_KICKER_PIVOT_Z,
        ) * Rot(90, 0, 0),
        material=material_of("kicker_pin"),
    )
    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_turret_dump_kicker(Assembly())

# Stubs for standalone clearance checks.
assembly = assembly.add(
    "cross_rail_stub", _cross_rail_stub(), material="aluminium_anodized"
)
assembly = assembly.add("cam_stub", _cam_stub(), material="pla_matte")

# All pairwise no-interference between kicker parts and stubs. The pin
# tangent-fits the bores (zero-overlap in the model); foot tangent-sits
# on the rail top (also zero-overlap).
#
# Slice c7 (2026-05-05) note: the cam rise from -64 → -44 lifted the
# cam Z range [-50, -38] into the lifter beam's Z slab [-53, -45],
# exposing a small overlap where the lifter beam's hub-ward inner
# corner clips the cam BASE_R=25 footprint. Option a' (2026-05-05)
# swung the lifter beam from Y=+30 to Y=-25; the clipping corner moved
# from (X=-12, Y=+19) at heading ~122° to (X=-19, Y=-14) at heading
# ~216° (still in the BASE_R band — distance √(19²+14²) ≈ 23.6 mm
# < BASE_R=25). The clip persists at the new heading; resolving it
# still requires a kicker redesign (notched beam, smaller cam BASE_R,
# or reworked rest-position kinematics). Until then, the overlap is
# documented via `assert_interference` so it stays visible without
# crashing the build, and disappears (failing this assertion) the
# moment the redesign lands.
_ALL = list(KICKER_PART_NAMES) + ["cross_rail_stub", "cam_stub"]
_KNOWN_OVERLAPS = {("kicker_lever", "cam_stub")}
for _i, _a in enumerate(_ALL):
    for _b in _ALL[_i + 1:]:
        if (_a, _b) in _KNOWN_OVERLAPS or (_b, _a) in _KNOWN_OVERLAPS:
            assembly = assembly.assert_interference(
                _a,
                _b,
                reason=(
                    "c7 cam rise (-64 → -44 center) lifted cam Z into the "
                    "lifter-beam slab; option a' moved the clipping corner "
                    "from heading ~122° to ~216° but the BASE_R clip "
                    "persists. Redesign queued (notched beam or smaller "
                    "cam BASE_R)."
                ),
            )
        else:
            assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    inspect(
        kicker_mount(),
        method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
        out="outputs",
        name="kicker_mount",
    )
    inspect(
        kicker_lever(),
        method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
        out="outputs",
        name="kicker_lever",
    )
