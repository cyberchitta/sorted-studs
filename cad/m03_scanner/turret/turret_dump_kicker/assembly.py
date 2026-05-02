"""
M03 scanner — rev-4 turret dump kicker (slice f2 of the 4-station turret).

Standalone bench-mounted L-lever pivoted on a printed bracket bolted to
the slice (a) cross-rail's top T-slot. The lever's hub-end picks off
the slice (f1) shaft cam at heading 180° (the -X side, between hub and
DUMP); its lifter-end carries a flat pad below the DUMP-station
platform. At every indexed dwell a cam lobe rotates into heading 180°
and pushes the follower outward by 5 mm (BASE_R 25 → PEAK_R 30); the
lever rotates around the pivot pin and the lifter pad rises to push up
on a tab on the DUMP platform's underside, tilting the platform so the
brick slides off into the diverter inlet below. Active-push: at rest
(cam at valley) the lifter is retracted, so a stalled mechanism leaves
all platforms level (no spurious dumps).

This slice ships REST geometry only (cam at valley, follower at radial
BASE_R, lifter retracted). Cam-follower kinematics under shaft rotation
— the actual lever angle vs cam profile, follower roller geometry,
peak-stroke contact with the platform tab — are deferred to slices
f3/f4 (per-arm hinge stop + platform tab) and slice f5 (top-level
integrate). Same precedent as slice f1, which shipped the cam without
a follower: the static slice locks placement and clearances, kinematic
verification lands when the mating parts arrive.

Members (3 total — 2 printed + 1 stock):
  * kicker_mount  — printed bracket: foot sitting on the cross-rail's
                    top face (z = -200) with two M5 vertical bolt-
                    clearance holes into the rail's top T-slot, plus a
                    pillar rising to the pivot zone with a horizontal
                    Ø5.4 pivot-pin bore (axis along ±Y) at
                    (TURRET_DUMP_KICKER_PIVOT_X, 0, _PIVOT_Z). Mount
                    body lives at y ∈ [-15, +15] (centered on y = 0 so
                    the bolt holes drill through clean foot wall).
  * kicker_lever  — printed L-lever: hub-disc at pivot + hub arm + hub-
                    end follower block + lifter arm + lifter pad. Lever
                    body lives at y ∈ [+16, +28] (1 mm air gap from
                    mount + 12 mm thick), cantilevered on the +Y side
                    of the mount with the pivot pin spanning both.
  * kicker_pin    — stock M5 round pin (Ø5 × 50 mm), axis along ±Y,
                    captures the lever between the mount and a
                    snap-ring / E-clip (not modelled).

For-standalone-only stubs (1 each, duplicated rather than sibling-
imported per `cad/CLAUDE.md` principle 2):
  * cross_rail_stub — 60 mm slice of the slice (a) cross-rail under the
                      mount, for the bolt-line + foot-tangent check.
  * cam_stub        — slice (f1) cam disc, for the lever-vs-cam clearance
                      check at the rest position.

Coordinate frame = turret-local frame (matches `../params.py`):
    origin = hub axis at the cross-rail's top face Z plane
    +X     = operator's right            (parallel to world +X)
    +Y     = away from operator          (parallel to world +Y)
    +Z     = up                          (parallel to world +Z)

Print orientation (both printed parts): bottom face on the build plate,
+Z up (build123d default). The mount has only vertical bolt holes
(clean through-hole) and a horizontal pivot bore (Ø5.4 ceiling bridges
fine in FDM at this diameter — same allowance as
`platform_hinge_bearing` and the slice (c) spider). The lever has a
horizontal pivot bore (same allowance) and no other overhangs.
"""

import math
import sys
from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from turret.params import (  # noqa: E402
    TURRET_DUMP_CAM_BASE_R,
    TURRET_DUMP_CAM_BOTTOM_Z,
    TURRET_DUMP_CAM_CENTER_Z,
    TURRET_DUMP_CAM_HEIGHT,
    TURRET_DUMP_CAM_PEAK_R,
    TURRET_DUMP_CAM_TOP_Z,
    TURRET_DUMP_KICKER_LIFTER_TOP_Z,
    TURRET_DUMP_KICKER_LIFTER_X,
    TURRET_DUMP_KICKER_PIVOT_X,
    TURRET_DUMP_KICKER_PIVOT_Y,
    TURRET_DUMP_KICKER_PIVOT_Z,
    TURRET_FRAME_CROSS_Y,
    TURRET_FRAME_FLOOR_Z,
    TURRET_HINGE_PIN_D,
    WALL_MIN,
)


# --- Cross-rail interface (private mirrors of slice (a)) ---------------
#
# Cross-rail along ±X at y = TURRET_FRAME_CROSS_Y = 0, top face at
# z = -200. Mount foot bolts into the rail's top T-slot at y = 0.

MEMBER = 20.0                      # 2020 extrusion
RAIL_TOP_Z = TURRET_FRAME_FLOOR_Z + MEMBER / 2                     # = -200


# --- Pivot pin ---------------------------------------------------------

PIVOT_PIN_D = TURRET_HINGE_PIN_D                                   # = 5
PIVOT_BORE_D = PIVOT_PIN_D + 0.4                                    # = 5.4


# --- Mount geometry (printed bracket on cross-rail) --------------------

# Foot dimensions (sits on rail top face). X spans along the rail so
# both M5 bolt holes drop into the rail's top T-slot at y = 0.
MOUNT_FOOT_X = 60.0
MOUNT_FOOT_Y = 30.0    # depth (across rail)
MOUNT_FOOT_Z = 8.0     # vertical foot height
_MOUNT_FOOT_BOTTOM_Z = RAIL_TOP_Z                                  # = -200
_MOUNT_FOOT_TOP_Z = _MOUNT_FOOT_BOTTOM_Z + MOUNT_FOOT_Z            # = -192
_MOUNT_FOOT_CENTER_Z = (_MOUNT_FOOT_BOTTOM_Z + _MOUNT_FOOT_TOP_Z) / 2

# M5 bolt clearance, two holes along the rail T-slot (at y = 0).
MOUNT_BOLT_HOLE_D = 5.5
MOUNT_BOLT_SPACING_X = 35.0                                        # along rail
_MOUNT_BOLT_HALF_X = MOUNT_BOLT_SPACING_X / 2

# Pillar (rises from foot to enclose pivot bore). Centered on the foot
# in X / Y; spans foot top up to a wall-min cap above the pivot bore.
MOUNT_PILLAR_X = 25.0
MOUNT_PILLAR_Y = MOUNT_FOOT_Y                                      # = 30 (depth)
_MOUNT_PILLAR_BOTTOM_Z = _MOUNT_FOOT_TOP_Z                         # = -192
_PILLAR_CAP_ABOVE_BORE = WALL_MIN + 2.0                            # = 3.5
_MOUNT_PILLAR_TOP_Z = (
    TURRET_DUMP_KICKER_PIVOT_Z + PIVOT_BORE_D / 2 + _PILLAR_CAP_ABOVE_BORE
)                                                                  # ≈ -123.8
MOUNT_PILLAR_Z = _MOUNT_PILLAR_TOP_Z - _MOUNT_PILLAR_BOTTOM_Z      # ≈ 68.2 (vertical)
_MOUNT_PILLAR_CENTER_Z = (_MOUNT_PILLAR_BOTTOM_Z + _MOUNT_PILLAR_TOP_Z) / 2


# --- Lever geometry (printed L-lever) ----------------------------------

LEVER_T = 12.0                                                     # Y-thickness (depth)
_LEVER_AIR_GAP_Y = 1.0                                             # mount→lever
LEVER_Y_INSIDE = MOUNT_FOOT_Y / 2 + _LEVER_AIR_GAP_Y               # = +16
LEVER_Y_OUTSIDE = LEVER_Y_INSIDE + LEVER_T                         # = +28
LEVER_Y_CENTER = (LEVER_Y_INSIDE + LEVER_Y_OUTSIDE) / 2            # = +22

# Hub disc (at pivot bore). OD chosen to leave WALL_MIN around bore.
LEVER_HUB_DISC_OD = 24.0
_LEVER_HUB_DISC_R = LEVER_HUB_DISC_OD / 2

# Arm beam width (perpendicular to arm direction in X-Z plane).
LEVER_ARM_W = 14.0

# Hub-end block (where the cam follower roller will attach in slice
# f5). The cam material at heading 180° at REST sits inside radius
# BASE_R = 25 from the cam axis (cam-local cross-section is the same
# as world cross-section at REST). The lever body lives at y = +22
# (LEVER_Y_INSIDE = +16), off the cam axis Y plane, so the hub-end's
# inboard-inner corner at (LEVER_HUB_END_X + LEVER_HUB_END_X_LEN/2,
# LEVER_Y_INSIDE) sees the cam profile at a heading in the cam's
# valley region (≈ 145°, delta ≈ 35° from nearest lobe). Hub-end X is
# pushed outboard so this corner clears BASE_R; the follower roller
# itself bridges from the lever face down to the cam OD plane (slice
# f5).
LEVER_HUB_END_X = -32.0
LEVER_HUB_END_Z = TURRET_DUMP_CAM_CENTER_Z                         # = -75
LEVER_HUB_END_X_LEN = 18.0
LEVER_HUB_END_Z_LEN = 14.0

# Lifter pad block (top face = LIFTER_TOP_Z at rest).
LIFTER_PAD_X_LEN = 30.0
LIFTER_PAD_Z_LEN = 8.0
_LIFTER_PAD_CENTER_Z = TURRET_DUMP_KICKER_LIFTER_TOP_Z - LIFTER_PAD_Z_LEN / 2


# --- Pivot pin dimensions ----------------------------------------------

# Spans y ∈ [-16, +29] = mount inner face minus 1 mm overhang to lever
# outer face plus 1 mm overhang. 45 mm of stock M5 round.
_PIN_OVERHANG = 1.0
_PIN_Y_INSIDE = -(MOUNT_FOOT_Y / 2) - _PIN_OVERHANG                # = -16
_PIN_Y_OUTSIDE = LEVER_Y_OUTSIDE + _PIN_OVERHANG                   # = +29
PIN_LENGTH = _PIN_Y_OUTSIDE - _PIN_Y_INSIDE                        # = 45
_PIN_Y_CENTER = (_PIN_Y_INSIDE + _PIN_Y_OUTSIDE) / 2               # = +6.5


# --- Sanity asserts ----------------------------------------------------

# Mount: bolt-hole edge to pivot-bore edge wall (X direction). Bolt
# holes (axis ±Z) at x = ±_MOUNT_BOLT_HALF_X = ±17.5 from pivot X.
# Pivot bore (axis ±Y) at pivot X. Their closest approach in X-line is
# the X-distance between bolt-hole edge (X = ±_MOUNT_BOLT_HALF_X minus
# bolt-hole radius toward pivot) and pivot-bore edge (X = ±PIVOT_BORE_D/2).
_bolt_to_bore_x = (
    _MOUNT_BOLT_HALF_X - MOUNT_BOLT_HOLE_D / 2 - PIVOT_BORE_D / 2
)                                                                  # ≈ 12.05
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
# bottom Z). Platform bracket bottom Z = -PLATFORM_ACRYLIC_T - BOND_GAP
# - BRACKET_BLOCK_Z = -4 - 0.3 - 23 = -27.3.
_PLATFORM_BRACKET_BOTTOM_Z = -27.3
_lifter_to_platform = _PLATFORM_BRACKET_BOTTOM_Z - TURRET_DUMP_KICKER_LIFTER_TOP_Z
assert _lifter_to_platform > 0, (
    f"lifter top Z ({TURRET_DUMP_KICKER_LIFTER_TOP_Z}) ≥ platform "
    f"bracket bottom ({_PLATFORM_BRACKET_BOTTOM_Z}); rest position "
    f"would crash the platform"
)

# Lever hub-end clears the cam OD at REST (cam at valley, BASE_R radius
# at heading 180°). Hub-end block at (LEVER_HUB_END_X, LEVER_HUB_END_Z,
# LEVER_Y_CENTER); its closest-to-cam-axis point is the inboard X face
# at X = LEVER_HUB_END_X + LEVER_HUB_END_X_LEN/2 = -25 + 9 = -16. At Y
# = LEVER_Y_INSIDE = +16, the lever's inboard corner is at radial
# sqrt(16² + 16²) = 22.6 from cam axis — INSIDE BASE_R = 25 along the
# unit vector to that corner. Compute the actual cam OD radius at that
# corner's heading and check.
_corner_x = LEVER_HUB_END_X + LEVER_HUB_END_X_LEN / 2               # = -16
_corner_y = LEVER_Y_INSIDE                                          # = +16
_corner_r = math.hypot(_corner_x, _corner_y)                        # ≈ 22.6
_corner_heading_deg = math.degrees(math.atan2(_corner_y, _corner_x))
# Cam profile (4-fold): reduce heading to delta in [0, 45].
_delta = (_corner_heading_deg + 360.0) % 90.0
if _delta > 45.0:
    _delta = 90.0 - _delta
_HALF_PEAK = 10.0                                                  # from slice f1
_RAMP = 10.0
if _delta <= _HALF_PEAK:
    _cam_r = TURRET_DUMP_CAM_PEAK_R
elif _delta <= _HALF_PEAK + _RAMP:
    _cam_r = TURRET_DUMP_CAM_PEAK_R - (_delta - _HALF_PEAK) / _RAMP * (
        TURRET_DUMP_CAM_PEAK_R - TURRET_DUMP_CAM_BASE_R
    )
else:
    _cam_r = TURRET_DUMP_CAM_BASE_R
assert _corner_r > _cam_r, (
    f"lever hub-end inboard corner radial {_corner_r:.2f} mm ≤ cam OD "
    f"radius {_cam_r:.2f} mm at heading {_corner_heading_deg:.1f}°; "
    f"lever crashes the cam at REST. Move LEVER_HUB_END_X further "
    f"outboard or shrink LEVER_HUB_END_X_LEN."
)


# --- Part functions ----------------------------------------------------

def kicker_mount():
    """Printed bracket: foot + pillar with horizontal pivot bore.
    Centered on (TURRET_DUMP_KICKER_PIVOT_X, 0, *) in turret-local;
    bottom face on cross-rail top (z = RAIL_TOP_Z)."""
    foot = (
        Pos(TURRET_DUMP_KICKER_PIVOT_X, 0, _MOUNT_FOOT_CENTER_Z)
        * Box(MOUNT_FOOT_X, MOUNT_FOOT_Y, MOUNT_FOOT_Z)
    )
    # Two M5 vertical bolt clearance holes at y = 0, x = pivot ± half.
    # Cylinder default axis +Z = vertical = correct under Z-up; no
    # rotation needed.
    for sx in (-1, 1):
        foot -= (
            Pos(
                TURRET_DUMP_KICKER_PIVOT_X + sx * _MOUNT_BOLT_HALF_X,
                0,
                _MOUNT_FOOT_CENTER_Z,
            )
            * Cylinder(MOUNT_BOLT_HOLE_D / 2, MOUNT_FOOT_Z * 3)
        )

    pillar = (
        Pos(TURRET_DUMP_KICKER_PIVOT_X, 0, _MOUNT_PILLAR_CENTER_Z)
        * Box(MOUNT_PILLAR_X, MOUNT_PILLAR_Y, MOUNT_PILLAR_Z)
    )

    body = foot + pillar

    # Pivot bore — axis along ±Y (depth direction), through the pillar.
    # Default Cylinder axis is +Z; rotate to ±Y.
    body -= (
        Pos(
            TURRET_DUMP_KICKER_PIVOT_X,
            0,
            TURRET_DUMP_KICKER_PIVOT_Z,
        )
        * Rot(90, 0, 0)
        * Cylinder(PIVOT_BORE_D / 2, MOUNT_PILLAR_Y * 3)
    )

    return body


def kicker_lever():
    """Printed L-lever: hub disc + hub arm + hub-end block + lifter
    arm + lifter pad. Body lives at y ∈ [LEVER_Y_INSIDE,
    LEVER_Y_OUTSIDE], rotated in the X-Z plane around the pivot bore.
    REST orientation: hub-end at follower-contact position (cam BASE_R
    radius at heading 180°), lifter pad TOP face at LIFTER_TOP_Z."""
    # Hub disc at pivot. Axis along ±Y (pivot direction). Default
    # Cylinder axis +Z; rotate to ±Y.
    hub_disc = (
        Pos(
            TURRET_DUMP_KICKER_PIVOT_X,
            LEVER_Y_CENTER,
            TURRET_DUMP_KICKER_PIVOT_Z,
        )
        * Rot(90, 0, 0)
        * Cylinder(_LEVER_HUB_DISC_R, LEVER_T)
    )

    # Hub arm: rectangular beam from pivot to hub-end center.
    hub_arm = _arm_beam(
        x0=TURRET_DUMP_KICKER_PIVOT_X,
        z0=TURRET_DUMP_KICKER_PIVOT_Z,
        x1=LEVER_HUB_END_X,
        z1=LEVER_HUB_END_Z,
    )

    hub_end_block = (
        Pos(LEVER_HUB_END_X, LEVER_Y_CENTER, LEVER_HUB_END_Z)
        * Box(LEVER_HUB_END_X_LEN, LEVER_T, LEVER_HUB_END_Z_LEN)
    )

    # Lifter arm: rectangular beam from pivot to lifter pad center.
    lifter_arm = _arm_beam(
        x0=TURRET_DUMP_KICKER_PIVOT_X,
        z0=TURRET_DUMP_KICKER_PIVOT_Z,
        x1=TURRET_DUMP_KICKER_LIFTER_X,
        z1=_LIFTER_PAD_CENTER_Z,
    )

    lifter_pad = (
        Pos(
            TURRET_DUMP_KICKER_LIFTER_X,
            LEVER_Y_CENTER,
            _LIFTER_PAD_CENTER_Z,
        )
        * Box(LIFTER_PAD_X_LEN, LEVER_T, LIFTER_PAD_Z_LEN)
    )

    body = hub_disc + hub_arm + hub_end_block + lifter_arm + lifter_pad

    # Pivot bore through hub disc — axis along ±Y.
    body -= (
        Pos(
            TURRET_DUMP_KICKER_PIVOT_X,
            LEVER_Y_CENTER,
            TURRET_DUMP_KICKER_PIVOT_Z,
        )
        * Rot(90, 0, 0)
        * Cylinder(PIVOT_BORE_D / 2, LEVER_T * 3)
    )

    return body


def _arm_beam(x0: float, z0: float, x1: float, z1: float):
    """Rectangular arm beam in the X-Z plane, Y-thickness LEVER_T,
    centered at the midpoint of (x0, z0)→(x1, z1) and rotated to align
    its long axis along that direction. Width LEVER_ARM_W
    perpendicular to the arm in the X-Z plane.

    Box default long axis +X. Ry(α) maps +X = (1,0,0) onto
    (cos α, 0, -sin α); to land on (dx/L, 0, dz/L) we need
    cos α = dx/L and sin α = -dz/L, giving α = atan2(-dz, dx).
    """
    dx = x1 - x0
    dz = z1 - z0
    arm_len = math.hypot(dx, dz)
    arm_angle_deg = math.degrees(math.atan2(-dz, dx))
    return (
        Pos((x0 + x1) / 2, LEVER_Y_CENTER, (z0 + z1) / 2)
        * Rot(0, arm_angle_deg, 0)
        * Box(arm_len, LEVER_T, LEVER_ARM_W)
    )


def kicker_pin(length: float = PIN_LENGTH, d: float = PIVOT_PIN_D):
    """Stock M5 round pivot pin. Default Cylinder axis +Z; caller
    rotates with Rot(90, 0, 0) to align with the pivot bore (axis ±Y)."""
    return Cylinder(d / 2, length)


# --- Stubs for the standalone (duplicated, not imported) ---------------

def _cross_rail_stub(length: float = MOUNT_FOOT_X + 20.0):
    """Slice (a) cross-rail stub under the mount. Length spans the
    foot footprint plus a margin so the bolt-line + foot tangent
    contact is exercised in the standalone."""
    return (
        Pos(TURRET_DUMP_KICKER_PIVOT_X, TURRET_FRAME_CROSS_Y, TURRET_FRAME_FLOOR_Z)
        * Box(length, MEMBER, MEMBER)
    )


def _cam_stub():
    """Slice (f1) cam disc geometry — used here only for the lever-vs-
    cam clearance check at REST. 4-lobed profile rebuilt locally per
    cad/CLAUDE.md principle 2 (no sibling imports). The cam is pinned
    to the dwell orientation: a lobe peak sits at heading 180° (the
    follower face)."""
    from build123d import Polygon, extrude

    # Profile sampling — match slice (f1)'s _radius_at.
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
    # Disc axis is now +Z (build123d default); no lift rotation needed.
    return Pos(0, 0, TURRET_DUMP_CAM_CENTER_Z) * body


# --- Assembly builder --------------------------------------------------

KICKER_PART_NAMES = ("kicker_mount", "kicker_lever", "kicker_pin")


def add_turret_dump_kicker(a: Assembly, base=Pos(0, 0, 0)) -> Assembly:
    """Place the kicker mount + lever + pin in turret-local. `base`
    (default: identity) is pre-multiplied to lift the assembly into a
    parent frame (e.g. world)."""
    a = a.add("kicker_mount", kicker_mount(), location=base)
    a = a.add("kicker_lever", kicker_lever(), location=base)
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
    )
    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_turret_dump_kicker(Assembly())

# Stubs for standalone clearance checks.
assembly = assembly.add("cross_rail_stub", _cross_rail_stub())
assembly = assembly.add("cam_stub", _cam_stub())

# All pairwise no-interference between kicker parts and stubs. The pin
# tangent-fits the bores (zero-overlap in the model); foot tangent-sits
# on the rail top (also zero-overlap).
_ALL = list(KICKER_PART_NAMES) + ["cross_rail_stub", "cam_stub"]
for _i, _a in enumerate(_ALL):
    for _b in _ALL[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: bottom face on the build plate, +Z up
    # (build123d default). The mount has only a vertical bolt-hole
    # pair (clean through) and a horizontal Ø5.4 pivot bore (90°
    # overhang at the bore ceiling bridges fine in FDM at this
    # diameter — same allowance as platform_hinge_bearing). The lever
    # has the same horizontal Ø5.4 pivot bore plus rectangular arms
    # with no other overhangs.
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
