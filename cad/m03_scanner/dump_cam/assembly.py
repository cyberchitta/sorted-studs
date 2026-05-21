"""
M03 scanner — rev-4 turret dump cam (slice f1 of the 4-station turret).

Standalone printed cam disc keyed to the slice (b) shaft. Drives the
bench-mounted DUMP follower (slice f2 — vertical pin + slide guide +
rocker) via 4 raised lobes on the disc's +Z face. The lobes are 90°
apart aligned with the spider arm headings — LOAD / IMAGE / DUMP /
RETURN — so at every indexed dwell, one lobe sits under the bench-
fixed pin at heading 180° and lifts it. The pin rides directly on
the cam face (no roller); the rocker maps the small pin Z-stroke to
the platform tab's larger lift stroke. During indexing the lobe
rotates clear and the pin drops back to the disc top face.

Members (1 printed body + 1 stock shaft):
  * dump_cam       — printed disc (plain Ø50 cylinder, BASE_R=25) with
                     central Ø8.4 shaft bore, an M3 set-screw heat-set
                     insert at heading +45° (between lobes — same idiom
                     as slice (c) spider), and 4 raised lobes on its
                     +Z face.
  * hub_shaft_stub — stock Ø8 shaft stub. Same Ø and Z span as the
                     slice (b) shaft; duplicated here rather than
                     imported from the sibling because cad/CLAUDE.md
                     principle 2 forbids sibling-to-sibling imports.

Coordinate frame = turret-local frame (matches `../params.py`):
    origin = hub axis at the cross-rail's top face Z plane
    +X     = operator's right            (parallel to world +X)
    +Y     = away from operator          (parallel to world +Y)
    +Z     = up                          (parallel to world +Z)

Face-lobe profile (4-fold symmetric on +Z disc face):
    lobe headings:    0° (+X / LOAD), 90° (+Y / IMAGE),
                      180° (-X / DUMP), 270° (-Y / RETURN)
    lobe radial range R ∈ [LOBE_R_INNER=15, LOBE_R_OUTER=24] mm —
                      8 mm wide annulus centred on the pin contact
                      radius (TURRET_DUMP_CAM_PIN_R=20). Outer edge
                      stops 1 mm inside the disc OD (BASE_R=25)
                      so a flat rim survives at the lobe sides.
    lobe angular extent ±15° about each heading (30° per lobe). cos²
                      Z profile: z(θ_off) = LOBE_RISE · cos²(π·θ_off
                      / (2·15°)) for |θ_off| ≤ 15°, zero outside.
                      Zero slope at peak and at both feet — smooth
                      enough that the pin rides without skipping.
    lobe rise         TURRET_DUMP_CAM_LOBE_RISE = 2 mm above the disc
                      top face. The 2 mm cap (vs the radial cam's
                      5 mm) keeps the cam stack Z-anchored: lobe top
                      at z = -36 with the spider bottom at z = -35,
                      1 mm clearance. The rocker (slice f2) recovers
                      the platform stroke via a ~7.5:1 lever ratio.

Implementation: each lobe is built as a union of thin angular wedges
(annulus segments) extruded from the disc top face up to z(θ) at
each wedge. At PROFILE_STEP_DEG = 1° and LOBE_RISE = 2 mm, the
step between adjacent wedges is ≤ 0.07 mm — below FDM layer height
and visually smooth after print.

Print orientation: bottom face on the build plate, +Z up (build123d
default). The Ø8.4 shaft bore prints as a clean vertical hole; the
horizontal set-screw bore reads as a 90° overhang in the ray sampler
but bridges cleanly in FDM at Ø4 (same allowance as
`platform_hinge_bearing` and the slice (c) spider). The face lobes
print as low-rise ramps with ≤ 22° local slope — well inside the
45° FDM overhang threshold.
"""

import math
import sys
from pathlib import Path

from build123d import Cylinder, Polygon, Pos, Rot, extrude

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    TURRET_DUMP_CAM_BASE_R,
    TURRET_DUMP_CAM_BOTTOM_Z,
    TURRET_DUMP_CAM_CENTER_Z,
    TURRET_DUMP_CAM_HEIGHT,
    TURRET_DUMP_CAM_LOBE_RISE,
    TURRET_DUMP_CAM_LOBE_TOP_Z,
    TURRET_DUMP_CAM_PIN_R,
    TURRET_DUMP_CAM_TOP_Z,
    TURRET_HUB_X,
    TURRET_HUB_Y,
    TURRET_SHAFT_D,
    TURRET_SHAFT_TOP_Z,
    WALL_MIN,
)
from design_systems import color_of  # noqa: E402
from materials import material_of  # noqa: E402


# --- Face-lobe geometry (private to this sub-assembly) -----------------

# 4 lobes at headings 0 / 90 / 180 / 270 (LOAD / IMAGE / DUMP / RETURN).
LOBE_HEADINGS_DEG = (0.0, 90.0, 180.0, 270.0)

# Half-extent (degrees) of each lobe about its heading. Total per-lobe
# angular extent = 2 × LOBE_HALF_DEG.
LOBE_HALF_DEG = 15.0

# Lobe radial range. Outer edge stops 1 mm inside the disc OD so a flat
# rim survives outside the lobes; inner edge stays well clear of the
# Ø8.4 central bore and the +45° set-screw bore (which sits between
# lobes anyway).
LOBE_R_OUTER = TURRET_DUMP_CAM_BASE_R - 1.0    # = 24
LOBE_R_INNER = LOBE_R_OUTER - 8.0              # = 16

# Angular discretisation step. At 1° step and LOBE_RISE = 2 mm, the
# max Z-step between adjacent wedges is ≤ 0.07 mm — below FDM layer
# height. 30 wedges per lobe × 4 lobes = 120 wedges total.
PROFILE_STEP_DEG = 1.0


# --- Hub bore + set-screw ----------------------------------------------

# 0.4 mm diametric clearance over the Ø8 shaft (same fit as slice (c)
# spider's HUB_BORE_D).
CAM_BORE_D = TURRET_SHAFT_D + 0.4                     # = 8.4

# M3 set-screw + heat-set insert idiom from slice (c) spider. Insert
# is heat-pressed into a Ø4 radial bore at heading +45°; the M3
# screw threads into the insert and bears on the shaft.
CAM_SET_SCREW_INSERT_D = 4.0
CAM_SET_SCREW_HEADING_DEG = 45.0


# --- Sanity asserts ----------------------------------------------------

# Lobe annular range fits between the central bore and the disc OD,
# with WALL_MIN inside the bore wall and at least PIN_R within the
# radial band.
assert LOBE_R_INNER > CAM_BORE_D / 2 + WALL_MIN, (
    f"lobe inner radius {LOBE_R_INNER} too close to bore wall "
    f"(bore radius + WALL_MIN = {CAM_BORE_D/2 + WALL_MIN}); raise "
    f"LOBE_R_INNER or shrink CAM_BORE_D"
)
assert LOBE_R_OUTER <= TURRET_DUMP_CAM_BASE_R, (
    f"lobe outer radius {LOBE_R_OUTER} exceeds disc OD "
    f"{TURRET_DUMP_CAM_BASE_R}; shrink LOBE_R_OUTER"
)
assert LOBE_R_INNER <= TURRET_DUMP_CAM_PIN_R <= LOBE_R_OUTER, (
    f"pin contact radius {TURRET_DUMP_CAM_PIN_R} lies outside lobe "
    f"radial band [{LOBE_R_INNER}, {LOBE_R_OUTER}]; the pin would "
    f"miss the lobe at the dwell"
)

# Lobe angular extent leaves a clean valley between adjacent lobes
# (otherwise the cos² ramps overlap at the inter-lobe heading).
assert 2 * LOBE_HALF_DEG < 90.0, (
    f"lobe full angle {2 * LOBE_HALF_DEG}° ≥ 90° spacing; adjacent "
    f"lobes will collide. Shrink LOBE_HALF_DEG"
)

# Lobe rise must clear the FDM tessellation.
assert TURRET_DUMP_CAM_LOBE_RISE >= WALL_MIN, (
    f"lobe rise {TURRET_DUMP_CAM_LOBE_RISE} mm < WALL_MIN "
    f"({WALL_MIN}); raise TURRET_DUMP_CAM_LOBE_RISE"
)

# Lobe-ramp slope ≤ 45° for clean FDM overhang. Max slope of the cos²
# profile is at θ_off = ±W/2, where dz/dθ = LOBE_RISE·π/(2·W) and
# arc-length s = R·dθ. So slope (dz/ds) = LOBE_RISE·π/(2·W·R) using
# W in radians. Check at the smallest radius the lobe spans.
_W_rad = math.radians(LOBE_HALF_DEG)
_max_slope = (TURRET_DUMP_CAM_LOBE_RISE * math.pi) / (2 * _W_rad * LOBE_R_INNER)
_max_slope_deg = math.degrees(math.atan(_max_slope))
assert _max_slope_deg <= 45.0, (
    f"lobe ramp peak slope {_max_slope_deg:.1f}° > 45° FDM overhang "
    f"threshold; lower LOBE_RISE, raise LOBE_HALF_DEG, or raise "
    f"LOBE_R_INNER"
)

# Set-screw insert + remaining wall fit inside the disc at heading
# +45° (between lobes, so the disc radial wall is BASE_R full thickness).
_ss_wall = (TURRET_DUMP_CAM_BASE_R - CAM_BORE_D / 2) - CAM_SET_SCREW_INSERT_D
assert _ss_wall >= WALL_MIN, (
    f"set-screw wall {_ss_wall:.2f} mm < WALL_MIN ({WALL_MIN}); the "
    f"valley radius BASE_R ({TURRET_DUMP_CAM_BASE_R}) leaves only "
    f"{TURRET_DUMP_CAM_BASE_R - CAM_BORE_D / 2:.2f} mm of disc wall, "
    f"of which the Ø{CAM_SET_SCREW_INSERT_D} insert consumes "
    f"{CAM_SET_SCREW_INSERT_D} mm. Raise BASE_R or shrink the insert."
)

# Set-screw heading must lie clear of any lobe footprint (radially it
# already clears at the disc-side radial; angularly it sits in the
# inter-lobe valley by construction).
_ss_to_nearest_lobe_deg = min(
    abs(CAM_SET_SCREW_HEADING_DEG - h) for h in LOBE_HEADINGS_DEG
)
assert _ss_to_nearest_lobe_deg > LOBE_HALF_DEG, (
    f"set-screw heading +{CAM_SET_SCREW_HEADING_DEG}° sits within a "
    f"lobe (nearest lobe {_ss_to_nearest_lobe_deg}° away, lobe half-"
    f"extent {LOBE_HALF_DEG}°); pick a heading in the inter-lobe valley"
)

# Z-span consistency: disc thickness derived = nominal.
_z_thickness_derived = TURRET_DUMP_CAM_TOP_Z - TURRET_DUMP_CAM_BOTTOM_Z
assert abs(_z_thickness_derived - TURRET_DUMP_CAM_HEIGHT) < 1e-6, (
    f"cam disc TOP - BOTTOM ({_z_thickness_derived}) ≠ HEIGHT "
    f"({TURRET_DUMP_CAM_HEIGHT}); update one to match"
)

# Cam Z-band fits between housing top and spider bottom (coarse check;
# tight clearances are asserted at the top-level integrate slice).
_HOUSING_TOP_Z = -60.0
_SPIDER_BOTTOM_Z = -35.0
assert TURRET_DUMP_CAM_BOTTOM_Z > _HOUSING_TOP_Z, (
    f"cam bottom ({TURRET_DUMP_CAM_BOTTOM_Z}) sinks into the housing "
    f"(top at z = {_HOUSING_TOP_Z}); raise TURRET_DUMP_CAM_CENTER_Z"
)
assert TURRET_DUMP_CAM_LOBE_TOP_Z < _SPIDER_BOTTOM_Z, (
    f"lobe top ({TURRET_DUMP_CAM_LOBE_TOP_Z}) collides with the spider "
    f"(bottom at z = {_SPIDER_BOTTOM_Z}); lower TURRET_DUMP_CAM_CENTER_Z "
    f"or shrink TURRET_DUMP_CAM_LOBE_RISE"
)


# --- Face-lobe builder -------------------------------------------------

def _lobe_z(theta_offset_deg: float) -> float:
    """Lobe Z above the disc top face at `theta_offset_deg` from a
    lobe centre. cos² profile: peak at θ_off=0, zero at θ_off=±HALF.
    Zero outside the lobe envelope. Smooth (zero slope) at both peak
    and feet."""
    if abs(theta_offset_deg) >= LOBE_HALF_DEG:
        return 0.0
    s = theta_offset_deg / LOBE_HALF_DEG     # ∈ [-1, +1]
    return TURRET_DUMP_CAM_LOBE_RISE * math.cos(math.pi * s / 2) ** 2


def _build_face_lobe(heading_deg: float, disc_top_z: float):
    """Build one raised face lobe centred at `heading_deg`. The lobe
    is a union of thin annular-segment wedges extruded from the disc
    top face up to the cos² lobe profile.

    Returned solid is in part-local coordinates (cam centred on the
    XY origin, disc top at Z = `disc_top_z`)."""
    n_steps = int(round(2 * LOBE_HALF_DEG / PROFILE_STEP_DEG))
    blob = None
    for k in range(n_steps):
        theta_a_deg = heading_deg - LOBE_HALF_DEG + k * PROFILE_STEP_DEG
        theta_b_deg = theta_a_deg + PROFILE_STEP_DEG
        # Wedge height = profile at the wedge midpoint. Gives a smoother
        # visual approximation than picking either endpoint and keeps
        # the per-wedge over/undershoot symmetric.
        theta_mid_off = (theta_a_deg + theta_b_deg) / 2 - heading_deg
        z_wedge = _lobe_z(theta_mid_off)
        if z_wedge <= 0:
            continue
        ta = math.radians(theta_a_deg)
        tb = math.radians(theta_b_deg)
        pts = [
            (LOBE_R_INNER * math.cos(ta), LOBE_R_INNER * math.sin(ta)),
            (LOBE_R_OUTER * math.cos(ta), LOBE_R_OUTER * math.sin(ta)),
            (LOBE_R_OUTER * math.cos(tb), LOBE_R_OUTER * math.sin(tb)),
            (LOBE_R_INNER * math.cos(tb), LOBE_R_INNER * math.sin(tb)),
        ]
        profile = Polygon(*pts, align=None)
        wedge = Pos(0, 0, disc_top_z) * extrude(profile, amount=z_wedge)
        blob = wedge if blob is None else blob + wedge
    return blob


# --- Part functions ----------------------------------------------------

def dump_cam():
    """Printed cam disc, axis along ±Z, centred on part-local origin
    in X / Y and on Z = 0. Caller translates so the disc centerline
    lands at TURRET_DUMP_CAM_CENTER_Z in the assembly frame."""
    # Plain cylindrical disc (no radial profile). Cylinder() centres
    # along its own axis by default; build123d's default axis is +Z,
    # so the disc spans Z ∈ [-H/2, +H/2] about the part-local origin.
    body = Cylinder(TURRET_DUMP_CAM_BASE_R, TURRET_DUMP_CAM_HEIGHT)

    # 4 raised face lobes on the +Z disc face.
    disc_top_z = TURRET_DUMP_CAM_HEIGHT / 2
    for heading_deg in LOBE_HEADINGS_DEG:
        body += _build_face_lobe(heading_deg, disc_top_z)

    # Central Ø8.4 shaft bore (axis along ±Z = build123d default).
    # Length 3× disc height punches cleanly through both disc and any
    # lobe sitting on top.
    body -= Cylinder(CAM_BORE_D / 2, TURRET_DUMP_CAM_HEIGHT * 3)

    # M3 set-screw bore at heading +45° (radial through the inter-lobe
    # valley). Cylinder centred at the radial midpoint between bore and
    # outer valley face; length spans bore-to-OD with a small overshoot
    # so the cylinder exits the disc on both ends (inner end pokes into
    # the empty shaft bore). Rot(-90, +45, 0) maps the default Cylinder
    # axis +Z onto (cos45, cos45, 0) — heading +45° in the XY plane.
    ss_radial_mid = (TURRET_DUMP_CAM_BASE_R + CAM_BORE_D / 2) / 2
    ss_length = (TURRET_DUMP_CAM_BASE_R - CAM_BORE_D / 2) + 4.0
    ss_c45 = math.sqrt(0.5)
    body -= (
        Pos(ss_radial_mid * ss_c45, ss_radial_mid * ss_c45, 0)
        * Rot(-90, CAM_SET_SCREW_HEADING_DEG, 0)
        * Cylinder(CAM_SET_SCREW_INSERT_D / 2, ss_length)
    )

    return body


def _shaft_stub(length: float):
    """Stock Ø8 shaft stub. Mirrors slice (b) `hub_shaft` for this
    standalone's no-interference check (sibling-to-sibling imports
    are not allowed per cad/CLAUDE.md principle 2). Caller positions
    the stub at the rail top + extends it up to TURRET_SHAFT_TOP_Z."""
    return Pos(0, 0, length / 2) * Cylinder(TURRET_SHAFT_D / 2, length)


# --- Assembly builder --------------------------------------------------

DUMP_CAM_PART_NAME = "dump_cam"


def add_turret_dump_cam(a: Assembly, base=Pos(0, 0, 0)) -> Assembly:
    """Place the dump cam centred on the hub axis at
    TURRET_DUMP_CAM_CENTER_Z. Cam is positioned in turret-local;
    `base` (default: identity) is pre-multiplied to lift it into a
    parent frame (e.g. world)."""
    return a.with_part(
        DUMP_CAM_PART_NAME,
        dump_cam(),
        location=base * Pos(
            TURRET_HUB_X, TURRET_HUB_Y, TURRET_DUMP_CAM_CENTER_Z
        ),
        material=material_of(DUMP_CAM_PART_NAME),
        color=color_of(DUMP_CAM_PART_NAME),
    )


# --- Standalone assembly -----------------------------------------------

# Rail top at z = -170 (= TURRET_FRAME_FLOOR_Z + MEMBER/2 with floor
# at -180). Shaft spans rail-top up to TURRET_SHAFT_TOP_Z; duplicate
# the constant locally rather than import slice (a)'s frame helpers.
_SHAFT_BASE_Z = -170.0
_SHAFT_LEN = TURRET_SHAFT_TOP_Z - _SHAFT_BASE_Z      # = 160

assembly = add_turret_dump_cam(Assembly())
assembly = assembly.with_part(
    "hub_shaft_stub",
    _shaft_stub(length=_SHAFT_LEN),
    location=Pos(TURRET_HUB_X, TURRET_HUB_Y, _SHAFT_BASE_Z),
    material="steel",
)
assembly = assembly.assert_no_interference(
    DUMP_CAM_PART_NAME, "hub_shaft_stub"
)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: bottom face on build plate, +Z up (build123d
    # default). Vertical Ø8.4 bore is a clean through-hole; the
    # horizontal set-screw bore reads as a 90° overhang on its lower
    # half but bridges cleanly in FDM at Ø4 — same allowance taken in
    # the slice (c) spider and platform_hinge_bearing. The face lobes
    # are low-rise ramps (peak slope ≤ 22° at LOBE_R_INNER); well
    # inside the 45° FDM overhang threshold.
    #
    # wall_min relaxed below the project WALL_MIN=1.5: the cos² lobe
    # profile tapers smoothly to zero at the lobe feet (by design — the
    # handoff explicitly forbids sharp corners since the pin rides the
    # cam face without a roller), so the foot wedges are sub-1.5 mm
    # shelves. They are fully bonded to the 12 mm disc beneath, so
    # they print as single-layer features (typical FDM layer 0.2 mm)
    # without the warping/detachment risk that motivates the
    # project-wide 1.5 mm floor for free-standing walls. 0.5 mm is
    # ~one extrusion width; raise back to WALL_MIN if the cam pivots
    # to a discontinuous-foot profile.
    inspect(
        dump_cam(),
        method=FDM(wall_min_mm=0.5, overhang_max_deg=90.0),
        out="outputs",
        name="dump_cam",
    )
