"""
M03 scanner — rev-4 turret dump cam (slice f1 of the 4-station turret).

Standalone printed cam disc keyed to the slice (b) shaft. Drives the
bench-mounted DUMP kicker (slice f2, deferred) via a follower lever.
4 radial lobes 90° apart aligned with the spider arm headings —
LOAD / IMAGE / DUMP / RETURN — so that at every indexed dwell, one
lobe sits aligned with the bench-fixed kicker follower at heading
180° (DUMP) and pushes it outward; the lever transmits that radial
rise to the lifter pad at the DUMP station, tilting the parked
platform outer-down. During indexing the cam rotates with the shaft
and the follower drops into the valley between lobes, retracting the
kicker.

Members (1 printed body + 1 stock shaft):
  * dump_cam       — printed disc with central Ø8.4 shaft bore, an M3
                     set-screw heat-set insert at heading +45°
                     (between lobes — same idiom as slice (c)
                     spider), and a 4-lobed radial profile (4-fold
                     rotational symmetry).
  * hub_shaft_stub — stock Ø8 shaft stub. Same Ø and Z span as the
                     slice (b) shaft; duplicated here rather than
                     imported from the sibling because cad/CLAUDE.md
                     principle 2 forbids sibling-to-sibling imports.
                     Only its overlap with the cam disc is asserted;
                     other turret-internal interferences land in the
                     top-level integrate slice.

Coordinate frame = turret-local frame (matches `../params.py`):
    origin = hub axis at the cross-rail's top face Z plane
    +X     = operator's right            (parallel to world +X)
    +Y     = away from operator          (parallel to world +Y)
    +Z     = up                          (parallel to world +Z)

Lobe profile (around Z axis, 4-fold symmetric):
    lobe headings:    0° (+X / LOAD), 90° (+Y / IMAGE),
                      180° (-X / DUMP), 270° (-Y / RETURN)
    lobe peak:        20° wide, at PEAK_R = 30 mm
    lobe ramp:        10° each side, linear PEAK_R → BASE_R
    valley between:   50° wide, at BASE_R = 25 mm
Sum: 4 × (20° + 2 × 10°) + 4 × 50° = 360° ✓.

Print orientation: bottom face on the build plate, +Z up (build123d
default). The Ø8.4 shaft bore prints as a clean vertical hole; the
horizontal set-screw bore reads as a 90° overhang in the ray sampler
but bridges cleanly in FDM at Ø4 (same allowance as
`platform_hinge_bearing` and the slice (c) spider).
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
    TURRET_DUMP_CAM_PEAK_R,
    TURRET_DUMP_CAM_TOP_Z,
    TURRET_HUB_X,
    TURRET_HUB_Y,
    TURRET_SHAFT_D,
    TURRET_SHAFT_TOP_Z,
    WALL_MIN,
)
from design_systems import color_of  # noqa: E402
from materials import material_of  # noqa: E402


# --- Cam profile geometry (private to this sub-assembly) ---------------

# Lobe angular split (degrees). Each lobe = LOBE_PEAK_DEG flat at
# PEAK_R + LOBE_RAMP_DEG linear ramp on each side. With 4 lobes 90°
# apart, "lobe full angle" must be < 90° or adjacent lobes collide.
LOBE_PEAK_DEG = 20.0
LOBE_RAMP_DEG = 10.0
LOBE_FULL_DEG = LOBE_PEAK_DEG + 2 * LOBE_RAMP_DEG     # = 40

# 4 lobes at headings 0 / 90 / 180 / 270 (LOAD / IMAGE / DUMP / RETURN).
LOBE_HEADINGS_DEG = (0.0, 90.0, 180.0, 270.0)

# Profile sample step. 1° → 360 polygon vertices, plenty of resolution
# for the cam-vs-shaft interference check and FDM tessellation.
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

# Lobe budget: each lobe ≤ 90° spacing.
assert LOBE_FULL_DEG < 90.0, (
    f"single lobe full angle {LOBE_FULL_DEG}° ≥ 90° spacing; adjacent "
    f"lobes will collide. Shrink LOBE_PEAK_DEG or LOBE_RAMP_DEG"
)

# Lobe rise must be visible at the FDM tessellation.
_lobe_rise = TURRET_DUMP_CAM_PEAK_R - TURRET_DUMP_CAM_BASE_R
assert _lobe_rise >= WALL_MIN, (
    f"lobe rise {_lobe_rise:.2f} mm < WALL_MIN ({WALL_MIN}); raise "
    f"TURRET_DUMP_CAM_PEAK_R or shrink TURRET_DUMP_CAM_BASE_R"
)

# Set-screw insert + remaining wall fit inside the disc at heading
# +45° (which sits in the valley region, so the local disc radius is
# BASE_R, not PEAK_R).
_ss_wall = (TURRET_DUMP_CAM_BASE_R - CAM_BORE_D / 2) - CAM_SET_SCREW_INSERT_D
assert _ss_wall >= WALL_MIN, (
    f"set-screw wall {_ss_wall:.2f} mm < WALL_MIN ({WALL_MIN}); the "
    f"valley radius BASE_R ({TURRET_DUMP_CAM_BASE_R}) leaves only "
    f"{TURRET_DUMP_CAM_BASE_R - CAM_BORE_D / 2:.2f} mm of disc wall, "
    f"of which the Ø{CAM_SET_SCREW_INSERT_D} insert consumes "
    f"{CAM_SET_SCREW_INSERT_D} mm. Raise BASE_R or shrink the insert."
)

# Set-screw heading must lie in a valley (away from any lobe peak).
# Nearest lobe heading is 0° or 90°; +45° is exactly between them.
_ss_to_nearest_lobe_deg = min(
    abs(CAM_SET_SCREW_HEADING_DEG - h) for h in LOBE_HEADINGS_DEG
)
assert _ss_to_nearest_lobe_deg > LOBE_FULL_DEG / 2, (
    f"set-screw heading +{CAM_SET_SCREW_HEADING_DEG}° sits within a "
    f"lobe (nearest lobe {_ss_to_nearest_lobe_deg}° away, lobe half-"
    f"width {LOBE_FULL_DEG/2}°); pick a heading in a valley"
)

# Z-span consistency.
_z_thickness_derived = TURRET_DUMP_CAM_TOP_Z - TURRET_DUMP_CAM_BOTTOM_Z
assert abs(_z_thickness_derived - TURRET_DUMP_CAM_HEIGHT) < 1e-6, (
    f"cam disc TOP - BOTTOM ({_z_thickness_derived}) ≠ HEIGHT "
    f"({TURRET_DUMP_CAM_HEIGHT}); update one to match"
)

# Cam disc Z must lie strictly between the slice (b) housing top
# (z = -60 after c7; was -110 at c4 and -80 at c5) and the slice (c)
# spider bottom (z = -35). Clearance to both neighbours is asserted
# in the top-level integrate slice; this is a coarse range check.
_HOUSING_TOP_Z = -60.0
_SPIDER_BOTTOM_Z = -35.0
assert TURRET_DUMP_CAM_BOTTOM_Z > _HOUSING_TOP_Z, (
    f"cam bottom ({TURRET_DUMP_CAM_BOTTOM_Z}) sinks into the housing "
    f"(top at z = {_HOUSING_TOP_Z}); raise TURRET_DUMP_CAM_CENTER_Z"
)
assert TURRET_DUMP_CAM_TOP_Z < _SPIDER_BOTTOM_Z, (
    f"cam top ({TURRET_DUMP_CAM_TOP_Z}) collides with the spider "
    f"(bottom at z = {_SPIDER_BOTTOM_Z}); lower TURRET_DUMP_CAM_CENTER_Z"
)


# --- Profile sampling --------------------------------------------------

def _radius_at(theta_deg: float) -> float:
    """Cam radius at angle `theta_deg` (around Z axis, measured from
    +X heading toward +Y). Returns BASE_R in valleys, PEAK_R inside
    lobe peaks, linear interp in ramps. 4-fold symmetric, so reduce
    the input to its angular distance from the nearest lobe heading
    in [0, 45]°."""
    delta = (theta_deg + 360.0) % 90.0
    if delta > 45.0:
        delta = 90.0 - delta
    half_peak = LOBE_PEAK_DEG / 2
    if delta <= half_peak:
        return TURRET_DUMP_CAM_PEAK_R
    if delta <= half_peak + LOBE_RAMP_DEG:
        ramp_t = (delta - half_peak) / LOBE_RAMP_DEG
        return (
            TURRET_DUMP_CAM_PEAK_R
            - ramp_t * (TURRET_DUMP_CAM_PEAK_R - TURRET_DUMP_CAM_BASE_R)
        )
    return TURRET_DUMP_CAM_BASE_R


# --- Part functions ----------------------------------------------------

def dump_cam():
    """Printed cam disc, axis along ±Z, centred on part-local origin
    in X / Y and on Z = 0. Caller translates so the centerline lands
    at TURRET_DUMP_CAM_CENTER_Z in the assembly frame."""
    n = int(round(360.0 / PROFILE_STEP_DEG))
    pts = []
    for i in range(n):
        theta_deg = i * PROFILE_STEP_DEG
        r = _radius_at(theta_deg)
        # Polygon's local x → cam X; polygon's local y → cam Y.
        x = r * math.cos(math.radians(theta_deg))
        y = r * math.sin(math.radians(theta_deg))
        pts.append((x, y))

    # Polygon in build123d's XY plane (normal +Z). Extrude in +Z to
    # height H, recenter on Z origin. Net result: cam disc with axis
    # along Z (build123d default), spanning Z ∈ [-H/2, +H/2], with cam
    # heading 0° at +X and cam heading 90° at +Y.
    profile = Polygon(*pts, align=None)
    body = extrude(profile, amount=TURRET_DUMP_CAM_HEIGHT)
    body = Pos(0, 0, -TURRET_DUMP_CAM_HEIGHT / 2) * body

    # Central Ø8.4 shaft bore (axis along ±Z = build123d default).
    body -= Cylinder(CAM_BORE_D / 2, TURRET_DUMP_CAM_HEIGHT * 3)

    # M3 set-screw bore at heading +45° (radial through valley region).
    # Cylinder centred at the radial midpoint between bore and outer
    # valley face; length spans bore-to-OD with a small overshoot so
    # the cylinder exits the disc on both ends (inner end pokes into
    # the empty shaft bore). Rot(-90, +45, 0) maps default Cylinder
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
    # the slice (c) spider and platform_hinge_bearing.
    inspect(
        dump_cam(),
        method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
        out="outputs",
        name="dump_cam",
    )
