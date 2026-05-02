"""
M03 scanner — rev-3 turret arm spider (slice c of the 4-station turret).

Standalone printed "spider" body that clamps to the slice (b) shaft top
and carries the platform-side hinge for all 4 stations. NEXT_STEPS.md
originally read "4 instances ... single printed body parameterised by
station index" — that turned out to be the wrong topology (4 separate
arms stacked vertically on the shaft would force each platform to a
different Z, breaking the optical-equivalence requirement at IMAGE).
Replaced with a single 4-arm spider in 90° rotational symmetry around
the hub axis, printed once.

Members (5 total — 1 printed body + 4 stock pins):
  * spider — printed body fusing the central hub disc, 4 radial arm
    beams (in +X / -X / +Y / -Y), and 4 tip bushing blocks. Bores:
      - one Ø8.4 vertical shaft bore through the hub disc (Ø8 + 0.4
        diametric clearance over the slice (b) shaft)
      - one Ø4 radial set-screw bore at heading +45° (between the
        +X and +Y arms so it crosses no arm) for an M3 + heat-set
        insert that locks the spider to the shaft
      - four Ø5.4 horizontal pin bores (one per arm tip), each
        aligned tangentially to its arm's radial direction
  * pin_image, pin_load, pin_dump, pin_return — 4 stock M5 round
    pins (Ø5 × 30 mm), one per arm tip. Each pin is captured in its
    bushing block bore and protrudes 5 mm tangentially past each
    bushing-block face to host the slice (d) platform's bonded
    hinge bracket.

Coordinate frame = turret-local frame (matches `../params.py`):
    origin = hub axis at the cross-rail's top face Z plane
    +X     = operator's right            (parallel to world +X)
    +Y     = away from operator          (parallel to world +Y)
    +Z     = up                          (parallel to world +Z)

The spider's local origin sits at the hub axis at the pin-axis Z
plane: turret-local (TURRET_HUB_X, TURRET_HUB_Y, TURRET_HINGE_PIN_Z)
= (0, 0, -22.5). The disc + arms span z ∈ [-12.5, +12.5] locally
(= z ∈ [-35, -10] in world / turret-local Z), with the top face flush
with the slice (b) shaft top at z = -10.

Arm headings (around Z axis, measured from +X toward +Y):
    +X   →  LOAD station   (pin axis along ±Y)
    +Y   →  IMAGE station  (pin axis along ±X)
    -X   →  DUMP station   (pin axis along ±Y)
    -Y   →  RETURN station (pin axis along ±X)

Each arm carries a small printed "level stop" shoulder on the
OUTBOARD half of its bushing-block top face (radial
TURRET_HINGE_PIN_RADIAL → bushing block outer R). Top face is
coplanar with the cartridge acrylic underside at the level pose,
so gravity (CoM at platform centroid sits radially outboard of the
hinge) rolls each platform onto its arm's shoulder and rests
there. The shoulder is OUTBOARD only by design: a shoulder on the
inboard half would catch the kicker direction (outer-up tilt drops
the underside inboard of the hinge) and block the dump motion.

Print orientation: bottom face on the build plate, +Z up (build123d
default). The Ø8.4 shaft bore prints as a clean vertical hole; the 4
horizontal pin bores and 1 horizontal set-screw bore print as
horizontal cylinders whose lower-half ceilings read as 90° overhangs
in the ray sampler but bridge fine in FDM at Ø4–5.4 (same allowance
as platform_hinge_bearing). The 4 level-stop shoulders are simple
extruded boxes — no overhangs.
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
    PLATFORM_ACRYLIC_T,
    TURRET_HINGE_PIN_D,
    TURRET_HINGE_PIN_RADIAL,
    TURRET_HINGE_PIN_Z,
    TURRET_HUB_X,
    TURRET_HUB_Y,
    TURRET_SHAFT_D,
    TURRET_SHAFT_TOP_Z,
    WALL_MIN,
)


# --- Spider Z geometry (private to this sub-assembly) -------------------

# Spider Z span. 25 mm gives ≥ WALL_MIN around the Ø5.4 pin bore
# in the bushing blocks (12.5 - 2.7 = 9.8 mm vertical wall) and
# matches the hub disc height for a uniform Z profile across the
# whole printed body.
TURRET_ARM_SPIDER_HEIGHT = 25.0

# Spider top face Z in world. Flush with the shaft top so the spider
# slides on from above and bottoms out on a circlip / shoulder at
# the bottom of its hub bore (not modelled).
TURRET_ARM_SPIDER_TOP_Z = TURRET_SHAFT_TOP_Z                     # = -10


# --- Hub disc dimensions -----------------------------------------------

# Outer diameter chosen to leave wall around the M3 set-screw bore
# (Ø4 for a heat-set insert) running radially through the disc to
# the central shaft bore.
HUB_OD = 30.0
HUB_BORE_D = TURRET_SHAFT_D + 0.4         # = 8.4 (0.4 mm dia clearance)
HUB_HEIGHT = TURRET_ARM_SPIDER_HEIGHT     # = 25 (uniform Z profile)

# M3 set-screw heat-set insert diameter (real assembly: a brass
# insert is heat-pressed into this bore from the disc OD; the M3
# screw threads into the insert and bears on the shaft).
SET_SCREW_INSERT_D = 4.0


# --- Arm beam dimensions -----------------------------------------------

ARM_BEAM_W = 14.0                          # tangential width

_ARM_INNER_R = HUB_OD / 2                  # joins hub disc OD


# --- Bushing block dimensions ------------------------------------------

# Block tangential width — wider than the arm beam to give pin-bore
# wall thickness in the tangential direction.
BUSHING_BLOCK_W = 20.0

# Block radial depth (centred on TURRET_HINGE_PIN_RADIAL).
BUSHING_BLOCK_RADIAL_LEN = 18.0
_BUSHING_BLOCK_INNER_R = (
    TURRET_HINGE_PIN_RADIAL - BUSHING_BLOCK_RADIAL_LEN / 2
)
_BUSHING_BLOCK_OUTER_R = (
    TURRET_HINGE_PIN_RADIAL + BUSHING_BLOCK_RADIAL_LEN / 2
)

# Pin bore: Ø5.4 (0.4 mm diametric clearance over the M5 stock pin).
PIN_BORE_D = TURRET_HINGE_PIN_D + 0.4     # = 5.4


# --- Level stop shoulder dimensions ------------------------------------
#
# Each arm grows a printed shoulder on the OUTBOARD half of its
# bushing-block top face. The shoulder top is coplanar with the
# cartridge acrylic underside at the level pose (tangent contact —
# `assert_no_interference` treats this as zero overlap). With the
# platform CoM at the centroid sitting radially outboard of the hinge,
# gravity rolls the unconstrained platform outer-down; the shoulder
# catches it at level. Outer-up tilt (the kicker direction at DUMP)
# raises the underside outboard of the hinge AWAY from the shoulder,
# so the kicker stroke is unconstrained.
#
# Tangent extent matches BUSHING_BLOCK_W so the shoulder reuses the
# bushing block's existing AIR_GAP=0.5 to the cartridge brackets on
# either side; no new bracket clearance constraint is introduced.

# Acrylic underside Z in spider-local. Spider-local origin is the hinge
# pin axis Z plane (= world TURRET_HINGE_PIN_Z); acrylic underside in
# world is at -PLATFORM_ACRYLIC_T (top face at z = 0).
_ACRYLIC_UNDERSIDE_Z_LOCAL = -PLATFORM_ACRYLIC_T - TURRET_HINGE_PIN_Z   # = +18.5

# Stop radial: from hinge axis out to the bushing block outer face.
LEVEL_STOP_RADIAL_INNER = TURRET_HINGE_PIN_RADIAL                      # = 95
LEVEL_STOP_RADIAL_OUTER = _BUSHING_BLOCK_OUTER_R                       # = 104
LEVEL_STOP_RADIAL_LEN = LEVEL_STOP_RADIAL_OUTER - LEVEL_STOP_RADIAL_INNER
LEVEL_STOP_RADIAL_CENTER = (
    LEVEL_STOP_RADIAL_INNER + LEVEL_STOP_RADIAL_OUTER
) / 2

# Stop tangent extent — full bushing-block width.
LEVEL_STOP_W = BUSHING_BLOCK_W                                          # = 20

# Stop Z span: spider top face (= +HUB_HEIGHT/2) up to acrylic underside.
LEVEL_STOP_BOTTOM_Z_LOCAL = HUB_HEIGHT / 2                              # = +12.5
LEVEL_STOP_TOP_Z_LOCAL = _ACRYLIC_UNDERSIDE_Z_LOCAL                     # = +18.5
LEVEL_STOP_HEIGHT = LEVEL_STOP_TOP_Z_LOCAL - LEVEL_STOP_BOTTOM_Z_LOCAL
LEVEL_STOP_Z_CENTER = (
    LEVEL_STOP_BOTTOM_Z_LOCAL + LEVEL_STOP_TOP_Z_LOCAL
) / 2


# --- Pin dimensions ----------------------------------------------------

# Each pin protrudes 5 mm tangentially past each bushing-block face.
_PIN_PROTRUSION = 5.0
PIN_LENGTH = BUSHING_BLOCK_W + 2 * _PIN_PROTRUSION   # = 30


# --- Sanity asserts ----------------------------------------------------

# Set-screw insert leaves wall between insert tip and shaft bore.
_insert_to_shaft_wall = (HUB_OD - HUB_BORE_D) / 2 - SET_SCREW_INSERT_D
assert _insert_to_shaft_wall >= 0, (
    f"set-screw insert depth {SET_SCREW_INSERT_D} mm exceeds disc wall "
    f"({(HUB_OD - HUB_BORE_D) / 2:.2f} mm); raise HUB_OD or shrink "
    f"SET_SCREW_INSERT_D"
)

# Pin bore wall to bushing-block tangential (W) faces.
_pin_wall_w = BUSHING_BLOCK_W / 2 - PIN_BORE_D / 2
assert _pin_wall_w >= WALL_MIN, (
    f"pin-bore-to-block-W wall {_pin_wall_w:.2f} mm < WALL_MIN "
    f"({WALL_MIN}); raise BUSHING_BLOCK_W"
)

# Pin bore wall to spider Z (top + bottom) faces.
_pin_wall_z = HUB_HEIGHT / 2 - PIN_BORE_D / 2
assert _pin_wall_z >= WALL_MIN, (
    f"pin-bore-to-spider-Z wall {_pin_wall_z:.2f} mm < WALL_MIN "
    f"({WALL_MIN}); raise TURRET_ARM_SPIDER_HEIGHT"
)

# Pin bore wall to bushing-block radial (inner / outer) faces.
_pin_wall_r = (BUSHING_BLOCK_RADIAL_LEN - PIN_BORE_D) / 2
assert _pin_wall_r >= WALL_MIN, (
    f"pin-bore-to-block-radial wall {_pin_wall_r:.2f} mm < WALL_MIN "
    f"({WALL_MIN}); raise BUSHING_BLOCK_RADIAL_LEN"
)

# Spider top face Z (world) consistency — re-derive and compare.
_SPIDER_TOP_Z_DERIVED = TURRET_HINGE_PIN_Z + HUB_HEIGHT / 2
assert abs(TURRET_ARM_SPIDER_TOP_Z - _SPIDER_TOP_Z_DERIVED) < 1e-6, (
    f"TURRET_ARM_SPIDER_TOP_Z ({TURRET_ARM_SPIDER_TOP_Z}) ≠ derived "
    f"({_SPIDER_TOP_Z_DERIVED}); update TURRET_HINGE_PIN_Z or "
    f"TURRET_ARM_SPIDER_HEIGHT"
)

# Spider top face flush with shaft top (design assumes flush mounting).
assert abs(TURRET_ARM_SPIDER_TOP_Z - TURRET_SHAFT_TOP_Z) < 1e-6, (
    f"spider top ({TURRET_ARM_SPIDER_TOP_Z}) ≠ shaft top "
    f"({TURRET_SHAFT_TOP_Z}); design assumes flush mounting"
)

# Arm beam has positive length (hub disc OD inside bushing-block
# inner face).
assert _ARM_INNER_R < _BUSHING_BLOCK_INNER_R, (
    f"arm inner R ({_ARM_INNER_R}) ≥ block inner R "
    f"({_BUSHING_BLOCK_INNER_R}); arm beam has zero or negative length"
)

# Arm beam angular extent must not crash the +45° set-screw bore.
_arm_half_angle = math.degrees(math.atan2(ARM_BEAM_W / 2, _ARM_INNER_R))
assert _arm_half_angle < 45.0, (
    f"arm half-angle {_arm_half_angle:.1f}° exceeds 45°; +X / +Y arm "
    f"would clip the set-screw bore at heading +45°"
)

# Level stop must sit OUTBOARD of the hinge axis. A stop on the inboard
# half (radial < TURRET_HINGE_PIN_RADIAL) would catch the kicker
# direction (outer-up tilt drops the underside inboard of the hinge),
# blocking the dump motion.
assert LEVEL_STOP_RADIAL_INNER >= TURRET_HINGE_PIN_RADIAL - 1e-9, (
    f"level stop inner R {LEVEL_STOP_RADIAL_INNER} < hinge axis R "
    f"({TURRET_HINGE_PIN_RADIAL}); shoulder would also block the "
    f"kicker direction (outer-up tilt)"
)

# Level stop radial extent stays within the bushing block — printed as
# part of the block, with no overhang past its outer face.
assert LEVEL_STOP_RADIAL_OUTER <= _BUSHING_BLOCK_OUTER_R + 1e-9, (
    f"level stop outer R {LEVEL_STOP_RADIAL_OUTER} > bushing block "
    f"outer R ({_BUSHING_BLOCK_OUTER_R}); shoulder would overhang"
)

# Level stop tangent width ≤ bushing block width — preserves the
# existing AIR_GAP=0.5 between block face and cartridge brackets.
assert LEVEL_STOP_W <= BUSHING_BLOCK_W + 1e-9, (
    f"level stop tangent W {LEVEL_STOP_W} > bushing block W "
    f"({BUSHING_BLOCK_W}); shoulder would crash bracket inboard face"
)

# Level stop top Z matches the acrylic underside in the world frame.
_LEVEL_STOP_TOP_DERIVED = -PLATFORM_ACRYLIC_T - TURRET_HINGE_PIN_Z
assert abs(LEVEL_STOP_TOP_Z_LOCAL - _LEVEL_STOP_TOP_DERIVED) < 1e-6, (
    f"LEVEL_STOP_TOP_Z_LOCAL ({LEVEL_STOP_TOP_Z_LOCAL}) ≠ derived from "
    f"-PLATFORM_ACRYLIC_T - TURRET_HINGE_PIN_Z ({_LEVEL_STOP_TOP_DERIVED})"
)

# Level stop has positive height (i.e. spider top face sits below the
# acrylic underside; if the two coincide the shoulder collapses).
assert LEVEL_STOP_HEIGHT > 0, (
    f"level stop height {LEVEL_STOP_HEIGHT} ≤ 0; spider top "
    f"({LEVEL_STOP_BOTTOM_Z_LOCAL}) ≥ acrylic underside "
    f"({LEVEL_STOP_TOP_Z_LOCAL}) — check TURRET_HINGE_PIN_Z or "
    f"TURRET_ARM_SPIDER_HEIGHT"
)


# --- Part functions ----------------------------------------------------

def _arm_unit():
    """One arm + bushing block in the +X heading orientation, centred
    on spider-local origin at the hub axis. Pin bore axis along ±Y
    (= tangential at +X heading). Caller composes the full spider by
    rotating around Z for headings (0, 90, 180, 270)."""
    arm_x_center = (_ARM_INNER_R + _BUSHING_BLOCK_INNER_R) / 2
    arm_x_len = _BUSHING_BLOCK_INNER_R - _ARM_INNER_R
    beam = Pos(arm_x_center, 0, 0) * Box(arm_x_len, ARM_BEAM_W, HUB_HEIGHT)

    block = (
        Pos(TURRET_HINGE_PIN_RADIAL, 0, 0)
        * Box(BUSHING_BLOCK_RADIAL_LEN, BUSHING_BLOCK_W, HUB_HEIGHT)
    )

    # Pin bore through the bushing block (axis ±Y = tangential at +X
    # heading). Default Cylinder axis is +Z; rotate to lift onto ±Y.
    pin_bore = (
        Pos(TURRET_HINGE_PIN_RADIAL, 0, 0)
        * Rot(90, 0, 0)
        * Cylinder(PIN_BORE_D / 2, BUSHING_BLOCK_W * 3)
    )

    # Level stop shoulder on the bushing block top face, outboard half
    # only. Top face coplanar with cartridge acrylic underside at the
    # level pose (tangent contact); bottom face flush with bushing
    # block top (no air gap below — printed as a single integral
    # extension of the block).
    level_stop = (
        Pos(LEVEL_STOP_RADIAL_CENTER, 0, LEVEL_STOP_Z_CENTER)
        * Box(LEVEL_STOP_RADIAL_LEN, LEVEL_STOP_W, LEVEL_STOP_HEIGHT)
    )

    return beam + block + level_stop - pin_bore


def spider():
    """Full spider body: central hub disc + 4 radial arms + 4 tip
    bushing blocks. Spider-local origin at hub axis intersected with
    the pin-axis horizontal plane. Disc + arms span z ∈
    [-HUB_HEIGHT/2, +HUB_HEIGHT/2] around origin."""
    # Hub disc — Cylinder default axis +Z = vertical = correct under
    # Z-up. No rotation needed.
    body = Cylinder(HUB_OD / 2, HUB_HEIGHT)

    # Subtract the central shaft bore (axis ±Z).
    body -= Cylinder(HUB_BORE_D / 2, HUB_HEIGHT * 3)

    # Subtract the M3 set-screw bore at heading +45° in the XY
    # horizontal plane. Cylinder centred on the radial midpoint
    # between shaft bore and disc OD; length spans bore + small
    # overshoot so it exits the disc cleanly on both ends (only one
    # end actually creates an OD opening — the inner end pokes into
    # the already-empty shaft bore).
    ss_radial_mid = (HUB_OD / 2 + HUB_BORE_D / 2) / 2
    ss_length = (HUB_OD - HUB_BORE_D) / 2 + 4.0
    c45 = math.sqrt(0.5)
    # Rot(-90, 45, 0) maps default Cylinder axis +Z onto (c45, c45, 0)
    # — heading +45° in the XY horizontal plane.
    body -= (
        Pos(ss_radial_mid * c45, ss_radial_mid * c45, 0)
        * Rot(-90, 45, 0)
        * Cylinder(SET_SCREW_INSERT_D / 2, ss_length)
    )

    # Add 4 arm units by rotating one canonical arm around the Z axis.
    for heading_deg in (0, 90, 180, 270):
        body = body + (Rot(0, 0, heading_deg) * _arm_unit())

    return body


def pin(length: float = PIN_LENGTH, d: float = TURRET_HINGE_PIN_D):
    """Stock M5 round hinge pin. Default Cylinder axis +Z; callers
    rotate per arm heading to align with the bushing-block bore."""
    return Cylinder(d / 2, length)


# --- Pin world placements ---------------------------------------------

# Each pin axis lies along the tangential direction at its arm's
# heading. The pin is centred on the bushing block (= centred at
# radial TURRET_HINGE_PIN_RADIAL from the hub axis along its arm's
# radial line).
_PIN_PLACEMENTS = (
    # (name, world position, rotation aligning Cylinder +Z with the
    # bushing-block tangential axis)
    (
        "pin_image",
        (TURRET_HUB_X,
         TURRET_HUB_Y + TURRET_HINGE_PIN_RADIAL,
         TURRET_HINGE_PIN_Z),
        Rot(0, 90, 0),                # +Z → +X (pin axis along ±X)
    ),
    (
        "pin_load",
        (TURRET_HUB_X + TURRET_HINGE_PIN_RADIAL,
         TURRET_HUB_Y,
         TURRET_HINGE_PIN_Z),
        Rot(90, 0, 0),                # +Z → ±Y (tangential at +X heading)
    ),
    (
        "pin_return",
        (TURRET_HUB_X,
         TURRET_HUB_Y - TURRET_HINGE_PIN_RADIAL,
         TURRET_HINGE_PIN_Z),
        Rot(0, 90, 0),                # +Z → +X
    ),
    (
        "pin_dump",
        (TURRET_HUB_X - TURRET_HINGE_PIN_RADIAL,
         TURRET_HUB_Y,
         TURRET_HINGE_PIN_Z),
        Rot(90, 0, 0),                # +Z → ±Y
    ),
)

ARM_PART_NAMES = ("spider",) + tuple(name for name, _, _ in _PIN_PLACEMENTS)


# --- Assembly builder --------------------------------------------------

def add_turret_arm(a: Assembly, base=Pos(0, 0, 0)) -> Assembly:
    """Add the spider + 4 hinge pins. Parts are positioned in
    turret-local coordinates; `base` (default: identity) is
    pre-multiplied to lift them into a parent frame."""
    spider_pose = base * Pos(TURRET_HUB_X, TURRET_HUB_Y, TURRET_HINGE_PIN_Z)
    a = a.add("spider", spider(), location=spider_pose)
    for name, local_pos, rot in _PIN_PLACEMENTS:
        a = a.add(name, pin(), location=base * Pos(*local_pos) * rot)
    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_turret_arm(Assembly())

# All pairwise no-interference. Each pin sits in its bushing-block
# bore (Ø5 pin in Ø5.4 bore = 0.2 mm radial gap, no interference);
# the 4 pins are well-separated (≥ 70 mm apart). Spider/spider would
# be self-comparison so it doesn't appear.
for _i, _a in enumerate(ARM_PART_NAMES):
    for _b in ARM_PART_NAMES[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: bottom face on build plate, +Z up (build123d
    # default). The 4 horizontal pin bores + 1 horizontal set-screw
    # bore each have a lower-half ceiling that reads as a 90° overhang
    # but bridges cleanly in FDM at Ø4–5.4. Threshold raised to 90°
    # (same rationale as platform_hinge_bearing) so cylinder bridging
    # doesn't false-fail.
    inspect(
        spider(),
        method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
        out="outputs",
        name="spider",
    )
