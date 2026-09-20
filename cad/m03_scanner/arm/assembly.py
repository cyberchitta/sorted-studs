"""
M03 scanner — rev-3 turret arm spider (slice c of the 4-station turret).

Standalone printed "spider" body that clamps to the slice (b) shaft top
and carries the platform-side hinge for all 4 stations. _notes/state/m03-scanner.md
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
  * pin_image, pin_load, pin_idle, pin_dump — 4 stock M5 round
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

**Level-holding (per hinge): stock torsion spring around the M5
hinge pin.** The earlier per-arm "level stop" shoulder on each
bushing block — a shoulder under the platform's outboard half that
caught gravity-induced outer-down tilt at level — was removed
because it physically blocks the dump direction: at DUMP the
kicker tilts the platform OUTER-DOWN (brick slides outboard into
the diverter), so any rigid stop in that direction would prevent
the dump. Each hinge instead carries a preloaded torsion spring
(stock part, deferred to mechanism slice) sized to balance the
~0.18 N·m gravity torque from the platform's outboard CoM at
level. Failsafe: spring fails relaxed → platform tilts outer-down
under gravity at all stations → dumps the brick (acceptable given
spring is a high-cycle stock part and per-arm independent); spring
fails seized → platform stuck at level → kicker can't dump (one
platform's bricks queue at LOAD, recoverable). Shoulder geometry
is gone from the spider; the bushing block is now plain.

Print orientation: bottom face on the build plate, +Z up (build123d
default). The Ø8.4 shaft bore prints as a clean vertical hole; the 4
horizontal pin bores and 1 horizontal set-screw bore print as
horizontal cylinders whose lower-half ceilings read as 90° overhangs
in the ray sampler but bridge fine in FDM at Ø4–5.4 (same allowance
as platform_hinge_bearing).
"""

import math

from build123d import Box, Cylinder, Pos, Rot

from cad_khana.mechanism.assembly import Assembly


from ..params import (
    TURRET_HINGE_PIN_D,
    TURRET_HINGE_PIN_RADIAL,
    TURRET_HINGE_PIN_Z,
    TURRET_HUB_X,
    TURRET_HUB_Y,
    TURRET_SHAFT_D,
    TURRET_SHAFT_TOP_Z,
    WALL_MIN,
)
from ..materials import material_of


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

# The beam runs INSIDE the hub disc, not up to its OD. A beam whose
# inner face sits at r = HUB_OD/2 is tangent to the disc — one line of
# contact, no shared material, so hub and arms print as five loose
# pieces (caught 2026-09-20 by `assert_solid_count`). The embed is
# chosen so both inner corners fall inside the disc
# (sqrt(13**2 + 7**2) = 14.76 < 15). The beam's inner face is a chord,
# so each arm root also gains the small web between that chord and the
# disc's arc: +197 mm3 per arm, 0.7% of the body, inside the existing
# bounding box.
ARM_EMBED = 2.0                            # radial bite into the hub disc
_ARM_INNER_R = HUB_OD / 2 - ARM_EMBED      # joins INSIDE the hub disc OD


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


# --- Pin dimensions ----------------------------------------------------

# Each pin protrudes 5 mm tangentially past each bushing-block face.
_PIN_PROTRUSION = 5.0
PIN_LENGTH = BUSHING_BLOCK_W + 2 * _PIN_PROTRUSION   # = 30


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

    return beam + block - pin_bore


def spider():
    """Full spider body: central hub disc + 4 radial arms + 4 tip
    bushing blocks. Spider-local origin at hub axis intersected with
    the pin-axis horizontal plane. Disc + arms span z ∈
    [-HUB_HEIGHT/2, +HUB_HEIGHT/2] around origin."""
    # Wall and proportion sanity for this body. The bores and the
    # set-screw insert are subtracted negative space and the arm-beam
    # bounds are loose parameter checks, so none is expressible as a
    # part-pair claim — they stay bare and fire when the spider is cut.
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
        "pin_dump",
        (TURRET_HUB_X,
         TURRET_HUB_Y - TURRET_HINGE_PIN_RADIAL,
         TURRET_HINGE_PIN_Z),
        Rot(0, 90, 0),                # +Z → +X
    ),
    (
        "pin_idle",
        (TURRET_HUB_X - TURRET_HINGE_PIN_RADIAL,
         TURRET_HUB_Y,
         TURRET_HINGE_PIN_Z),
        Rot(90, 0, 0),                # +Z → ±Y
    ),
)

ARM_PART_NAMES = ("spider",) + tuple(name for name, _, _ in _PIN_PLACEMENTS)


# --- Assembly builder --------------------------------------------------

def build_turret_arm() -> Assembly:
    """Spider + 4 hinge pins sub-assembly, in turret-local
    coordinates; the caller places it."""
    # Spider-top Z datum consistency. Both sides are constants, so
    # there is nothing to measure — bare, in the builder that places
    # the spider at the datum they describe.
    _spider_top_z_derived = TURRET_HINGE_PIN_Z + HUB_HEIGHT / 2
    assert abs(TURRET_ARM_SPIDER_TOP_Z - _spider_top_z_derived) < 1e-6, (
        f"TURRET_ARM_SPIDER_TOP_Z ({TURRET_ARM_SPIDER_TOP_Z}) ≠ derived "
        f"({_spider_top_z_derived}); update TURRET_HINGE_PIN_Z or "
        f"TURRET_ARM_SPIDER_HEIGHT"
    )
    # Spider top face flush with shaft top (design assumes flush mounting).
    assert abs(TURRET_ARM_SPIDER_TOP_Z - TURRET_SHAFT_TOP_Z) < 1e-6, (
        f"spider top ({TURRET_ARM_SPIDER_TOP_Z}) ≠ shaft top "
        f"({TURRET_SHAFT_TOP_Z}); design assumes flush mounting"
    )

    a = Assembly()
    a = a.with_part(
        "spider", spider(),
        location=Pos(TURRET_HUB_X, TURRET_HUB_Y, TURRET_HINGE_PIN_Z),
        material=material_of("spider"),
    )
    for name, local_pos, rot in _PIN_PLACEMENTS:
        a = a.with_part(
            name, pin(), location=Pos(*local_pos) * rot,
            material=material_of(name),
        )
    # Hub disc and the four arms are one printed body. They met along
    # four tangent lines until 2026-09-20 — volume, bbox and every
    # drawn view of that spider looked exactly right.
    return a.assert_solid_count("spider")


# --- Standalone assembly -----------------------------------------------

# All pairwise no-interference. Each pin sits in its bushing-block
# bore (Ø5 pin in Ø5.4 bore = 0.2 mm radial gap, no interference);
# the 4 pins are well-separated (≥ 70 mm apart). Spider/spider would
# be self-comparison so it doesn't appear.
assembly = build_turret_arm().assert_no_interference_within(ARM_PART_NAMES)
