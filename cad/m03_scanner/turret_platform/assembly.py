"""
M03 scanner — rev-4 turret platform cartridge (slice d of the 4-station
turret), shrunk for rev-5 4-cam topology in slice c3 (2026-05-05).

Two-piece "cartridge": a PMMA acrylic insert (110 × 110 × 4 mm) dropped
into a single printed frame body that integrates the perimeter ring,
acrylic pocket, hinge brackets, and kicker extension. Replaces the
prior 5-piece bonded design (acrylic + 2 brackets + ring + tab) — the
single printed frame is more dimensionally precise (hinge-to-extension
geometry fixed by the printer rather than the glue jig) and removes the
PLA-PMMA bond joints. The acrylic just provides the transparent
brick-resting surface.

Members (2 total — 1 PMMA insert + 1 printed frame):
  * acrylic_insert — 110 × 110 × 4 mm PMMA, top face at z = 0, centred
    on origin in X / Y. Sized to the rev-5 imaged region (per
    `sizing.py`). Dropped into the frame's pocket; rests on 4 corner
    support lips at z = -4. Slip-fit; on assembly the user secures
    the four corners with dabs of structural adhesive (not modelled).
  * frame — single printed body integrating:
      - top ring (z ∈ [0, +5], 120 × 120 outer, 110 × 110 inner) —
        5 mm wide brick-containment lip above the acrylic
      - pocket walls (z ∈ [-4, 0], same cross-section) — capture the
        acrylic edges
      - corner lips (z ∈ [-7, -4], 5 × 10 × 10 mm at pocket corners) —
        support the acrylic from below; only at corners (not a
        continuous bottom lip) so the central viewing area through the
        transparent acrylic stays clear from the below-camera vertices.
        Per-camera below-view occlusion analysis is rig-level
        (visibility.py) and not enforced here.
      - hub-side skirt (z ∈ [-27.3, -4] at cart Y ∈ [-60, -55]) —
        5 mm-thick wall extending the perimeter down for bracket
        attachment; full X span minus a 21 mm-wide notch at cart
        X ∈ [-10.5, +10.5] so the slice (c) bushing block passes
        through unobstructed
      - hinge brackets (cart X = ±13, Y ∈ [-68, -52], z ∈ [-27.3, -4])
        — integrated into the skirt at the bracket positions; horizontal
        Ø5.4 pin bore along ±X at cart Y = -60, z = -22.5
      - kicker extension (cart X ∈ [+21, +39]) — L-shape cantilevered
        hub-ward from the skirt at cart X = +30: stem at z ∈ [-15, -4]
        running from cart Y = -55 to -111, plus a drop block at z ∈
        [-43, -15] over cart Y ∈ [-111, -89]. Contact face (drop
        block bottom) at z = -43 sits 2 mm above the slice (f2) lifter
        pad's REST top; cam stroke closes the gap and lifts the
        platform. Lift point at cart-frame (X=+30, Y=-100) is 40 mm
        hub-side of the hinge — clean single-direction outer-DOWN
        torque about the hinge, with no competing bonded brick-side
        push as in the prior straddle-the-hinge tab design.

Coordinate frame = cartridge-local IMAGE pose:
    origin = brick centroid on platform top face
    +X     = along pin axis
    +Y     = radial outward from hub
    +Z     = up

`add_turret_platform(a, station)` translates and rotates this
cartridge to each station in turret-local (option a' rename,
2026-05-05): IMAGE at (0, +R, 0) with no rotation, LOAD at (+R, 0,
0) with Rz(-90), IDLE at (-R, 0, 0) with Rz(+90), DUMP at (0, -R,
0) with Rz(+180). Same shape, four poses. The new DUMP rotation
(Rz(+180)) is the prior RETURN-station rotation; the IDLE rotation
(Rz(+90)) is the prior DUMP-station rotation. `PLATFORM_ACRYLIC_SIZE`
(= 120) in `../params.py` refers to the FRAME outer footprint; the
acrylic insert is 110. The turret arm-radius clearance math
(`TURRET_HINGE_PIN_RADIAL = TURRET_ARM_R - 60`) uses the platform
outer half-width.

Print orientation: bottom face on build plate, +Z up (build123d
default). Horizontal Ø5.4 pin bore bridges fine in FDM at this
diameter (90° overhang threshold, same as the prior platform brackets
and `platform_hinge_bearing`). The extension stem bridges 33 mm of
mid-air at z = -15 between the drop block and the skirt — within
PLA's bridge capability for the 18 mm beam width; supports recommended
for clean underside finish.
"""

import math
import sys
from pathlib import Path

from build123d import Box, Color, Cylinder, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    PLATFORM_ACRYLIC_SIZE,
    PLATFORM_ACRYLIC_T,
    TURRET_ARM_R,
    TURRET_DUMP_KICKER_LIFTER_TOP_Z,
    TURRET_DUMP_KICKER_LIFTER_TURRET_X,
    TURRET_DUMP_KICKER_LIFTER_TURRET_Y,
    TURRET_HINGE_PIN_D,
    TURRET_HINGE_PIN_RADIAL,
    TURRET_HINGE_PIN_Z,
    WALL_MIN,
)
from design_systems import color_of  # noqa: E402
from materials import material_of  # noqa: E402


# --- Cartridge-local IMAGE-pose references -----------------------------

# In cartridge-local, the IMAGE pin axis runs along ±X. Z matches
# turret-local Z (vertical). Y is "radial outward from hub" at IMAGE
# pose, so the hub-side of the platform sits at -Y.
PIN_AXIS_Z_LOCAL = TURRET_HINGE_PIN_Z                          # = -22.5
# Cartridge centroid at the brick centroid (origin); the pin sits at
# the hub-side edge of the frame, RADIAL short of the hub.
# Cartridge-local hub Y = -ARM_R when placed at IMAGE (centroid at
# turret +R), so cartridge-local pin Y = -ARM_R + RADIAL.
PIN_AXIS_Y_LOCAL = TURRET_HINGE_PIN_RADIAL - TURRET_ARM_R      # = -60


# --- Imaged region (rev-5 sizing.py) -----------------------------------

# Side of the square brick-landing region every camera must see fully
# (= BRICK_MAX + 2*SCATTER_HALF in sizing.py at default brick=80,
# scatter=±15). The acrylic insert must cover at least this region.
IMAGED_REGION_SIDE_MM = 110.0


# --- Acrylic insert (PMMA) ---------------------------------------------

# Sized to the imaged region — covers the 110 × 110 area cameras must
# see fully. Same 4 mm thickness so existing PMMA stock applies; user
# cuts the 110 square from larger sheet.
ACRYLIC_INSERT_SIZE = IMAGED_REGION_SIDE_MM
ACRYLIC_INSERT_T = PLATFORM_ACRYLIC_T                          # = 4


# --- Frame outer geometry ----------------------------------------------

FRAME_OUTER_SIZE = PLATFORM_ACRYLIC_SIZE                        # = 120
FRAME_TOP_RING_HEIGHT = 5.0     # brick-containment lip above acrylic
FRAME_WALL_T = (FRAME_OUTER_SIZE - ACRYLIC_INSERT_SIZE) / 2     # = 5

# Frame Z anchors. Acrylic top at z = 0 (matches the cartridge centroid
# Z plane and the prior bonded-pieces design).
ACRYLIC_TOP_Z = 0.0
ACRYLIC_BOTTOM_Z = -ACRYLIC_INSERT_T                            # = -4
TOP_RING_TOP_Z = ACRYLIC_TOP_Z + FRAME_TOP_RING_HEIGHT          # = +5

# --- Corner lips (acrylic support at 4 pocket corners) -----------------

# Corner lips (rather than a continuous bottom lip) preserve the
# transparent central viewing path through the acrylic from the below
# cameras (rev-5 4-cam topology). A continuous bottom lip would project
# an opaque rectangle into every below-camera frame; corner-only lips
# limit any below-view obstruction to the imaged region's corners.
# Detailed below-view occlusion is checked at the rig level
# (visibility.py), not asserted here.
CORNER_LIP_SIZE = 10.0
CORNER_LIP_T = 3.0
CORNER_LIP_TOP_Z = ACRYLIC_BOTTOM_Z                             # = -4
CORNER_LIP_BOTTOM_Z = CORNER_LIP_TOP_Z - CORNER_LIP_T           # = -7


# --- Hinge bracket dimensions (integrated into hub-side skirt) ---------

# Bracket-local axes match cartridge-local. Mirrors the prior bonded-
# bracket footprint so slice (c) bushing-block placement stays valid.
BRACKET_BLOCK_X = 5.0            # spans one pin-tip protrusion zone
BRACKET_BLOCK_Y = 16.0           # spans across the pin axis
BRACKET_BOTTOM_Z_LOCAL = -27.3   # matches prior design; sets lifter clearance
BRACKET_TOP_Z_LOCAL = ACRYLIC_BOTTOM_Z                          # = -4
BRACKET_BLOCK_Z = BRACKET_TOP_Z_LOCAL - BRACKET_BOTTOM_Z_LOCAL  # = 23.3

# Pin bore — Ø5.4 = 0.4 mm diametric clearance over the M5 stock pin.
PIN_BORE_D = TURRET_HINGE_PIN_D + 0.4                           # = 5.4
_PIN_Z_BRACKET_LOCAL = PIN_AXIS_Z_LOCAL - BRACKET_BOTTOM_Z_LOCAL  # = 4.8

# Slice (c) bushing block half-width along the pin axis (mirrors slice
# (c)'s BUSHING_BLOCK_W = 20).
_BUSHING_HALF_X = 10.0
_PIN_AIR_GAP = 0.5               # bracket-to-bushing-block clearance

BRACKET_X_CENTER_OFFSET = (
    _BUSHING_HALF_X + _PIN_AIR_GAP + BRACKET_BLOCK_X / 2          # = 13
)
BRACKET_POS_X_LOCAL = +BRACKET_X_CENTER_OFFSET
BRACKET_NEG_X_LOCAL = -BRACKET_X_CENTER_OFFSET
BRACKET_Y_LOCAL = PIN_AXIS_Y_LOCAL                                # = -60


# --- Hub-side skirt (perimeter wall extended down for bracket attach) --

# Skirt is the hub-side perimeter wall continued from z = -4 down to
# the bracket bottom. Same Y range as the perimeter wall there (5 mm
# thick). Notched at cart X ∈ [-10.5, +10.5] for the slice (c)
# bushing block (X ∈ [-10, +10], Z ∈ [-35, -10]) to pass through
# unobstructed.
HUB_SKIRT_Y_OUTER = -FRAME_OUTER_SIZE / 2                          # = -60
HUB_SKIRT_Y_INNER = HUB_SKIRT_Y_OUTER + FRAME_WALL_T               # = -55
HUB_SKIRT_Z_BOTTOM = BRACKET_BOTTOM_Z_LOCAL                        # = -27.3
HUB_SKIRT_Z_TOP = ACRYLIC_BOTTOM_Z                                 # = -4

# Bushing-block notch: clears the slice (c) block X ∈ [-10, +10] with
# the same 0.5 mm air gap used for bracket-to-block clearance.
_BUSHING_NOTCH_HALF_X = _BUSHING_HALF_X + _PIN_AIR_GAP             # = 10.5
_BUSHING_NOTCH_X_LEN = 2 * _BUSHING_NOTCH_HALF_X                   # = 21
# Notch only needs to cover the bushing-block Z range; above z = -10
# the skirt is intact across the full X span.
_BUSHING_BLOCK_TOP_Z = -10.0    # mirrors slice (c) BUSHING_BLOCK_W Z


# --- Kicker extension (cantilevered from skirt past hinge) -------------

# Extension reaches the lift point at cart-frame (X=+30, Y=-100) — a
# 40 mm lever arm hub-side of the hinge axis at cart Y=-60. Cart X=+30
# is well clear of the slice (c) DUMP spider arm (cart X ∈ [-7, +7]
# at the IMAGE-pose equivalent heading) and well inside the frame's
# +X edge (+60). Per-camera below-view occlusion is rig-level
# (visibility.py), not asserted here.
EXTENSION_X_CENTER = +30.0
EXTENSION_X_LEN = 18.0
EXTENSION_X_INNER = EXTENSION_X_CENTER - EXTENSION_X_LEN / 2       # = +21
EXTENSION_X_OUTER = EXTENSION_X_CENTER + EXTENSION_X_LEN / 2       # = +39

# Y span: from skirt inner face (-55) to the tip past the lift point.
EXTENSION_Y_OUTBOARD = HUB_SKIRT_Y_INNER                           # = -55
EXTENSION_Y_TIP = -111.0
LIFT_POINT_Y_LOCAL = -100.0     # = cart Y where lifter pad center contacts

# L-shape: stem at the acrylic-bottom band (thin), drop block at the
# tip hangs down to the lifter contact face.
EXTENSION_LIFTER_CLEARANCE_Z = 2.0
EXTENSION_BOTTOM_Z = (
    TURRET_DUMP_KICKER_LIFTER_TOP_Z + EXTENSION_LIFTER_CLEARANCE_Z  # = -43
)
EXTENSION_STEM_BOTTOM_Z = -15.0
EXTENSION_STEM_TOP_Z = ACRYLIC_BOTTOM_Z                            # = -4
# Drop block Y span centered on lift point with margin for pad coverage.
EXTENSION_DROP_Y_OUTBOARD = -89.0
EXTENSION_DROP_Y_TIP = EXTENSION_Y_TIP                             # = -111


# --- Sanity asserts ----------------------------------------------------

# Pin bore wall to bracket Z (top + bottom) faces.
assert _PIN_Z_BRACKET_LOCAL - PIN_BORE_D / 2 >= WALL_MIN, (
    f"pin-bore to bracket bottom-Z wall "
    f"{_PIN_Z_BRACKET_LOCAL - PIN_BORE_D / 2:.2f} mm < WALL_MIN ({WALL_MIN}); "
    f"raise BRACKET_BLOCK_Z or move TURRET_HINGE_PIN_Z down"
)
assert (BRACKET_BLOCK_Z - _PIN_Z_BRACKET_LOCAL) - PIN_BORE_D / 2 >= WALL_MIN, (
    f"pin-bore to bracket top-Z wall "
    f"{(BRACKET_BLOCK_Z - _PIN_Z_BRACKET_LOCAL) - PIN_BORE_D / 2:.2f} mm < "
    f"WALL_MIN ({WALL_MIN})"
)

# Pin bore wall to bracket Y (front + back) faces. Bore centred on
# bracket Y centerline so both walls equal.
_pin_y_wall = BRACKET_BLOCK_Y / 2 - PIN_BORE_D / 2
assert _pin_y_wall >= WALL_MIN, (
    f"pin-bore to bracket Y wall {_pin_y_wall:.2f} mm < WALL_MIN "
    f"({WALL_MIN}); raise BRACKET_BLOCK_Y"
)

# Bracket inboard X face (toward bushing block) clears the bushing block.
_inboard_face_x = abs(BRACKET_POS_X_LOCAL) - BRACKET_BLOCK_X / 2
assert _inboard_face_x - _BUSHING_HALF_X >= _PIN_AIR_GAP - 1e-9, (
    f"bracket inboard X face {_inboard_face_x:.2f} mm closer than "
    f"AIR_GAP ({_PIN_AIR_GAP}) to bushing block face "
    f"({_BUSHING_HALF_X}); shift the bracket outboard"
)

# Bracket outboard face inside frame X footprint.
assert (
    abs(BRACKET_POS_X_LOCAL) + BRACKET_BLOCK_X / 2
    <= FRAME_OUTER_SIZE / 2
), "bracket outboard X face overhangs frame edge"

# Frame perimeter wall thickness ≥ WALL_MIN.
assert FRAME_WALL_T >= WALL_MIN, (
    f"frame perimeter wall thickness {FRAME_WALL_T:.2f} mm < WALL_MIN "
    f"({WALL_MIN}); shrink ACRYLIC_INSERT_SIZE or grow FRAME_OUTER_SIZE"
)

# Acrylic insert covers the imaged region.
assert ACRYLIC_INSERT_SIZE >= IMAGED_REGION_SIDE_MM, (
    f"acrylic insert {ACRYLIC_INSERT_SIZE} < imaged region "
    f"{IMAGED_REGION_SIDE_MM} mm; cameras would see frame instead of "
    f"acrylic at the imaged region edge"
)
# (Per-camera below-view occlusion of corner lips and kicker extension
# is rig-level, checked by visibility.py — not asserted here.)

# Corner lip width ≥ WALL_MIN.
assert CORNER_LIP_SIZE >= WALL_MIN, (
    f"corner lip size {CORNER_LIP_SIZE} < WALL_MIN ({WALL_MIN})"
)
assert CORNER_LIP_T >= WALL_MIN, (
    f"corner lip thickness {CORNER_LIP_T} < WALL_MIN ({WALL_MIN})"
)

# Bushing-block notch clears the slice (c) block X face with AIR_GAP.
assert _BUSHING_NOTCH_HALF_X >= _BUSHING_HALF_X + _PIN_AIR_GAP - 1e-9, (
    f"bushing-block notch half-width {_BUSHING_NOTCH_HALF_X} mm "
    f"insufficient over block half-width {_BUSHING_HALF_X} + AIR_GAP "
    f"({_PIN_AIR_GAP})"
)

# Bushing-block notch leaves wall material between notch outer face and
# bracket inner face.
_notch_to_bracket_x = (
    abs(BRACKET_POS_X_LOCAL) - BRACKET_BLOCK_X / 2 - _BUSHING_NOTCH_HALF_X
)
assert _notch_to_bracket_x >= -1e-9, (
    f"bushing notch ({_BUSHING_NOTCH_HALF_X}) overlaps bracket inner "
    f"face ({abs(BRACKET_POS_X_LOCAL) - BRACKET_BLOCK_X/2}); brackets "
    f"would lose their hub-side skirt connection"
)

# Extension X within frame footprint.
assert EXTENSION_X_OUTER <= FRAME_OUTER_SIZE / 2, (
    f"extension outer X face {EXTENSION_X_OUTER} overhangs frame edge "
    f"{FRAME_OUTER_SIZE / 2}"
)

# Extension Y reaches the lift point.
assert EXTENSION_Y_TIP <= LIFT_POINT_Y_LOCAL, (
    f"extension Y tip {EXTENSION_Y_TIP} doesn't reach lift point Y "
    f"{LIFT_POINT_Y_LOCAL}"
)

# Extension joins the skirt at the hub-side wall.
assert EXTENSION_Y_OUTBOARD == HUB_SKIRT_Y_INNER, (
    f"extension outboard end {EXTENSION_Y_OUTBOARD} doesn't meet skirt "
    f"inner face {HUB_SKIRT_Y_INNER}"
)

# Extension clear of the +X bracket in cart X.
_BRACKET_POS_OUTER_X = BRACKET_POS_X_LOCAL + BRACKET_BLOCK_X / 2  # = +15.5
_EXT_TO_BRACKET_X_GAP = EXTENSION_X_INNER - _BRACKET_POS_OUTER_X
assert _EXT_TO_BRACKET_X_GAP >= _PIN_AIR_GAP - 1e-9, (
    f"extension inner X face {EXTENSION_X_INNER} too close to +X "
    f"bracket outer X face {_BRACKET_POS_OUTER_X} (gap "
    f"{_EXT_TO_BRACKET_X_GAP} < AIR_GAP {_PIN_AIR_GAP})"
)

# Lever arm to lift point ≥ minimum for reliable dump.
_LEVER_ARM = abs(LIFT_POINT_Y_LOCAL - PIN_AXIS_Y_LOCAL)            # = 40
assert _LEVER_ARM >= 30.0, (
    f"lever arm hinge→lift {_LEVER_ARM:.2f} mm < 30 mm; dump may "
    f"be unreliable — push EXTENSION_Y_TIP further hub-ward"
)

# (Extension below-view occlusion is rig-level — visibility.py.)

# Extension bottom strictly above lifter pad top at REST.
assert EXTENSION_BOTTOM_Z > TURRET_DUMP_KICKER_LIFTER_TOP_Z, (
    f"extension bottom {EXTENSION_BOTTOM_Z} ≤ lifter top at REST "
    f"{TURRET_DUMP_KICKER_LIFTER_TOP_Z}; would contact at rest"
)

# Lifter pad cart-frame position derived from turret-local matches the
# extension's lift point. Option a' (2026-05-05) DUMP at heading 270°
# uses Rz(+180) + centroid translate (0, -ARM_R), so the inversion
# from turret-local back to cart-frame is:
#   cart_x = -turret_x
#   cart_y = -(turret_y + ARM_R) = -turret_y - ARM_R
_LIFTER_CART_X = -TURRET_DUMP_KICKER_LIFTER_TURRET_X                # = +30
_LIFTER_CART_Y = -TURRET_DUMP_KICKER_LIFTER_TURRET_Y - TURRET_ARM_R  # = -100
assert _LIFTER_CART_X == EXTENSION_X_CENTER, (
    f"lifter cart X {_LIFTER_CART_X} ≠ extension X center "
    f"{EXTENSION_X_CENTER}"
)
assert _LIFTER_CART_Y == LIFT_POINT_Y_LOCAL, (
    f"lifter cart Y {_LIFTER_CART_Y} ≠ extension lift point Y "
    f"{LIFT_POINT_Y_LOCAL}"
)


# --- Part functions ----------------------------------------------------

def acrylic_insert(
    size: float = ACRYLIC_INSERT_SIZE,
    thickness: float = ACRYLIC_INSERT_T,
):
    """110 × 110 × 4 mm PMMA insert centred on origin in X / Y, top
    face at z = 0. Drops into the frame's pocket; rests on the 4
    corner support lips at z = -4."""
    return Pos(0, 0, -thickness / 2) * Box(size, size, thickness)


def frame():
    """Single printed frame body: top ring + pocket walls + 4 corner
    lips + hub-side skirt (with bushing notch) + integrated hinge
    brackets (with horizontal pin bore) + L-shaped kicker extension."""
    # Top ring + pocket walls — one perimeter tube spanning z ∈ [-4, +5].
    perim_z_center = (TOP_RING_TOP_Z + ACRYLIC_BOTTOM_Z) / 2       # = +0.5
    perim_z_len = TOP_RING_TOP_Z - ACRYLIC_BOTTOM_Z                # = 9
    body = (
        Pos(0, 0, perim_z_center)
        * Box(FRAME_OUTER_SIZE, FRAME_OUTER_SIZE, perim_z_len)
    )
    body -= (
        Pos(0, 0, perim_z_center)
        * Box(ACRYLIC_INSERT_SIZE, ACRYLIC_INSERT_SIZE, perim_z_len * 3)
    )

    # Corner lips — 4 rectangular tabs at the pocket corners.
    lip_z_center = (CORNER_LIP_TOP_Z + CORNER_LIP_BOTTOM_Z) / 2
    for sx in (-1, +1):
        for sy in (-1, +1):
            corner_x = sx * (ACRYLIC_INSERT_SIZE / 2 - CORNER_LIP_SIZE / 2)
            corner_y = sy * (ACRYLIC_INSERT_SIZE / 2 - CORNER_LIP_SIZE / 2)
            body += (
                Pos(corner_x, corner_y, lip_z_center)
                * Box(CORNER_LIP_SIZE, CORNER_LIP_SIZE, CORNER_LIP_T)
            )

    # Hub-side skirt — full X span at cart Y ∈ [-90, -86].
    skirt_y_center = (HUB_SKIRT_Y_OUTER + HUB_SKIRT_Y_INNER) / 2   # = -88
    skirt_z_center = (HUB_SKIRT_Z_BOTTOM + HUB_SKIRT_Z_TOP) / 2    # = -15.65
    skirt_z_len = HUB_SKIRT_Z_TOP - HUB_SKIRT_Z_BOTTOM             # = 23.3
    body += (
        Pos(0, skirt_y_center, skirt_z_center)
        * Box(FRAME_OUTER_SIZE, FRAME_WALL_T, skirt_z_len)
    )

    # Notch the skirt for the slice (c) bushing block to pass through.
    notch_z_center = (HUB_SKIRT_Z_BOTTOM + _BUSHING_BLOCK_TOP_Z) / 2
    notch_z_len = _BUSHING_BLOCK_TOP_Z - HUB_SKIRT_Z_BOTTOM        # = 17.3
    body -= (
        Pos(0, skirt_y_center, notch_z_center)
        * Box(_BUSHING_NOTCH_X_LEN, FRAME_WALL_T * 3, notch_z_len)
    )

    # Hinge brackets — at cart X = ±13, hanging from the skirt.
    bracket_z_center = (BRACKET_BOTTOM_Z_LOCAL + BRACKET_TOP_Z_LOCAL) / 2
    for bracket_x in (BRACKET_POS_X_LOCAL, BRACKET_NEG_X_LOCAL):
        body += (
            Pos(bracket_x, BRACKET_Y_LOCAL, bracket_z_center)
            * Box(BRACKET_BLOCK_X, BRACKET_BLOCK_Y, BRACKET_BLOCK_Z)
        )

    # Horizontal pin bore (axis ±X) — limited to the bracket X-span
    # so the bore doesn't graze the skirt outboard of the brackets and
    # leave a thin (1.3 mm) wall sliver. The notch already removes the
    # skirt across the bore Z between brackets, so a per-bracket bore
    # is sufficient.
    for bracket_x in (BRACKET_POS_X_LOCAL, BRACKET_NEG_X_LOCAL):
        body -= (
            Pos(bracket_x, PIN_AXIS_Y_LOCAL, PIN_AXIS_Z_LOCAL)
            * Rot(0, 90, 0)
            * Cylinder(PIN_BORE_D / 2, BRACKET_BLOCK_X)
        )

    # Kicker extension — L-shape: stem at acrylic-bottom band, drop
    # block hanging down to the lifter contact face at the tip.
    stem_y_center = (EXTENSION_Y_OUTBOARD + EXTENSION_Y_TIP) / 2
    stem_y_len = EXTENSION_Y_OUTBOARD - EXTENSION_Y_TIP            # = 55
    stem_z_center = (EXTENSION_STEM_TOP_Z + EXTENSION_STEM_BOTTOM_Z) / 2
    stem_z_len = EXTENSION_STEM_TOP_Z - EXTENSION_STEM_BOTTOM_Z    # = 11
    body += (
        Pos(EXTENSION_X_CENTER, stem_y_center, stem_z_center)
        * Box(EXTENSION_X_LEN, stem_y_len, stem_z_len)
    )

    drop_y_center = (EXTENSION_DROP_Y_OUTBOARD + EXTENSION_DROP_Y_TIP) / 2
    drop_y_len = EXTENSION_DROP_Y_OUTBOARD - EXTENSION_DROP_Y_TIP  # = 22
    drop_z_center = (EXTENSION_STEM_BOTTOM_Z + EXTENSION_BOTTOM_Z) / 2
    drop_z_len = EXTENSION_STEM_BOTTOM_Z - EXTENSION_BOTTOM_Z      # = 28
    body += (
        Pos(EXTENSION_X_CENTER, drop_y_center, drop_z_center)
        * Box(EXTENSION_X_LEN, drop_y_len, drop_z_len)
    )

    return body


# --- Per-station placement helper --------------------------------------

# Rotation that maps the cartridge-local IMAGE pose onto each station's
# turret-local pose. The cartridge is built in cartridge-local with the
# pin axis along ±X and "outward" along +Y. Each station has its own
# (radial-outward, tangential) directions in turret-local; the rotation
# below picks the proper-rotation root that aligns cartridge axes onto
# them. (Option a' rename, 2026-05-05: DUMP moved to heading 270° with
# Rz(+180) — the prior RETURN rotation. The prior DUMP heading 180°
# slot is now IDLE, with the prior DUMP rotation Rz(+90).)
#
#   IMAGE  : Rz(  0°)  cartridge +X → turret +X, +Y → +Y   heading 90°
#   LOAD   : Rz(-90°)  cartridge +X → turret -Y, +Y → +X   heading 0°
#   IDLE   : Rz(+90°)  cartridge +X → turret +Y, +Y → -X   heading 180°
#   DUMP   : Rz(180°)  cartridge +X → turret -X, +Y → -Y   heading 270°
STATION_ANGLES = {
    "image": 0,
    "load":  -90,
    "idle":  90,
    "dump":  180,
}

# Acrylic insert shape-truth color — faint blue-white at α=0.25 to
# render as clear PMMA in cad-khana's flat-shaded GLB path. The
# chitra-cad photo-real path uses ``acrylic_clear`` from its material
# catalog (transmission=1.0, ior=1.49) and ignores this color.
ACRYLIC_INSERT_COLOR = Color(0.95, 0.97, 0.98, 0.25)

_STATION_CENTROIDS = {
    "image": Pos(0, +TURRET_ARM_R, 0),
    "load":  Pos(+TURRET_ARM_R, 0, 0),
    "idle":  Pos(-TURRET_ARM_R, 0, 0),
    "dump":  Pos(0, -TURRET_ARM_R, 0),
}

# Cartridge-local part placements. Used by both the standalone
# (cartridge in isolation) and the per-station builder below
# (translated + rotated to its turret-local pose).
_IMAGE_LOCS = (
    ("acrylic", Pos(0, 0, 0)),
    ("frame", Pos(0, 0, 0)),
)

_PART_BUILDERS = {
    "acrylic": acrylic_insert,
    "frame": frame,
}

def add_turret_platform(
    a: Assembly, station: str, base=Pos(0, 0, 0)
) -> Assembly:
    """Place one cartridge at the named station ('image', 'load',
    'idle', 'dump'). Cartridge geometry is built in cartridge-local
    IMAGE pose; this helper applies a per-station rotation around the
    hub axis (turret-local origin) and translates to the station's
    centroid. `base` (default: identity) is pre-multiplied to lift the
    result into a parent frame. Part names are prefixed with the
    station to avoid collisions across instances
    ("platform_load_acrylic", etc.)."""
    angle = STATION_ANGLES[station]
    centroid = _STATION_CENTROIDS[station]
    rotation = Rot(0, 0, angle)
    for sub_name, image_loc in _IMAGE_LOCS:
        full_name = f"platform_{station}_{sub_name}"
        # The acrylic insert carries an intrinsic translucent baseColor
        # — design-system ``color_of`` returns None for it (intrinsic
        # material colour), but the shape-truth GLB needs an alpha
        # channel to render through. Faint blue-white at α=0.25 reads
        # as clear PMMA in <model-viewer>'s flat-shaded path; the
        # photo-real Cycles path (chitra-cad) uses the full
        # transmission/IOR material instead and ignores this color.
        color = (
            ACRYLIC_INSERT_COLOR
            if sub_name == "acrylic"
            else color_of(full_name)
        )
        a = a.with_part(
            full_name,
            _PART_BUILDERS[sub_name](),
            location=base * centroid * rotation * image_loc,
            material=material_of(full_name),
            color=color,
        )
    return a


# --- Standalone assembly (cartridge-local, no station placement) ------

# Build the cartridge in cartridge-local with no station rotation /
# translation. Pairwise no-interference checks are invariant under
# rigid motion; this verifies the cartridge geometry alone.
assembly = Assembly()
_STANDALONE_NAMES = tuple(f"cartridge_{sub_name}" for sub_name, _ in _IMAGE_LOCS)
for (sub_name, image_loc), full_name in zip(_IMAGE_LOCS, _STANDALONE_NAMES):
    assembly = assembly.with_part(full_name, _PART_BUILDERS[sub_name](), location=image_loc)

for _i, _a in enumerate(_STANDALONE_NAMES):
    for _b in _STANDALONE_NAMES[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    inspect(
        frame(),
        method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
        out="outputs",
        name="frame",
    )
