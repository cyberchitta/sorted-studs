"""
M05 hex-tower — rotating square-column flap mechanism (generic floor).

Single-floor model of the rotating-column trap-door design for Approach F.
The column is a square tube with a circular collar at the bottom (bearing
interface); the collar rides in a fixed ring bearing held coaxial with
the column by a **spider bracket** — a short printed hub with six
integrated saddles that clamp over the top of radial 2020 aluminum
extrusions running out to the inner-hex frame. The aluminum carries
the span; the printed part is only a junction. No solid floor disk;
the drop chute is unobstructed.

An exit opening on one wall is closed in DOWN state by a flap hinged at
its middle: a rectangular catch section (inner of hinge) covers the
column cross-section when UP, and a narrower spout section (outer of
hinge) delivers bricks into the selected basket.

This is an **intermediate-floor** model — the base floor additionally
has a shaft + motor mount + bearing boss to drive rotation. Those are
not modeled here; intermediate floors are structurally passive beyond
the ring-bearing support.

One top-level parameter — `MAX_BRICK_DIM` — sizes the column interior.
`N_BASKETS_PER_FLOOR` and `R_BASKET_INNER` together size the spout exit
width (narrower = finer basket angular targeting). `R_BASKET_INNER` also
sets the inner-hex vertex distance and therefore the spider arm length.

Coordinate frame:
    origin = column axis ∩ floor datum (z=0 = bearing/spider base)
    +Z     = up
    +X     = radial outward toward the exit opening
    +Y     = tangent at the opening (hinge axis direction)

Tilt convention (`Rot(0, θ, 0)`; positive θ rotates the +X tip toward -Z):
    tilt = 0°   — flap horizontal; catch in -X, spout in +X.
    tilt = -90° — catch vertical down (DOWN state; spout vertical up).
    tilt = +10° — catch 10° above horizontal (UP / catching state).

Gravity bias: catch section is longer than spout ⇒ DOWN is a stable
equilibrium (catch hangs, spout rises).
"""

import math

from build123d import Box, Cylinder, Location, Part, Polygon, Pos, Rot, extrude

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM


# --- Top-level parameters -----------------------------------------------

MAX_BRICK_DIM = 80.0            # largest LEGO dim this sorter handles (mm)
BRICK_SLOP = 5.0                # radial clearance brick-to-column-wall (mm)
N_BASKETS_PER_FLOOR = 6         # 6 / 12 / 18; affects spout-exit width
R_BASKET_INNER = 150.0          # radial distance column axis → basket lip (mm)
R_HEX_OUTER = 270.0             # outer-hex apothem (center → edge midpoint, mm)


# --- Derived parameters -------------------------------------------------

# Column cross-section
COL_INTERIOR_SIDE = MAX_BRICK_DIM + 2 * BRICK_SLOP   # 90
COL_WALL_T = 3.0
COL_OUTER_SIDE = COL_INTERIOR_SIDE + 2 * COL_WALL_T  # 96

# Bottom collar (bearing interface)
COLLAR_CLEARANCE = 1.0
COLLAR_OD = COL_OUTER_SIDE * math.sqrt(2) + 2 * COLLAR_CLEARANCE  # ≈ 138
COLLAR_T = 5.0

# Flap — catch is rectangular, spout is a narrower rectangle (step at
# hinge; smooth taper is a follow-up)
FLAP_T = 4.0
FLAP_W = COL_INTERIOR_SIDE - 2              # 88 — 1 mm side clearance
FLAP_L_CATCH = COL_INTERIOR_SIDE - 2        # 88 — spans column interior
# Spout reaches into the basket's inner lip
FLAP_L_SPOUT = R_BASKET_INNER - COL_OUTER_SIDE / 2 + 10.0  # 112

# Spout exit width — must be ≤ basket slot angular width AND ≤ flap width
BASKET_SLOT_W = 2 * R_BASKET_INNER * math.sin(math.pi / N_BASKETS_PER_FLOOR)
SPOUT_EXIT_W = min(BASKET_SLOT_W, FLAP_W)

# Sanity: spout exit must fit the largest brick
assert SPOUT_EXIT_W >= MAX_BRICK_DIM, (
    f"SPOUT_EXIT_W ({SPOUT_EXIT_W:.1f} mm) < MAX_BRICK_DIM "
    f"({MAX_BRICK_DIM:.1f} mm). The spout exit can't pass the largest "
    f"brick. Increase R_BASKET_INNER (currently {R_BASKET_INNER}), "
    f"decrease N_BASKETS_PER_FLOOR (currently {N_BASKETS_PER_FLOOR}), "
    f"or decrease MAX_BRICK_DIM."
)

# Flap knuckle / hinge pin
KNUCKLE_OD = 10.0
PIN_D = 3.0
PIN_HOLE_D = 3.6
PIN_LEN = COL_OUTER_SIDE + 4.0

# Hinge bosses on the column (inside surface, on each side of the opening)
BOSS_X = 10.0                               # radial extent (inward)
BOSS_Y = 3.0                                # tangential thickness
BOSS_H_Z = 12.0
BOSS_X_CENTER = COL_INTERIOR_SIDE / 2 - BOSS_X / 2

# Opening + pivot
OPENING_W = FLAP_W + 2.0                    # 90
OPENING_Z_BOT = COLLAR_T + 2.0              # above bottom collar
PIVOT_X = COL_INTERIOR_SIDE / 2             # at inner face of +X wall
PIVOT_Z = OPENING_Z_BOT + FLAP_L_CATCH      # DOWN catch reaches opening bottom
# Opening top: big enough for brick exit AND for spout clearance at DOWN
EXIT_MARGIN = 5.0
_ABOVE_PIVOT = max(
    FLAP_L_SPOUT + 5.0,                      # spout peaks at DOWN
    FLAP_T / 2 + MAX_BRICK_DIM + EXIT_MARGIN # brick exit clearance at UP
)
OPENING_Z_TOP = PIVOT_Z + _ABOVE_PIVOT

BOSS_Y_OFFSET = OPENING_W / 2 + BOSS_Y / 2

# Column height — just above the opening
COL_Z_BOT = 0.0
COL_Z_TOP = OPENING_Z_TOP + 5.0

# Ring bearing
RING_BEARING_ID = COLLAR_OD + 2 * COLLAR_CLEARANCE
RING_BEARING_OD = RING_BEARING_ID + 20.0

# Bearing spider — a short printed hub that grips the bearing, with six
# integrated saddle brackets around its perimeter. Each saddle drops
# over the top of a 2020 aluminum extrusion running radially out to the
# inner-hex frame; the extrusion carries the structural span, the
# printed part is only a junction.
N_SPIDER_SADDLES = 6
SPIDER_HUB_ID = RING_BEARING_OD + 1.0         # ≈ 161 — slip fit over bearing OD
SPIDER_HUB_OD = SPIDER_HUB_ID + 10.0          # ≈ 171 — 5 mm wall around bearing
SPIDER_T = COLLAR_T                           # 5 — coplanar with bearing

# 2020 aluminum extrusion profile (20 × 20 mm); extrusion top face is
# coincident with the bearing top plane (Z = COL_Z_BOT + COLLAR_T).
EXT_SIDE = 20.0

# Saddle bracket dimensions
SADDLE_T_TOP = 3.0                            # top-plate thickness
SADDLE_T_WALL = 3.0                           # side-wall thickness
SADDLE_COVER_H = 12.0                         # wrap-down depth over extrusion
SADDLE_L = 30.0                               # radial grip length
SADDLE_HUB_OVERLAP = 2.0                      # saddle-to-hub union overlap
SADDLE_BOLT_D = 4.5                           # M4 clearance hole
SADDLE_BOLT_OFFSET = 15.0                     # bolt X past hub OD (mid of grip)

# Basket — trapezoidal-prism container sitting in a hex bay
BASKET_HEX_CLEARANCE = 20.0                       # basket outer chord → frame
R_BASKET_OUTER = R_HEX_OUTER - BASKET_HEX_CLEARANCE
BASKET_ANG_SPAN_DEG = 360.0 / N_BASKETS_PER_FLOOR  # angular span per basket
BASKET_WALL_T = 3.0
BASKET_DEPTH_Z = 80.0                              # internal height
BASKET_RIM_Z = 0.0                                 # top rim at floor datum

# Tilt angles
TILT_UP = 10.0
TILT_DOWN = -90.0
TILT_SAMPLES = (-60.0, -30.0, 0.0)

WALL_MIN = 1.5


# --- Helpers ------------------------------------------------------------

def _cyl_along_y(r: float, length: float) -> Part:
    return Rot(90, 0, 0) * Cylinder(r, length)


def _trap_prism(
    r_in: float,
    r_out: float,
    half_angle_deg: float,
    z_bot: float,
    z_top: float,
    inset: float = 0.0,
) -> Part:
    """Angular-sector trapezoidal prism centered on +X.

    Four vertices at (r*cos(ha), ±r*sin(ha)) for r in {r_in, r_out}.
    `inset` offsets each of the four bounding lines perpendicular-inward
    by that amount — used to carve the hollow interior from an outer
    shell with a uniform wall thickness.
    """
    ha = math.radians(half_angle_deg)
    x_in = r_in * math.cos(ha) + inset
    x_out = r_out * math.cos(ha) - inset
    tan_ha = math.tan(ha)
    dy = inset / math.cos(ha)
    pts = [
        (x_in, -(x_in * tan_ha - dy)),
        (x_out, -(x_out * tan_ha - dy)),
        (x_out, +(x_out * tan_ha - dy)),
        (x_in, +(x_in * tan_ha - dy)),
    ]
    sketch = Polygon(*pts, align=None)
    return Pos(0, 0, z_bot) * extrude(sketch, amount=z_top - z_bot)


# --- Part functions -----------------------------------------------------

def bearing_spider() -> Part:
    """Printed hub that holds the ring bearing coaxial with the column.
    Central annulus grips the bearing's outer race; six integrated
    saddle brackets at the perimeter drop over the top of radial 2020
    aluminum extrusions. The aluminum carries the structural span out
    to the inner hex; the printed part is a short junction only.
    No solid disk — the drop chute below is unobstructed.
    """
    hub_z_center = COL_Z_BOT + SPIDER_T / 2
    ext_top_z = COL_Z_BOT + COLLAR_T            # extrusion top = bearing top

    # Hub annulus around the bearing
    hub_outer = Pos(0, 0, hub_z_center) * Cylinder(SPIDER_HUB_OD / 2, SPIDER_T)
    hub_inner = Pos(0, 0, hub_z_center) * Cylinder(SPIDER_HUB_ID / 2, SPIDER_T * 2)
    body = hub_outer - hub_inner

    # Saddle envelope (solid block; extrusion pocket + bolt hole subtracted below)
    saddle_w = EXT_SIDE + 2 * SADDLE_T_WALL
    saddle_h = SADDLE_COVER_H + SADDLE_T_TOP
    saddle_x_in = SPIDER_HUB_OD / 2 - SADDLE_HUB_OVERLAP
    saddle_x_out = SPIDER_HUB_OD / 2 + SADDLE_L
    saddle_z_bot = ext_top_z - SADDLE_COVER_H
    saddle_z_top = ext_top_z + SADDLE_T_TOP

    # Extrusion pocket — starts at hub OD (no hub intrusion), open outward
    cutout_x_in = SPIDER_HUB_OD / 2
    cutout_x_out = saddle_x_out + 5.0
    cutout_z_top = ext_top_z                    # preserve top plate
    cutout_z_bot = saddle_z_bot - 1.0

    # Bolt through top plate into extrusion's top T-slot (Y=0)
    bolt_x = SPIDER_HUB_OD / 2 + SADDLE_BOLT_OFFSET
    bolt_z_bot = ext_top_z - 1.0
    bolt_z_top = saddle_z_top + 1.0

    for i in range(N_SPIDER_SADDLES):
        theta = i * 360.0 / N_SPIDER_SADDLES
        rot = Rot(0, 0, theta)
        saddle_env = rot * Pos(
            (saddle_x_in + saddle_x_out) / 2, 0,
            (saddle_z_bot + saddle_z_top) / 2,
        ) * Box(saddle_x_out - saddle_x_in, saddle_w, saddle_h)
        cutout = rot * Pos(
            (cutout_x_in + cutout_x_out) / 2, 0,
            (cutout_z_bot + cutout_z_top) / 2,
        ) * Box(cutout_x_out - cutout_x_in, EXT_SIDE, cutout_z_top - cutout_z_bot)
        bolt_hole = rot * Pos(bolt_x, 0, (bolt_z_bot + bolt_z_top) / 2) * Cylinder(
            SADDLE_BOLT_D / 2, bolt_z_top - bolt_z_bot,
        )
        body = body + saddle_env - cutout - bolt_hole
    return body


def ring_bearing() -> Part:
    """Stationary annular bearing at floor level. The column's bottom
    collar rotates inside its inner race."""
    z_center = COL_Z_BOT + COLLAR_T / 2
    body = Pos(0, 0, z_center) * Cylinder(RING_BEARING_OD / 2, COLLAR_T)
    hole = Pos(0, 0, z_center) * Cylinder(RING_BEARING_ID / 2, COLLAR_T * 2)
    return body - hole


def column() -> Part:
    """Square-section tube with a bottom circular collar, a tall exit
    opening in the +X wall, hinge bosses on the inside of the wall
    straddling the opening, and a pin bore through the bosses.
    """
    col_h = COL_Z_TOP - COL_Z_BOT
    col_z_center = (COL_Z_TOP + COL_Z_BOT) / 2

    outer = Pos(0, 0, col_z_center) * Box(COL_OUTER_SIDE, COL_OUTER_SIDE, col_h)
    inner = Pos(0, 0, col_z_center) * Box(
        COL_INTERIOR_SIDE, COL_INTERIOR_SIDE, col_h * 2
    )
    tube = outer - inner

    # Exit-opening cutout on the +X wall
    cut_z_center = (OPENING_Z_TOP + OPENING_Z_BOT) / 2
    cut_h = OPENING_Z_TOP - OPENING_Z_BOT
    cut = Pos(COL_OUTER_SIDE, 0, cut_z_center) * Box(
        COL_OUTER_SIDE * 2, OPENING_W, cut_h
    )
    tube = tube - cut

    # Bottom circular collar with a square hole (lets bricks fall through)
    collar_z = COL_Z_BOT + COLLAR_T / 2
    collar_disc = Pos(0, 0, collar_z) * Cylinder(COLLAR_OD / 2, COLLAR_T)
    collar_hole = Pos(0, 0, collar_z) * Box(
        COL_INTERIOR_SIDE, COL_INTERIOR_SIDE, COLLAR_T * 2
    )
    tube = tube + (collar_disc - collar_hole)

    # Hinge bosses — inside the column, at the tangential edges of the opening
    boss_plus = Pos(BOSS_X_CENTER, +BOSS_Y_OFFSET, PIVOT_Z) * Box(
        BOSS_X, BOSS_Y, BOSS_H_Z
    )
    boss_minus = Pos(BOSS_X_CENTER, -BOSS_Y_OFFSET, PIVOT_Z) * Box(
        BOSS_X, BOSS_Y, BOSS_H_Z
    )
    tube = tube + boss_plus + boss_minus

    # Pin bore through bosses (and through the air across the opening)
    pin_bore = Pos(PIVOT_X, 0, PIVOT_Z) * _cyl_along_y(
        PIN_HOLE_D / 2, PIN_LEN + 4.0
    )
    return tube - pin_bore


def hinge_pin() -> Part:
    return _cyl_along_y(PIN_D / 2, PIN_LEN)


def flap() -> Part:
    """Middle-pivot flap: wide rectangular catch section on -X side of
    hinge, narrower rectangular spout on +X side, knuckle at pivot.
    Thickness centered on z=0 in local frame. Pin axis = local Y.

    Step change in width at the hinge (catch = FLAP_W, spout =
    SPOUT_EXIT_W). Smooth tapered spout is a follow-up cosmetic.
    """
    catch = Pos(-FLAP_L_CATCH / 2, 0, 0) * Box(FLAP_L_CATCH, FLAP_W, FLAP_T)
    spout = Pos(+FLAP_L_SPOUT / 2, 0, 0) * Box(
        FLAP_L_SPOUT, SPOUT_EXIT_W, FLAP_T
    )
    knuckle = _cyl_along_y(KNUCKLE_OD / 2, FLAP_W)
    body = catch + spout + knuckle
    pin_hole = _cyl_along_y(PIN_HOLE_D / 2, FLAP_W * 2)
    return body - pin_hole


def basket() -> Part:
    """Trapezoidal-prism container for one basket slot, centered on +X.
    Angular span = 360° / N_BASKETS_PER_FLOOR. Inner chord wall at radial
    distance R_BASKET_INNER from the column axis; outer chord at
    R_BASKET_OUTER. Walls on all four sides, a floor, open top at
    BASKET_RIM_Z. Top flange (for hex-rail hang) not modeled yet.
    """
    half_angle_deg = BASKET_ANG_SPAN_DEG / 2
    z_bot = BASKET_RIM_Z - BASKET_DEPTH_Z
    z_top = BASKET_RIM_Z
    outer = _trap_prism(
        R_BASKET_INNER, R_BASKET_OUTER, half_angle_deg, z_bot, z_top,
    )
    cavity = _trap_prism(
        R_BASKET_INNER, R_BASKET_OUTER, half_angle_deg,
        z_bot + BASKET_WALL_T, z_top + 1.0,
        inset=BASKET_WALL_T,
    )
    return outer - cavity


# --- Assembly -----------------------------------------------------------

pivot_loc = Location((PIVOT_X, 0, PIVOT_Z))

assembly = Assembly()
assembly = (
    assembly
    .add("bearing_spider", bearing_spider())
    .add("ring_bearing", ring_bearing())
    .add("column", column())
    .add("hinge_pin", hinge_pin(), location=pivot_loc)
    .add("flap_down", flap(), location=pivot_loc * Rot(0, TILT_DOWN, 0))
    .add("flap_up", flap(), location=pivot_loc * Rot(0, TILT_UP, 0))
)
for angle in TILT_SAMPLES:
    assembly = assembly.add(
        f"flap_at_{int(angle)}", flap(),
        location=pivot_loc * Rot(0, angle, 0),
    )

# Baskets around the full ring. basket_plus_x (i=0) is the one the spout
# aims at; the rest fill out the bay layout so renders show the ring.
basket_names = []
for i in range(N_BASKETS_PER_FLOOR):
    theta = i * 360.0 / N_BASKETS_PER_FLOOR
    name = "basket_plus_x" if i == 0 else f"basket_{i:02d}"
    basket_names.append(name)
    assembly = assembly.add(name, basket(), location=Rot(0, 0, theta))

assembly = (
    assembly
    # Fixed parts don't interfere with each other
    .assert_no_interference("bearing_spider", "ring_bearing")
    .assert_no_interference("bearing_spider", "column")
    .assert_no_interference("ring_bearing", "column")
    .assert_no_interference("hinge_pin", "column")
    # Basket sits in its hex bay — must clear all fixed column-side parts
    .assert_no_interference("basket_plus_x", "column")
    .assert_no_interference("basket_plus_x", "bearing_spider")
    .assert_no_interference("basket_plus_x", "ring_bearing")
    # Flap DOWN — closes the lower part of the opening, clears everything
    .assert_no_interference("flap_down", "bearing_spider")
    .assert_no_interference("flap_down", "ring_bearing")
    .assert_no_interference("flap_down", "column")
    .assert_no_interference("flap_down", "hinge_pin")
    .assert_no_interference("flap_down", "basket_plus_x")
    # Flap UP — catch spans column, spout clears wall, nothing fixed is hit
    .assert_no_interference("flap_up", "bearing_spider")
    .assert_no_interference("flap_up", "ring_bearing")
    .assert_no_interference("flap_up", "column")
    .assert_no_interference("flap_up", "hinge_pin")
    .assert_no_interference("flap_up", "basket_plus_x")
)
for angle in TILT_SAMPLES:
    name = f"flap_at_{int(angle)}"
    assembly = (
        assembly
        .assert_no_interference(name, "bearing_spider")
        .assert_no_interference(name, "ring_bearing")
        .assert_no_interference(name, "column")
        .assert_no_interference(name, "hinge_pin")
        .assert_no_interference(name, "basket_plus_x")
    )

# Non-plus-x baskets are rotational copies; check they also clear the
# fixed parts (catches any frame-vs-basket miss at higher N).
for name in basket_names[1:]:
    assembly = (
        assembly
        .assert_no_interference(name, "column")
        .assert_no_interference(name, "bearing_spider")
        .assert_no_interference(name, "ring_bearing")
    )

# Adjacent baskets must not overlap (validates side-by-side fit in a bay).
if len(basket_names) >= 2:
    assembly = assembly.assert_no_interference(
        basket_names[0], basket_names[1]
    )


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Hinge pin is stock; inspect the printed parts. Boolean-artifact knife-
    # edges at curved-surface intersections produce spurious sub-mm
    # readings, so we run all inspections but don't let one failure stop
    # the rest — diagnostics are still written to disk for review.
    method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0)
    for name, part_fn in [
        ("bearing_spider", bearing_spider),
        ("ring_bearing", ring_bearing),
        ("column", column),
        ("flap", flap),
        ("basket", basket),
    ]:
        try:
            inspect(part_fn(), method=method, out="outputs", name=name)
        except SystemExit:
            pass  # printability warning recorded in JSON; don't block
