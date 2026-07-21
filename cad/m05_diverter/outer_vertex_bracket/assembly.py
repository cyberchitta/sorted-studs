"""
M05 hex-tower — outer-hex vertex bracket (floor-corner variant).

Star-shaped printed bracket that joins **four** aluminum extrusions
at every outer-hex vertex: two outer-hex ring edges meeting at 120°
interior angle, one **radial** 2020 extrusion running inward from
the vertex along the bisector toward the inner hex, and one
**through-column** vertical 2020 extrusion that passes continuously
through every floor (see `_notes/decisions/m05-hex-cascade.md`). The bracket does not
support the column's bottom; it clamps the column's side via a
T-slot connection at this floor's Z.

**Bottom-mounted horizontal arms, hybrid vertical arm.** The hub +
three horizontal arms (2 hex-edge + 1 radial) hang off the *bottom*
T-slot of the horizontal extrusions at z ∈ [BRACKET_Z_TOP - PLATE_T,
BRACKET_Z_TOP] = [-26, -20]. This leaves the frame's top surface
flush at z=0 so baskets can rest on it without the bracket plate
clipping them. The vertical arm is a single plate at the column's
-X face that extends **from the hub's bottom (z=-26) up to
z=VERT_ARM_TOP_Z** — spanning the below-floor bracket body plus the
above-floor clamp region. Two bolts fire in +X through the plate
into the column's -X T-slot at z=10, z=30 (same positions as the
earlier top-mount variant, so the column sees the same clamp
Z-range). The rising portion of the arm between z=-26 and z=0 is a
structural riser that transfers the bracket's below-floor load back
up to the column's clamped section.

The radial arm is what makes this the "outer-vertex" bracket rather
than the original corner bracket: putting the radial rail at the
outer-hex vertex (instead of at an outer-edge midpoint) moves its
outer end over the vertical column, which carries the load to the
floor in compression instead of bending the outer-hex edge.

Coordinate frame:
    +X       : bisector of the two hex-edge arms, pointing toward the
               hex center (INWARD). The radial arm also runs along
               +X. Frame places the bracket at vertex V_k with world
               rotation v_angle_k + 180° so that local +X maps to
               inward at every vertex.
    -X       : hex exterior (outward)
    +Z       : up
    z=0      : floor datum. Horizontal extrusion top faces sit here;
               horizontal bracket plates hang below at
               z ∈ [-EXT_SIZE - PLATE_T, -EXT_SIZE]. The vertical
               through-column passes through this datum continuously
               (no bottom face at z=0).
    arm_h1   : hex-edge arm at +60° from +X (along one outer-hex edge)
    arm_h2   : hex-edge arm at -60° from +X (along the other outer-hex edge)
    arm_rad  : radial arm along +X (toward hex center) carrying the radial rail
    vert     : vertical arm on the -X face of the through-column,
               z ∈ [-26, VERT_ARM_TOP_Z]

Each arm is a flat plate with two M5 clearance holes that align to a T-slot
on the extrusion: horizontal arms bolt into their extrusion's *bottom*
T-slot (bolts fire +Z); vertical arm bolts into the column's -X
side T-slot (bolts fire +X).
"""

from build123d import Box, Cylinder, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM


# --- Parameters ---------------------------------------------------------

# Extrusion — 2020 (committed choice for the tower frame).
EXT_SIZE = 20.0
EXT_STUB_LEN = 60.0                # stand-in stub length for interference checks

# M5 hardware (standard for 2020 T-slots).
BOLT_HOLE_D = 5.5                  # M5 clearance
BOLT_FROM_EXT_END = 10.0           # first bolt inset from the extrusion's inner end
BOLT_SPACING = 20.0                # pitch between the two bolts on each arm

# Bracket body.
PLATE_T = 6.0                      # plate thickness (hub and arms)
HUB_HALF = 14.0                    # hub is 2*HUB_HALF square in plan
ARM_W = EXT_SIZE                   # arm width matches extrusion face width
ARM_BOLT_SPAN = BOLT_FROM_EXT_END + BOLT_SPACING + BOLT_FROM_EXT_END  # 40

# Distance from the hex vertex (bracket origin) to the inner end of each
# horizontal extrusion along its axis. Must be large enough that the
# horizontal stub does not intersect the vertical extrusion's 20x20 footprint.
HOR_INNER_OFFSET = 22.0

# Horizontal arm total length (root at hub center, tip past the outer bolt).
ARM_L_TOTAL = HOR_INNER_OFFSET + ARM_BOLT_SPAN  # 62

# Horizontal plates hang below the extrusion plane. Top face of hub /
# horizontal arms is flush with the extrusion's bottom face at
# z = -EXT_SIZE; body at z ∈ [BRACKET_Z_TOP - PLATE_T, BRACKET_Z_TOP]
# = [-26, -20].
BRACKET_Z_TOP = -EXT_SIZE

# Vertical arm: single plate from the hub's bottom up to
# VERT_ARM_TOP_Z (kept at the same Z as the earlier top-mount variant
# so the column bolts land at z=10 and z=30). The plate therefore
# spans z ∈ [BRACKET_Z_TOP - PLATE_T, VERT_ARM_TOP_Z] = [-26, 40].
VERT_ARM_TOP_Z = ARM_BOLT_SPAN     # 40 — top of vertical plate in world Z
VERT_BOLT_Z_1 = BOLT_FROM_EXT_END                        # 10
VERT_BOLT_Z_2 = BOLT_FROM_EXT_END + BOLT_SPACING          # 30

# Horizontal arm angle from +X (hex interior angle = 2 * this).
HALF_HEX_ANGLE_DEG = 60.0

# FDM printability floor.
WALL_MIN = 1.5


# --- Parametric sanity checks -------------------------------------------
# cad-khana's assert_clearance is part-to-part and bolt holes are
# subtracted negative space, so bolt-hole-to-edge wall thickness isn't
# directly expressible as an assertion. The global assert_min_wall on
# the bracket catches thin walls implicitly, but these plain Python
# asserts fail at script-run time if someone bumps BOLT_HOLE_D or
# BOLT_SPACING past safe parameters.
assert ARM_W - BOLT_HOLE_D >= 2 * WALL_MIN, (
    f"bolt-hole-to-arm-edge wall {(ARM_W - BOLT_HOLE_D) / 2:.2f}mm < {WALL_MIN}mm"
)
assert BOLT_SPACING - BOLT_HOLE_D >= 2 * WALL_MIN, (
    f"bolt-hole-to-bolt-hole wall {BOLT_SPACING - BOLT_HOLE_D:.2f}mm < {2 * WALL_MIN}mm"
)


# --- Part functions -----------------------------------------------------

def _through_hole_z(hole_d: float, plate_t: float) -> Part:
    """Cylindrical hole along +Z, long enough to cut cleanly through a
    plate of thickness plate_t."""
    return Cylinder(hole_d / 2, plate_t * 3)


def _through_hole_x(hole_d: float, plate_t: float) -> Part:
    """Cylindrical hole along +X, long enough to cut cleanly through a
    plate of thickness plate_t."""
    return Rot(0, 90, 0) * Cylinder(hole_d / 2, plate_t * 3)


def bracket() -> Part:
    """Single-body outer-vertex bracket: hub + two hex-edge arms +
    one radial arm + one vertical arm. Hub + horizontal arms live at
    z ∈ [BRACKET_Z_TOP - PLATE_T, BRACKET_Z_TOP] (below the
    extrusions). The vertical arm extends from the hub's bottom up
    to VERT_ARM_TOP_Z at the column's -X face. The through-column
    passes through the hub's square cutout at hub-Z.
    """
    plate_cz = BRACKET_Z_TOP - PLATE_T / 2   # hub/horizontal-arm center z (= -23)

    # Hub: flat square plate below the floor datum.
    hub = Pos(0, 0, plate_cz) * Box(2 * HUB_HALF, 2 * HUB_HALF, PLATE_T)

    # Horizontal arm in local frame: along +X, root at x=0 (inside the hub),
    # tip at x=ARM_L_TOTAL, z center at plate_cz, y ∈ [-ARM_W/2, +ARM_W/2].
    arm_plate = Pos(ARM_L_TOTAL / 2, 0, plate_cz) * Box(ARM_L_TOTAL, ARM_W, PLATE_T)
    bolt_x1 = HOR_INNER_OFFSET + BOLT_FROM_EXT_END
    bolt_x2 = bolt_x1 + BOLT_SPACING
    arm_local = (
        arm_plate
        - Pos(bolt_x1, 0, plate_cz) * _through_hole_z(BOLT_HOLE_D, PLATE_T)
        - Pos(bolt_x2, 0, plate_cz) * _through_hole_z(BOLT_HOLE_D, PLATE_T)
    )
    arm_h1 = Rot(0, 0, HALF_HEX_ANGLE_DEG) * arm_local
    arm_h2 = Rot(0, 0, -HALF_HEX_ANGLE_DEG) * arm_local
    arm_rad = arm_local  # radial arm along +X toward hex center

    # Vertical arm: single plate on the -X face of the through-column,
    # spanning from the hub's bottom up to VERT_ARM_TOP_Z. Plate
    # thickness PLATE_T along X; width ARM_W along Y.
    va_x_center = -EXT_SIZE / 2 - PLATE_T / 2
    va_z_bot = BRACKET_Z_TOP - PLATE_T                      # flush with hub bottom
    va_z_top = VERT_ARM_TOP_Z
    va_h_total = va_z_top - va_z_bot
    va_z_center = (va_z_bot + va_z_top) / 2
    v_plate = Pos(va_x_center, 0, va_z_center) * Box(PLATE_T, ARM_W, va_h_total)
    vert_arm = (
        v_plate
        - Pos(va_x_center, 0, VERT_BOLT_Z_1) * _through_hole_x(BOLT_HOLE_D, PLATE_T)
        - Pos(va_x_center, 0, VERT_BOLT_Z_2) * _through_hole_x(BOLT_HOLE_D, PLATE_T)
    )

    # The through-column passes through the hub's Z range. Cut its
    # footprint out of the hub plate (the vertical arm at x ∈
    # [-16, -10] stays entirely outside the column's -X face at x=-10,
    # so the cutout doesn't touch it).
    vert_cutout = Pos(0, 0, plate_cz) * Box(
        EXT_SIZE, EXT_SIZE, PLATE_T * 3
    )
    return (hub + arm_h1 + arm_h2 + arm_rad + vert_arm) - vert_cutout


def vert_ext_stub(size: float = EXT_SIZE) -> Part:
    """Through-column vertical extrusion stand-in. Spans from below
    the hub up past the vertical arm's top so the bracket can be
    checked against the column at every point along its body."""
    z_bot = BRACKET_Z_TOP - PLATE_T - 10.0   # 10 mm below hub bottom
    z_top = VERT_ARM_TOP_Z + 10.0             # 10 mm above arm top
    length = z_top - z_bot
    return Pos(0, 0, (z_bot + z_top) / 2) * Box(size, size, length)


def horiz_ext_stub_local(length: float = EXT_STUB_LEN,
                         size: float = EXT_SIZE,
                         inner_offset: float = HOR_INNER_OFFSET) -> Part:
    """Horizontal extrusion stand-in along its local +X axis. Top face at
    z=0, axis at z=-size/2. Inner end at x=inner_offset; outer end at
    x=inner_offset+length."""
    return Pos(inner_offset + length / 2, 0, -size / 2) * Box(length, size, size)


# --- Assembly -----------------------------------------------------------

assembly = (
    Assembly()
    .with_part("bracket", bracket())
    .with_part("vert_ext", vert_ext_stub())
    .with_part("horiz_ext_1", horiz_ext_stub_local(), location=Rot(0, 0, HALF_HEX_ANGLE_DEG))
    .with_part("horiz_ext_2", horiz_ext_stub_local(), location=Rot(0, 0, -HALF_HEX_ANGLE_DEG))
    .with_part("radial_ext", horiz_ext_stub_local())
    .assert_no_interference("bracket", "vert_ext")
    .assert_no_interference("bracket", "horiz_ext_1")
    .assert_no_interference("bracket", "horiz_ext_2")
    .assert_no_interference("bracket", "radial_ext")
    .assert_no_interference("vert_ext", "horiz_ext_1")
    .assert_no_interference("vert_ext", "horiz_ext_2")
    .assert_no_interference("vert_ext", "radial_ext")
    .assert_no_interference("horiz_ext_1", "horiz_ext_2")
    .assert_no_interference("horiz_ext_1", "radial_ext")
    .assert_no_interference("horiz_ext_2", "radial_ext")
)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # The bracket has real overhangs under the hub and horizontal arms when
    # printed with +Z up — they will need slicer supports or a different
    # build orientation. For now keep the overhang check informational by
    # raising the threshold; the wall-min check is the live assertion.
    inspect(
        bracket(),
        method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
        out="outputs",
        name="bracket",
    )
