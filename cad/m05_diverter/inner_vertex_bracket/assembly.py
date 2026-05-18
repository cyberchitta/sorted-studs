"""
M05 hex-tower — inner-hex vertex bracket.

Star-shaped printed bracket that joins four aluminum extrusions at
every inner-hex vertex: two inner-hex ring edges (top face at z=0),
one **outboard** 2020 radial extrusion running from the vertex out
to the outer-hex vertex, and one **inboard** 2020 radial extrusion
running from the vertex in toward the spider hub (both rails sit
with top face at z=0). A vertical 2020 through-column passes
through the bracket at the vertex axis (see
`../inner_columns_handoff.md`); the hub plate carries a square
cutout that clears the column footprint so the column can pass
through the bracket's Z range. The clamp jaws / vertical arm that
will actually grip the column are slice (d) — deferred — so this
bracket geometrically tolerates the column but does not yet
restrain it.

Splitting the radial at the inner vertex was forced by the through-
column: a single radial spanning [RAIL_INNER_R, RAIL_OUTER_R] would
otherwise pass straight through the inner column. Both radial arms
clamp the rail from below at z ∈ [-26, -20]; the inboard arm is
shorter (single bolt) because the spider saddle grips the inboard
rail's other end at r ∈ [RAIL_INNER_R, RAIL_INNER_R +
SADDLE_GRIP_LEN] and a 2-bolt inboard arm would crowd that grip
zone.

This mirrors `outer_vertex_bracket` structurally, but:
  - no vertical clamp arm yet (slice d);
  - two radial arms (inboard + outboard) rather than one.

**Bottom-mounted.** The bracket hangs off the *bottom* T-slot of the
extrusions, body at z ∈ [-EXT_SIZE - PLATE_T, -EXT_SIZE] = [-26, -20].
Bolts fire upward (+Z) through the plate into the extrusion's bottom
T-slot. The reason: a top-mounted plate at z ∈ [0, PLATE_T] overlaps
the basket Z-range [0, 80], so the bracket's horizontal arms would
clip the basket sector and force the basket to shrink angularly.
Flipping below leaves the whole frame top surface flush at z=0 so
the basket can rest anywhere on it. The saddle grip zone of the
bearing spider sits at r ≈ [90, 110], well inside the inner-hex
vertex (r = R_HEX_INNER · 2/√3), so nothing collides with the
below-plate at the vertex.

Coordinate frame:
    +X       : bisector of the two hex-edge arms, pointing toward
               the hex center (INWARD). Frame places the bracket at
               inner vertex V_k with world rotation v_angle_k + 180°
               so local +X maps to inward at every vertex — same
               convention as the outer bracket.
    -X       : hex exterior (outward); outboard radial arm runs
               along -X
    +Z       : up
    z=0      : floor datum. Extrusion top faces sit here; bracket
               plate body at z ∈ [-EXT_SIZE - PLATE_T, -EXT_SIZE]
               (below the extrusions).
    arm_h1   : hex-edge arm at +60° from +X (along one inner-hex edge)
    arm_h2   : hex-edge arm at -60° from +X (along the other inner-hex edge)
    arm_rad_out : outboard radial arm along -X carrying the rail
                  segment to the outer-hex vertex (2 bolts)
    arm_rad_in  : inboard radial arm along +X carrying the rail
                  segment to the spider hub (1 bolt — see saddle
                  clearance note above)

Each arm is a flat plate with M5 clearance hole(s) that align to a
T-slot on the bottom face of its extrusion.
"""

from build123d import Box, Cylinder, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM


# --- Parameters ---------------------------------------------------------

# Extrusion — 2020 (matches the frame + outer_vertex_bracket).
EXT_SIZE = 20.0
EXT_STUB_LEN = 60.0

# M5 hardware (standard for 2020 T-slots).
BOLT_HOLE_D = 5.5
BOLT_FROM_EXT_END = 10.0
BOLT_SPACING = 20.0

# Bracket body. Plate hangs below each extrusion's bottom face at
# z=-EXT_SIZE; body at z ∈ [BRACKET_Z_TOP - PLATE_T, BRACKET_Z_TOP]
# = [-26, -20]. Basket Z-range [0, 80] no longer overlaps the plate,
# so the hub and arms can extend freely in xy without catching the
# basket. HUB_HALF = 14 matches the outer bracket and leaves a 4 mm
# wall thickness around the 20×20 vertical-column cutout.
PLATE_T = 6.0
HUB_HALF = 14.0
BRACKET_Z_TOP = -EXT_SIZE     # plate top face flush with extrusion bottom
ARM_W = EXT_SIZE
ARM_BOLT_SPAN = BOLT_FROM_EXT_END + BOLT_SPACING + BOLT_FROM_EXT_END  # 40

# Distance from the inner-hex vertex (bracket origin) to the inner end
# of each extrusion along its own axis. Matches the outer bracket so
# the frame can use a single HOR_INNER_OFFSET everywhere.
HOR_INNER_OFFSET = 22.0

# Horizontal arm total length (root at hub center, tip past the outer bolt).
ARM_L_TOTAL = HOR_INNER_OFFSET + ARM_BOLT_SPAN  # 62

# Inboard-radial arm — single bolt to clear the spider saddle that
# grips the inboard rail's inner end at r ∈ [90, 110]. With the
# vertex at r ≈ 161.66 and BRACKET_HOR_OFFSET = 22, a single bolt
# at +10 mm beyond the rail's outer end sits at r ≈ 129.66, ~20 mm
# clear of the saddle.
ARM_IN_BOLT_SPAN = 2 * BOLT_FROM_EXT_END                 # 20 — single bolt centred
ARM_IN_L_TOTAL = HOR_INNER_OFFSET + ARM_IN_BOLT_SPAN     # 42

# Hex interior half-angle.
HALF_HEX_ANGLE_DEG = 60.0

# FDM printability floor.
WALL_MIN = 1.5


# --- Parametric sanity checks -------------------------------------------

assert ARM_W - BOLT_HOLE_D >= 2 * WALL_MIN, (
    f"bolt-hole-to-arm-edge wall {(ARM_W - BOLT_HOLE_D) / 2:.2f}mm < {WALL_MIN}mm"
)
assert BOLT_SPACING - BOLT_HOLE_D >= 2 * WALL_MIN, (
    f"bolt-hole-to-bolt-hole wall {BOLT_SPACING - BOLT_HOLE_D:.2f}mm < {2 * WALL_MIN}mm"
)


# --- Part helpers -------------------------------------------------------

def _through_hole_z(hole_d: float, plate_t: float) -> Part:
    """Cylindrical hole along +Z, long enough to cut cleanly through a
    plate of thickness plate_t."""
    return Cylinder(hole_d / 2, plate_t * 3)


# --- Part functions -----------------------------------------------------

def bracket() -> Part:
    """Single-body inner-vertex bracket: square donut hub (cutout
    clears the inner-hex through-column) + two hex-edge arms (at
    ±60° from local +X) + outboard radial arm (along -X, 2 bolts)
    + inboard radial arm (along +X, 1 bolt). All plates at PLATE_T
    thickness, hanging below the extrusion plane at
    z ∈ [BRACKET_Z_TOP - PLATE_T, BRACKET_Z_TOP] = [-26, -20]. The
    through-column passes through the hub's square cutout at hub-Z;
    the clamp arms that will actually grip the column are slice
    (d) — not present yet.
    """
    plate_cz = BRACKET_Z_TOP - PLATE_T / 2  # plate center z

    # Hub: flat square plate below the floor datum.
    hub = Pos(0, 0, plate_cz) * Box(2 * HUB_HALF, 2 * HUB_HALF, PLATE_T)

    # Canonical 2-bolt arm along local +X. Bolt holes start
    # BOLT_FROM_EXT_END inside the ext's inner end (itself at
    # x=HOR_INNER_OFFSET).
    arm_plate = Pos(ARM_L_TOTAL / 2, 0, plate_cz) * Box(ARM_L_TOTAL, ARM_W, PLATE_T)
    bolt_x1 = HOR_INNER_OFFSET + BOLT_FROM_EXT_END
    bolt_x2 = bolt_x1 + BOLT_SPACING
    arm_2bolt = (
        arm_plate
        - Pos(bolt_x1, 0, plate_cz) * _through_hole_z(BOLT_HOLE_D, PLATE_T)
        - Pos(bolt_x2, 0, plate_cz) * _through_hole_z(BOLT_HOLE_D, PLATE_T)
    )
    arm_h1 = Rot(0, 0, HALF_HEX_ANGLE_DEG) * arm_2bolt
    arm_h2 = Rot(0, 0, -HALF_HEX_ANGLE_DEG) * arm_2bolt
    arm_rad_out = Rot(0, 0, 180.0) * arm_2bolt  # outward toward outer hex

    # Inboard radial arm: single bolt at the midpoint of the short
    # plate; spans HOR_INNER_OFFSET + 2·BOLT_FROM_EXT_END so the
    # inboard rail's outer end is clamped without reaching into the
    # spider saddle's r ∈ [90, 110] grip zone.
    arm_in_plate = Pos(ARM_IN_L_TOTAL / 2, 0, plate_cz) * Box(
        ARM_IN_L_TOTAL, ARM_W, PLATE_T,
    )
    bolt_in_x = HOR_INNER_OFFSET + BOLT_FROM_EXT_END
    arm_rad_in = (
        arm_in_plate
        - Pos(bolt_in_x, 0, plate_cz) * _through_hole_z(BOLT_HOLE_D, PLATE_T)
    )

    # The through-column passes through the hub's Z range. Cut its
    # footprint out of the hub plate (4 mm wall thickness remains on
    # each side; hex-edge and radial arms connect to those walls).
    vert_cutout = Pos(0, 0, plate_cz) * Box(
        EXT_SIZE, EXT_SIZE, PLATE_T * 3
    )
    return (hub + arm_h1 + arm_h2 + arm_rad_out + arm_rad_in) - vert_cutout


def vert_ext_stub(size: float = EXT_SIZE) -> Part:
    """Through-column vertical extrusion stand-in. Spans well past
    the bracket's Z range so the column-vs-bracket interference
    check sees a full vertical pass-through."""
    z_bot = BRACKET_Z_TOP - PLATE_T - 10.0   # 10 mm below hub bottom
    z_top = 30.0                              # well above the floor datum
    length = z_top - z_bot
    return Pos(0, 0, (z_bot + z_top) / 2) * Box(size, size, length)


def horiz_ext_stub_local(length: float = EXT_STUB_LEN,
                         size: float = EXT_SIZE,
                         inner_offset: float = HOR_INNER_OFFSET) -> Part:
    """Horizontal extrusion stand-in along its local +X axis. Top face
    at z=0, axis at z=-size/2. Inner end at x=inner_offset; outer end
    at x=inner_offset+length."""
    return Pos(inner_offset + length / 2, 0, -size / 2) * Box(length, size, size)


# --- Assembly -----------------------------------------------------------

# Two hex-edge stubs at ±60° (same as outer bracket), an outboard
# radial stub running along -X (placed by rotating the +X stub by
# 180°), an inboard radial stub running along +X, and the inner-hex
# through-column passing vertically through the hub's cutout.
assembly = (
    Assembly()
    .add("bracket", bracket())
    .add("vert_ext", vert_ext_stub())
    .add("horiz_ext_1", horiz_ext_stub_local(), location=Rot(0, 0, HALF_HEX_ANGLE_DEG))
    .add("horiz_ext_2", horiz_ext_stub_local(), location=Rot(0, 0, -HALF_HEX_ANGLE_DEG))
    .add("radial_ext_out", horiz_ext_stub_local(), location=Rot(0, 0, 180.0))
    .add("radial_ext_in", horiz_ext_stub_local())
    .assert_no_interference("bracket", "vert_ext")
    .assert_no_interference("bracket", "horiz_ext_1")
    .assert_no_interference("bracket", "horiz_ext_2")
    .assert_no_interference("bracket", "radial_ext_out")
    .assert_no_interference("bracket", "radial_ext_in")
    .assert_no_interference("vert_ext", "horiz_ext_1")
    .assert_no_interference("vert_ext", "horiz_ext_2")
    .assert_no_interference("vert_ext", "radial_ext_out")
    .assert_no_interference("vert_ext", "radial_ext_in")
    .assert_no_interference("horiz_ext_1", "horiz_ext_2")
    .assert_no_interference("horiz_ext_1", "radial_ext_out")
    .assert_no_interference("horiz_ext_1", "radial_ext_in")
    .assert_no_interference("horiz_ext_2", "radial_ext_out")
    .assert_no_interference("horiz_ext_2", "radial_ext_in")
    .assert_no_interference("radial_ext_out", "radial_ext_in")
)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Real overhangs under the hub and arms when printed with +Z up;
    # keep overhang check informational (matches outer_vertex_bracket).
    inspect(
        bracket(),
        method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0),
        out="outputs",
        name="bracket",
    )
