"""
M05 hex-tower — inner-hex vertex bracket.

Star-shaped printed bracket that joins three aluminum extrusions at
every inner-hex vertex: two inner-hex ring edges (top face at z=0)
plus one **radial** 2020 extrusion running outward from the vertex
toward the outer hex (also top face at z=0). No vertical column
passes through this bracket (inner-hex verticals are deferred —
see `hex-cascade.md`), so the hub is a solid plate with no cutout.

This mirrors `outer_vertex_bracket` structurally, but:
  - no vertical arm / no square cutout in the hub;
  - radial arm points **outward** (local -X), because the radial
    rail extends from the inner vertex away from hex center out
    to the outer vertex.

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
    -X       : hex exterior (outward); radial arm runs along -X
    +Z       : up
    z=0      : floor datum. Extrusion top faces sit here; bracket
               plate body at z ∈ [-EXT_SIZE - PLATE_T, -EXT_SIZE]
               (below the extrusions).
    arm_h1   : hex-edge arm at +60° from +X (along one inner-hex edge)
    arm_h2   : hex-edge arm at -60° from +X (along the other inner-hex edge)
    arm_rad  : radial arm along -X (outward) carrying the radial rail

Each arm is a flat plate with two M5 clearance holes that align to a
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
# basket — HUB_HALF = 12 is plenty of margin at any R_HEX_INNER.
PLATE_T = 6.0
HUB_HALF = 12.0
BRACKET_Z_TOP = -EXT_SIZE     # plate top face flush with extrusion bottom
ARM_W = EXT_SIZE
ARM_BOLT_SPAN = BOLT_FROM_EXT_END + BOLT_SPACING + BOLT_FROM_EXT_END  # 40

# Distance from the inner-hex vertex (bracket origin) to the inner end
# of each extrusion along its own axis. Matches the outer bracket so
# the frame can use a single HOR_INNER_OFFSET everywhere.
HOR_INNER_OFFSET = 22.0

# Horizontal arm total length (root at hub center, tip past the outer bolt).
ARM_L_TOTAL = HOR_INNER_OFFSET + ARM_BOLT_SPAN  # 62

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
    """Single-body inner-vertex bracket: solid square hub + two
    hex-edge arms (at ±60° from local +X) + one radial arm (at 180°
    from local +X, i.e., along -X). All plates at PLATE_T thickness,
    hanging below the extrusion plane at
    z ∈ [BRACKET_Z_TOP - PLATE_T, BRACKET_Z_TOP] = [-26, -20].
    """
    plate_cz = BRACKET_Z_TOP - PLATE_T / 2  # plate center z

    # Hub: flat square plate below the floor datum. No cutout (no vertical).
    hub = Pos(0, 0, plate_cz) * Box(2 * HUB_HALF, 2 * HUB_HALF, PLATE_T)

    # Canonical arm along local +X with two bolt clearance holes
    # aligned to the extrusion's bottom T-slot (starting
    # BOLT_FROM_EXT_END inside the ext's inner end, which itself
    # sits at x=HOR_INNER_OFFSET).
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
    arm_rad = Rot(0, 0, 180.0) * arm_local  # outward toward outer hex

    return hub + arm_h1 + arm_h2 + arm_rad


def horiz_ext_stub_local(length: float = EXT_STUB_LEN,
                         size: float = EXT_SIZE,
                         inner_offset: float = HOR_INNER_OFFSET) -> Part:
    """Horizontal extrusion stand-in along its local +X axis. Top face
    at z=0, axis at z=-size/2. Inner end at x=inner_offset; outer end
    at x=inner_offset+length."""
    return Pos(inner_offset + length / 2, 0, -size / 2) * Box(length, size, size)


# --- Assembly -----------------------------------------------------------

# Two hex-edge stubs at ±60° (same as outer bracket) plus a radial
# stub running outward along -X (placed by rotating the +X stub by 180°).
assembly = (
    Assembly()
    .add("bracket", bracket())
    .add("horiz_ext_1", horiz_ext_stub_local(), location=Rot(0, 0, HALF_HEX_ANGLE_DEG))
    .add("horiz_ext_2", horiz_ext_stub_local(), location=Rot(0, 0, -HALF_HEX_ANGLE_DEG))
    .add("radial_ext", horiz_ext_stub_local(), location=Rot(0, 0, 180.0))
    .assert_no_interference("bracket", "horiz_ext_1")
    .assert_no_interference("bracket", "horiz_ext_2")
    .assert_no_interference("bracket", "radial_ext")
    .assert_no_interference("horiz_ext_1", "horiz_ext_2")
    .assert_no_interference("horiz_ext_1", "radial_ext")
    .assert_no_interference("horiz_ext_2", "radial_ext")
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
