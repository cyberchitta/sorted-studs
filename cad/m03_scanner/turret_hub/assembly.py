"""
M03 scanner — rev-3 turret hub (slice b of the 4-station turret).

Standalone printed pillow-block hub housing carrying a vertical Ø8
shaft on two 608ZZ ball bearings. Bolts to the slice (a) turret_frame
cross-rail (top T-slot) at the hub axis (world x = TURRET_HUB_X = 0,
z = TURRET_HUB_Z = -130). Drive coupling (motor + belt or worm) is
deferred until shaft layout is fixed; this slice delivers the
mechanical support, not the rotation source.

Members (4 total):
  * hub_housing       — printed rectangular block (HOUSING_X × HOUSING_Y
                        × HOUSING_Z) with one Ø22 vertical through-bore
                        (bearing seats top + bottom, shaft clearance in
                        the middle) and two Ø5.5 vertical M5 bolt
                        clearance holes spaced BOLT_SPACING_X apart along
                        the cross-rail axis. Bolts drop into the rail's
                        top T-slot (which runs along world ±X), so both
                        sit on z = TURRET_HUB_Z (the rail centerline).
  * hub_shaft         — stock Ø8 round, axis vertical at the hub,
                        extending from the housing bottom face up to
                        TURRET_SHAFT_TOP_Y (above the top bearing for
                        slice (c) arm-hub attachment).
  * hub_bearing_top   — 608ZZ-equivalent annular cylinder (Ø22 OD ×
                        Ø8 ID × 7 mm) flush with the housing top face.
  * hub_bearing_bottom — same, flush with the housing bottom face.

Coordinate frame = world frame (matches `../params.py`):
    origin = brick centroid on the IMAGE platform top face
    +X     = operator's right
    +Y     = up
    +Z     = away from operator (toward the back of the machine)

Housing bottom face sits on the cross-rail top face at world
y = TURRET_FRAME_FLOOR_Y + MEMBER/2 = -210; the hub axis runs vertical
through (world x = 0, z = TURRET_HUB_Z). Bearings press into the Ø22
bore (tangent contact in the model, which `assert_no_interference`
treats as zero overlap); axial retention is via shaft circlips
(E-clips), not modelled. Shaft is captured by the bearings + circlips.

Print orientation: bottom face on the build plate, +Y up. The Ø22
bearing bore is a clean vertical through-hole (no overhang); the two
M5 bolt clearance holes likewise. No horizontal bores, no ledges —
prints cleanly at the default 45° FDM overhang threshold.
"""

import sys
from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    TURRET_FRAME_CROSS_Z,
    TURRET_FRAME_FLOOR_Y,
    TURRET_HUB_X,
    TURRET_HUB_Z,
    TURRET_SHAFT_D,
    TURRET_SHAFT_TOP_Y,
    WALL_MIN,
)


# --- Bearing + housing geometry (private to this sub-assembly) ----------

# 608ZZ ball bearing dimensions (ID 8, OD 22, width 7).
TURRET_BEARING_OD = 22.0
TURRET_BEARING_T = 7.0

# Housing height above the cross-rail top face. 100 mm gives an 86 mm
# clear span between the top and bottom bearings (each flush with its
# adjacent housing face) — enough moment support for the unbalanced-
# load case (one platform carrying a brick, three empty) without
# dominating the print volume.
TURRET_HUB_HOUSING_HEIGHT = 100.0

# Housing top face Y in world. Cross-rail top face at y = -210
# (= floor + MEMBER/2) plus housing height = -110.
TURRET_HUB_HOUSING_TOP_Y = (
    TURRET_FRAME_FLOOR_Y + 20.0 / 2 + TURRET_HUB_HOUSING_HEIGHT
)                                                                # = -110


# --- Cross-rail interface ----------------------------------------------

MEMBER = 20.0                              # 2020 cross-rail cross-section
RAIL_TOP_Y = TURRET_FRAME_FLOOR_Y + MEMBER / 2   # = -210


# --- Housing dimensions ------------------------------------------------

# X = along cross-rail (so the two T-slot bolts spread along the rail).
# Z = across rail. Both chosen wide enough to leave ≥ WALL_MIN around
# the central Ø22 bore and around each Ø5.5 bolt clearance hole.
HOUSING_X = 60.0
HOUSING_Y = TURRET_HUB_HOUSING_HEIGHT      # = 100
HOUSING_Z = 50.0

# Central bore diameter — bearing OD press fit. Modeled tangent
# (zero radial gap); real assembly relies on FDM tolerance + light
# press to seat the bearing OD. If numerical noise ever crosses the
# 0.001 mm³ assert_no_interference threshold, model the bore at
# BORE_D + ~0.2 mm to give a clean clearance.
BORE_D = TURRET_BEARING_OD


# --- Bolt pattern -------------------------------------------------------

# Two M5 bolts drop through the housing in -Y into the cross-rail's
# top T-slot. M5 clearance hole = 5.5 mm. The T-slot runs along world
# ±X, so both bolts must sit on the rail's centerline in Z (= z = 0
# in housing-local). Spacing must clear the central Ø22 bore on the
# inside and the housing X-edge on the outside — both ≥ WALL_MIN.
BOLT_HOLE_D = 5.5
BOLT_SPACING_X = 35.0
_BOLT_HALF_X = BOLT_SPACING_X / 2


# --- Shaft + bearing dimensions ----------------------------------------

SHAFT_D = TURRET_SHAFT_D
_SHAFT_LEN = TURRET_SHAFT_TOP_Y - RAIL_TOP_Y      # = 200

BEARING_OD = TURRET_BEARING_OD
BEARING_ID = SHAFT_D
BEARING_T = TURRET_BEARING_T


# --- Sanity asserts ----------------------------------------------------

# Bolt clearance hole leaves a wall to the central bore.
_bolt_to_bore = _BOLT_HALF_X - BORE_D / 2 - BOLT_HOLE_D / 2
assert _bolt_to_bore >= WALL_MIN, (
    f"bolt-to-bore wall {_bolt_to_bore:.2f} mm < WALL_MIN ({WALL_MIN}); "
    f"either widen BOLT_SPACING_X or shrink BORE_D"
)

# Bolt clearance hole leaves a wall to the housing X-end face.
_bolt_to_edge = HOUSING_X / 2 - _BOLT_HALF_X - BOLT_HOLE_D / 2
assert _bolt_to_edge >= WALL_MIN, (
    f"bolt-to-X-edge wall {_bolt_to_edge:.2f} mm < WALL_MIN ({WALL_MIN}); "
    f"either widen HOUSING_X or shrink BOLT_SPACING_X"
)

# Bore wall to the housing X / Z edges. X is the narrower direction.
_bore_wall_x = HOUSING_X / 2 - BORE_D / 2
assert _bore_wall_x >= WALL_MIN, (
    f"bore-to-X-edge wall {_bore_wall_x:.2f} mm < WALL_MIN ({WALL_MIN}); "
    f"raise HOUSING_X"
)
_bore_wall_z = HOUSING_Z / 2 - BORE_D / 2
assert _bore_wall_z >= WALL_MIN, (
    f"bore-to-Z-edge wall {_bore_wall_z:.2f} mm < WALL_MIN ({WALL_MIN}); "
    f"raise HOUSING_Z"
)

# Re-derive the world housing top Y from rail top + housing height
# and check it agrees with the params constant — guards against the
# two drifting apart silently.
_HOUSING_TOP_Y_DERIVED = RAIL_TOP_Y + HOUSING_Y
assert abs(TURRET_HUB_HOUSING_TOP_Y - _HOUSING_TOP_Y_DERIVED) < 1e-6, (
    f"TURRET_HUB_HOUSING_TOP_Y ({TURRET_HUB_HOUSING_TOP_Y}) disagrees with "
    f"rail-top + HOUSING_Y ({_HOUSING_TOP_Y_DERIVED}); update one to match"
)

# The slice (a) cross-rail must run through the hub axis Z so the
# housing centers on it. (Both should be -130 by construction.)
assert abs(TURRET_FRAME_CROSS_Z - TURRET_HUB_Z) < 1e-6, (
    f"cross-rail Z ({TURRET_FRAME_CROSS_Z}) ≠ hub Z ({TURRET_HUB_Z}); "
    f"hub housing assumes the rail runs through the hub axis"
)

# Shaft must extend above the housing top face for slice (c) arm
# attachment.
assert TURRET_SHAFT_TOP_Y > _HOUSING_TOP_Y_DERIVED, (
    f"shaft top ({TURRET_SHAFT_TOP_Y}) must sit above housing top "
    f"({_HOUSING_TOP_Y_DERIVED}) for slice (c) arm-hub attachment"
)


# --- Part functions ----------------------------------------------------

def housing(
    housing_x: float = HOUSING_X,
    housing_y: float = HOUSING_Y,
    housing_z: float = HOUSING_Z,
    bore_d: float = BORE_D,
    bolt_half_x: float = _BOLT_HALF_X,
    bolt_hole_d: float = BOLT_HOLE_D,
):
    """Printed pillow-block hub housing: rectangular body with one
    vertical Ø22 through-bore (bearing seats top + bottom, shaft
    clearance in the middle) and two vertical M5 bolt clearance holes
    that drop into the cross-rail's top T-slot."""
    body = Pos(0, housing_y / 2, 0) * Box(housing_x, housing_y, housing_z)

    # Central vertical through-bore (axis along ±Y).
    body -= (
        Pos(0, housing_y / 2, 0)
        * Rot(90, 0, 0)
        * Cylinder(bore_d / 2, housing_y * 3)
    )

    # Two vertical M5 clearance holes (axis along ±Y).
    for sx in (-1, 1):
        body -= (
            Pos(sx * bolt_half_x, housing_y / 2, 0)
            * Rot(90, 0, 0)
            * Cylinder(bolt_hole_d / 2, housing_y * 3)
        )

    return body


def shaft(shaft_d: float = SHAFT_D, length: float = _SHAFT_LEN):
    """Stock Ø8 round shaft, vertical at hub axis. Origin at the
    placement (caller positions); extends from y = 0 up to y = length
    in part-local coords."""
    return (
        Pos(0, length / 2, 0)
        * Rot(90, 0, 0)
        * Cylinder(shaft_d / 2, length)
    )


def bearing(
    bearing_od: float = BEARING_OD,
    bearing_id: float = BEARING_ID,
    bearing_t: float = BEARING_T,
):
    """608ZZ-equivalent annular cylinder. Axis along ±Y, centered on
    the placement origin. Stock part — modeled as an annulus to verify
    shaft-passes-through and OD-fits-bore."""
    annulus = Rot(90, 0, 0) * Cylinder(bearing_od / 2, bearing_t)
    annulus -= Rot(90, 0, 0) * Cylinder(bearing_id / 2, bearing_t * 3)
    return annulus


# --- Bearing placements (housing-local Y) ------------------------------

# Top bearing: top face flush with housing top face (y = HOUSING_Y).
# Bottom bearing: bottom face flush with housing bottom face (y = 0).
# Cylinder is centered at its placement origin → place each at the
# face Y minus half-thickness.
_TOP_BEARING_Y = HOUSING_Y - BEARING_T / 2
_BOTTOM_BEARING_Y = BEARING_T / 2


# --- Assembly builder --------------------------------------------------

HUB_PART_NAMES = (
    "hub_housing",
    "hub_shaft",
    "hub_bearing_top",
    "hub_bearing_bottom",
)


def add_turret_hub(a: Assembly) -> Assembly:
    """Add the four turret-hub parts (housing + shaft + 2 bearings) in
    world coordinates and return the updated assembly. Exposed so the
    top-level scanner assembly composes via the public surface instead
    of reaching into module-private state."""
    base = Pos(TURRET_HUB_X, RAIL_TOP_Y, TURRET_HUB_Z)
    a = a.add("hub_housing", housing(), location=base)
    a = a.add("hub_shaft", shaft(), location=base)
    a = a.add(
        "hub_bearing_top",
        bearing(),
        location=base * Pos(0, _TOP_BEARING_Y, 0),
    )
    a = a.add(
        "hub_bearing_bottom",
        bearing(),
        location=base * Pos(0, _BOTTOM_BEARING_Y, 0),
    )
    return a


# --- Standalone assembly -----------------------------------------------

assembly = add_turret_hub(Assembly())

# All pairwise no-interference. Bearings press-fit into the housing
# bore (tangent contact, passes assert_no_interference at zero
# overlap) and onto the shaft (same).
for _i, _a in enumerate(HUB_PART_NAMES):
    for _b in HUB_PART_NAMES[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: bottom face on the build plate (+Y up). FDM
    # default up_axis is +Z, so override. All bores are vertical
    # cylinders → no overhangs to lift the threshold for; default 45°
    # is enough.
    inspect(
        housing(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=45.0
        ),
        out="outputs",
        name="hub_housing",
    )
