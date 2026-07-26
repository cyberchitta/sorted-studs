"""
M03 scanner — rev-3 turret hub (slice b of the 4-station turret).

Standalone printed pillow-block hub housing carrying a vertical Ø8
shaft on two 608ZZ ball bearings. Bolts to the slice (a) turret_frame
cross-rail (top T-slot) at the hub axis (world x = TURRET_HUB_X = 0,
y = TURRET_HUB_Y = -130 after world placement). Drive coupling
(motor + belt or worm) is deferred until shaft layout is fixed; this
slice delivers the mechanical support, not the rotation source.

Members (4 total):
  * hub_housing       — printed rectangular block (HOUSING_X × HOUSING_Y
                        × HOUSING_Z) with one Ø22 vertical through-bore
                        (bearing seats top + bottom, shaft clearance in
                        the middle) and two Ø5.5 vertical M5 bolt
                        clearance holes spaced BOLT_SPACING_X apart along
                        the cross-rail axis. Bolts drop into the rail's
                        top T-slot (which runs along world ±X), so both
                        sit on y = TURRET_HUB_Y (the rail centerline).
  * hub_shaft         — stock Ø8 round, axis vertical at the hub,
                        extending from the housing bottom face up to
                        TURRET_SHAFT_TOP_Z (above the top bearing for
                        slice (c) arm-hub attachment).
  * hub_bearing_top   — 608ZZ-equivalent annular cylinder (Ø22 OD ×
                        Ø8 ID × 7 mm) flush with the housing top face.
  * hub_bearing_bottom — same, flush with the housing bottom face.

Coordinate frame = turret-local frame (matches `../params.py`):
    origin = hub axis at the cross-rail's top face Z plane
    +X     = operator's right            (parallel to world +X)
    +Y     = away from operator          (parallel to world +Y)
    +Z     = up                          (parallel to world +Z)

Housing bottom face sits on the cross-rail top face at
z = TURRET_FRAME_FLOOR_Z + MEMBER/2 = -170; the hub axis runs vertical
through turret-local (0, 0, *) by construction. Bearings press into
the Ø22 bore (tangent contact in the model, which
`assert_no_interference` treats as zero overlap); axial retention is
via shaft circlips (E-clips), not modelled. Shaft is captured by the
bearings + circlips.

Print orientation: bottom face on the build plate, +Z up (build123d
default). The Ø22 bearing bore is a clean vertical through-hole (no
overhang); the two M5 bolt clearance holes likewise. No horizontal
bores, no ledges — prints cleanly at the default 45° FDM overhang
threshold.
"""


from build123d import Box, Cylinder, Pos

from cad_khana.mechanism.assembly import Assembly


from ..params import (
    TURRET_FRAME_CROSS_Y,
    TURRET_FRAME_FLOOR_Z,
    TURRET_HUB_X,
    TURRET_HUB_Y,
    TURRET_SHAFT_D,
    TURRET_SHAFT_TOP_Z,
    WALL_MIN,
)
from ..materials import material_of


# --- Bearing + housing geometry (private to this sub-assembly) ----------

# 608ZZ ball bearing dimensions (ID 8, OD 22, width 7).
TURRET_BEARING_OD = 22.0
TURRET_BEARING_T = 7.0

# Housing height above the cross-rail top face. 60 mm gives a 46 mm
# clear span between the top and bottom bearings (each flush with its
# adjacent housing face) — comfortable for the static load (4 light
# platforms, slow indexing). Slice c4 (2026-05-05) shrank this from
# 100 → 60 to drop the floor 40 mm; slice c5 (2026-05-05) left this
# unchanged but raised the floor a further 30 mm so the housing top
# rose with it (-110 → -80) to absorb the housing-top → pulley-bottom
# gap. Slice c7 (2026-05-05) raised the floor another 20 mm; housing
# top moved -80 → -60 in lockstep.
TURRET_HUB_HOUSING_HEIGHT = 60.0

# Housing top face Z in world. Cross-rail top face at z = -120
# (= floor + MEMBER/2 after c7) plus housing height = -60.
TURRET_HUB_HOUSING_TOP_Z = (
    TURRET_FRAME_FLOOR_Z + 20.0 / 2 + TURRET_HUB_HOUSING_HEIGHT
)                                                                # = -60


# --- Cross-rail interface ----------------------------------------------

MEMBER = 20.0                              # 2020 cross-rail cross-section
RAIL_TOP_Z = TURRET_FRAME_FLOOR_Z + MEMBER / 2   # = -120 after c7


# --- Housing dimensions ------------------------------------------------

# X = along cross-rail (so the two T-slot bolts spread along the rail).
# Y = across rail. Both chosen wide enough to leave ≥ WALL_MIN around
# the central Ø22 bore and around each Ø5.5 bolt clearance hole.
HOUSING_X = 60.0
HOUSING_Y = 50.0
HOUSING_Z = TURRET_HUB_HOUSING_HEIGHT       # = 60 (vertical, after c4)

# Central bore diameter — bearing OD press fit. Modeled tangent
# (zero radial gap); real assembly relies on FDM tolerance + light
# press to seat the bearing OD. If numerical noise ever crosses the
# 0.001 mm³ assert_no_interference threshold, model the bore at
# BORE_D + ~0.2 mm to give a clean clearance.
BORE_D = TURRET_BEARING_OD


# --- Bolt pattern -------------------------------------------------------

# Two M5 bolts drop through the housing in -Z into the cross-rail's
# top T-slot. M5 clearance hole = 5.5 mm. The T-slot runs along world
# ±X, so both bolts must sit on the rail's centerline in Y (= y = 0
# in housing-local). Spacing must clear the central Ø22 bore on the
# inside and the housing X-edge on the outside — both ≥ WALL_MIN.
BOLT_HOLE_D = 5.5
BOLT_SPACING_X = 35.0
_BOLT_HALF_X = BOLT_SPACING_X / 2


# --- Shaft + bearing dimensions ----------------------------------------

SHAFT_D = TURRET_SHAFT_D
_SHAFT_LEN = TURRET_SHAFT_TOP_Z - RAIL_TOP_Z      # = 110 after c7

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

# Bore wall to the housing X / Y edges. X is the narrower direction.
_bore_wall_x = HOUSING_X / 2 - BORE_D / 2
assert _bore_wall_x >= WALL_MIN, (
    f"bore-to-X-edge wall {_bore_wall_x:.2f} mm < WALL_MIN ({WALL_MIN}); "
    f"raise HOUSING_X"
)
_bore_wall_y = HOUSING_Y / 2 - BORE_D / 2
assert _bore_wall_y >= WALL_MIN, (
    f"bore-to-Y-edge wall {_bore_wall_y:.2f} mm < WALL_MIN ({WALL_MIN}); "
    f"raise HOUSING_Y"
)

# Re-derive the world housing top Z from rail top + housing height
# and check it agrees with the params constant — guards against the
# two drifting apart silently.
_HOUSING_TOP_Z_DERIVED = RAIL_TOP_Z + HOUSING_Z
assert abs(TURRET_HUB_HOUSING_TOP_Z - _HOUSING_TOP_Z_DERIVED) < 1e-6, (
    f"TURRET_HUB_HOUSING_TOP_Z ({TURRET_HUB_HOUSING_TOP_Z}) disagrees with "
    f"rail-top + HOUSING_Z ({_HOUSING_TOP_Z_DERIVED}); update one to match"
)

# The slice (a) cross-rail must run through the hub axis Y so the
# housing centers on it. (Both should be 0 by construction.)
assert abs(TURRET_FRAME_CROSS_Y - TURRET_HUB_Y) < 1e-6, (
    f"cross-rail Y ({TURRET_FRAME_CROSS_Y}) ≠ hub Y ({TURRET_HUB_Y}); "
    f"hub housing assumes the rail runs through the hub axis"
)

# Shaft must extend above the housing top face for slice (c) arm
# attachment.
assert TURRET_SHAFT_TOP_Z > _HOUSING_TOP_Z_DERIVED, (
    f"shaft top ({TURRET_SHAFT_TOP_Z}) must sit above housing top "
    f"({_HOUSING_TOP_Z_DERIVED}) for slice (c) arm-hub attachment"
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
    body = Pos(0, 0, housing_z / 2) * Box(housing_x, housing_y, housing_z)

    # Central vertical through-bore (axis along ±Z = build123d default).
    body -= (
        Pos(0, 0, housing_z / 2)
        * Cylinder(bore_d / 2, housing_z * 3)
    )

    # Two vertical M5 clearance holes (axis along ±Z).
    for sx in (-1, 1):
        body -= (
            Pos(sx * bolt_half_x, 0, housing_z / 2)
            * Cylinder(bolt_hole_d / 2, housing_z * 3)
        )

    return body


def shaft(shaft_d: float = SHAFT_D, length: float = _SHAFT_LEN):
    """Stock Ø8 round shaft, vertical at hub axis (canonical local
    frame — identity intrinsic Location). The placement site lifts
    the centroid so the bottom face lands at the caller's reference
    plane; required for ``export_animated_glb`` to track the shaft's
    rigid-body trajectory inside the rotor sub-assembly without
    losing the part-internal offset."""
    return Cylinder(shaft_d / 2, length)


def bearing(
    bearing_od: float = BEARING_OD,
    bearing_id: float = BEARING_ID,
    bearing_t: float = BEARING_T,
):
    """608ZZ-equivalent annular cylinder. Axis along ±Z, centered on
    the placement origin. Stock part — modeled as an annulus to verify
    shaft-passes-through and OD-fits-bore."""
    annulus = Cylinder(bearing_od / 2, bearing_t)
    annulus -= Cylinder(bearing_id / 2, bearing_t * 3)
    return annulus


# --- Bearing placements (housing-local Z) ------------------------------

# Top bearing: top face flush with housing top face (z = HOUSING_Z).
# Bottom bearing: bottom face flush with housing bottom face (z = 0).
# Cylinder is centered at its placement origin → place each at the
# face Z minus half-thickness.
_TOP_BEARING_Z = HOUSING_Z - BEARING_T / 2
_BOTTOM_BEARING_Z = BEARING_T / 2


# --- Assembly builder --------------------------------------------------

HUB_STATIC_PART_NAMES = (
    "hub_housing",
    "hub_bearing_top",
    "hub_bearing_bottom",
)
HUB_ROTOR_PART_NAMES = ("hub_shaft",)
HUB_PART_NAMES = HUB_STATIC_PART_NAMES + HUB_ROTOR_PART_NAMES


def build_turret_hub_static() -> Assembly:
    """Static hub sub-assembly: housing + the two bearings, in
    turret-local coordinates. The bearing outer race is physically
    static; the inner race spins with the shaft, but each bearing
    renders as one rigid body so we pin it to the housing side
    (visual cost is nil)."""
    a = Assembly()
    hub_pose = Pos(TURRET_HUB_X, TURRET_HUB_Y, RAIL_TOP_Z)
    a = a.with_part(
        "hub_housing", housing(), location=hub_pose,
        material=material_of("hub_housing"),
    )
    a = a.with_part(
        "hub_bearing_top",
        bearing(),
        location=hub_pose * Pos(0, 0, _TOP_BEARING_Z),
        material=material_of("hub_bearing_top"),
    )
    a = a.with_part(
        "hub_bearing_bottom",
        bearing(),
        location=hub_pose * Pos(0, 0, _BOTTOM_BEARING_Z),
        material=material_of("hub_bearing_bottom"),
    )
    return a


def build_turret_hub_rotor() -> Assembly:
    """Rotor hub sub-assembly: the shaft (spins with the spider), in
    turret-local coordinates. Bake the +length/2 Z lift into the
    placement (Cylinder's centroid is at the placement origin) so the
    part itself stays at canonical identity intrinsic — required by
    ``export_animated_glb`` for parts inside the rotor sub-assembly."""
    a = Assembly()
    hub_pose = Pos(TURRET_HUB_X, TURRET_HUB_Y, RAIL_TOP_Z)
    a = a.with_part(
        "hub_shaft", shaft(),
        location=hub_pose * Pos(0, 0, _SHAFT_LEN / 2),
        material=material_of("hub_shaft"),
    )
    return a


# --- Standalone assembly -----------------------------------------------

def build_turret_hub_standalone() -> Assembly:
    """All four turret-hub parts (housing + shaft + 2 bearings). All
    pairwise no-interference, addressed by path under the two local
    subtrees. Bearings press-fit into the housing bore (tangent
    contact, passes assert_no_interference at zero overlap) and onto
    the shaft (same)."""
    return (
        Assembly()
        .with_subassembly("static", build_turret_hub_static())
        .with_subassembly("rotor", build_turret_hub_rotor())
        .assert_no_interference_within(
            tuple(f"static.{n}" for n in HUB_STATIC_PART_NAMES)
            + tuple(f"rotor.{n}" for n in HUB_ROTOR_PART_NAMES)
        )
    )


# Degenerate master, so `khana check <file>` resolves with no `:factory`.
assembly = build_turret_hub_standalone()
