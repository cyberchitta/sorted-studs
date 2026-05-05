"""
M03 scanner — ESP32-CAM clamp + mount (rev 5).

Single printed body per camera vertex: rail clamp base + beam + plate
that holds an AI-Thinker ESP32-CAM (M12 lens variant) by 4 corner M2
bolts. Successor to rev-4's split between three mirror_clamp_mount
instances + one camera_clamp_bracket — collapsed into a single
sub-assembly because the four tetrahedral cameras are now homogeneous
(no mirrors → no role distinction).

Four printed instances drive all four cameras (one body per instance
because the mount-rotations differ — same `clamp_mount(...)` function
parameterised by `mount_in_clamp`):

  * front-low   — clamps onto rail D's -Z (underside) face
  * back-upper  — clamps onto rail C's -Y face
  * below-left  — clamps onto rail A's +Z (top) face
  * below-right — X-mirror of below-left

The two below instances are mirror-images by lens-axis chirality
(different drilled bolt-pattern positions, same overall shape), so by
symmetry below-right's printability matches below-left's — one
inspect() pass covers both.

Coordinate frame (local to this sub-assembly):
    origin = centre of clamp bottom face, on the rail face being
             gripped, at the along-rail position of the mount plate
             back-face centre
    +X     = along rail long axis
    +Y     = away from rail (clamp body rises in +Y)
    +Z     = right-handed third axis
    y = 0                     : rail face (clamp bottom contact)
    y ∈ [0, BASE_H]           : clamp base body
    y > BASE_H                : beam rising into the mount plate

The plate sits in mount-local (transformed by `mount_in_clamp`) at
z ∈ [-PLATE_T, 0]. The PCB rear face sits on the plate's front face
(mount-local z = 0); PCB body at z ∈ [0, PCB_T]; M12 lens column
forward of that. The beam overshoots into the plate and is trimmed
flush with the front face for a clean PCB-seating surface.

Two M4 clamp bolts drop through the base along clamp-local -Y into
T-nuts captured in the rail's slot (T-nuts not modelled — stock
hardware, same convention as the rev-4 mirror_clamp_mount).

Print orientation: clamp base on the build plate, +Y up. Same overhang
character as rev-4 mirror_clamp_mount (the beam-to-plate junction on
the mount-side overhang); inspect() threshold raised to clear the same
beam ↔ plate boolean-cut tessellation sliver.

PCB hole pattern: dimensions are nominal-pending-verification. Online
sources disagree on the AI-Thinker ESP32-CAM mounting hole pitch
(~22 × 35 mm cited most commonly); calipers on a real board are the
gate before printing four for installation. M2 holes are oversized to
Ø2.4 mm so a ±0.3 mm pitch error doesn't require a reprint.
"""

import math
import sys
from pathlib import Path

from build123d import Box, Cylinder, Location, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from camera_rig.params import (  # noqa: E402
    CAMERA_MOUNT_PLATE_T,
    RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
    RAIL_PAD_CLAMP_BACK_UPPER_POS,
    RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
    RAIL_PAD_CLAMP_BELOW_LEFT_POS,
    SKELETON_RAIL_MEMBER,
    T_FRAME_RAIL_D_BOTTOM_Z,
    VERTEX_BACK_UPPER_POS,
    VERTEX_BELOW_LEFT_POS,
    WALL_MIN,
    horizontal_x_roll_deg,
    lens_axis_unit,
    rotation_for_normal,
)


# --- AI-Thinker ESP32-CAM PCB geometry (private) ------------------------

# PCB outline (rectangular). Source: cameras_design_notes.md, pending
# caliper measurement on a real board.
PCB_W = 40.5                      # along the long edge
PCB_H = 27.0                      # along the short edge
PCB_T = 4.5                       # PCB body thickness incl. underside components

# Corner M2 mounting holes. Pitch values are nominal-pending-calipers
# (see module docstring); holes oversized to absorb pitch error.
PCB_HOLE_PITCH_X = 35.0           # corner-to-corner along PCB +X (long)
PCB_HOLE_PITCH_Y = 22.0           # corner-to-corner along PCB +Y (short)
PCB_HOLE_D = 2.4                  # M2 clearance, oversized for pitch tolerance

# microSD slot (underside). Sits near one short edge; rough envelope
# pending a real-board scan.
SD_POCKET_W = 12.0                # along PCB +X
SD_POCKET_H = 4.0                 # along PCB +Y
SD_POCKET_DEPTH = 3.0             # into the plate from the front face

# PCB-trace antenna stub on one short edge — must sit in free air.
# Nominal envelope; real footprint TBD when a board is in hand.
ANTENNA_POCKET_W = 25.0           # along PCB +Y (across short edge)
ANTENNA_POCKET_H = 10.0           # PCB-stub overhang past the edge
ANTENNA_POCKET_DEPTH = 5.0        # into the plate from the front face

# M12 lens column stand-in. Lumps the M12 holder + lens stack into a
# single conservative cylinder forward of the PCB. Real M12 stacks vary
# by lens choice; envelope sized to admit any common 35° board lens.
M12_OD = 22.0
M12_L = 25.0


# --- Mount plate (private) ----------------------------------------------

# 50 × 50 mm; 4.75 mm flange around the 40.5 × 27 PCB on the long edge,
# 11.5 mm on the short edge, hosting the M2 corners with edge wall to
# spare. Square outline (vs rectangular matched to the PCB) is a
# deliberate choice: simpler boolean cuts and uniform corner radii.
PLATE_SIZE = 50.0


# --- Clamp base (rail gripper) parameters -------------------------------
#
# Identical to the rev-4 mirror_clamp_mount / camera_clamp_bracket
# rail-side half — the clamp interface is the mount-architecture-
# invariant part of the design.

BASE_L = 60.0                     # along rail (local +X)
BASE_W = SKELETON_RAIL_MEMBER     # across rail (local +Z) = rail width (20 mm)
BASE_H = 15.0                     # clamp base height above rail face

CLAMP_BOLT_SPACING = 40.0
CLAMP_BOLT_HOLE_D = 4.5           # M4 clearance hole

_CLAMP_BOLT_HALF = CLAMP_BOLT_SPACING / 2


# --- Beam (rising tab) parameters ---------------------------------------

BEAM_W = 30.0
BEAM_T = 20.0


# --- Mount pose in clamp-local (per-instance helper) --------------------

def mount_in_clamp_for(
    clamp_pos: tuple[float, float, float],
    clamp_rot: Rot,
    vertex_pos: tuple[float, float, float],
) -> Location:
    """Compose the mount-local → clamp-local `Location` from per-instance
    placement constants. The mount-local +Z axis is the camera's lens
    axis (vertex toward world origin); roll about that axis lands
    mount-local +X horizontal in world. Same idiom as rev-4
    `mirror_clamp_mount.mount_in_clamp_for(...)`, with `vertex_pos`
    replacing the (mirror_pos, mirror_normal) pair — both are derived
    from the same vertex coordinate now."""
    axis = lens_axis_unit(vertex_pos)
    clamp_world = Pos(*clamp_pos) * clamp_rot
    mount_world = Pos(*vertex_pos) * rotation_for_normal(
        axis, roll_deg=horizontal_x_roll_deg(axis)
    )
    return clamp_world.inverse() * mount_world


def _mount_back_in_clamp(mount_in_clamp: Location) -> tuple[float, float, float]:
    """Mount plate back-face centre in clamp-local. Used for beam tilt
    + length derivation and for module-level sanity asserts."""
    p = (mount_in_clamp * Pos(0, 0, -CAMERA_MOUNT_PLATE_T)).position
    return (p.X, p.Y, p.Z)


# Below-left pose — canonical POC for module-level asserts and standalone.
_MOUNT_IN_CLAMP: Location = mount_in_clamp_for(
    RAIL_PAD_CLAMP_BELOW_LEFT_POS,
    RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
    VERTEX_BELOW_LEFT_POS,
)
_MOUNT_BACK_XYZ = _mount_back_in_clamp(_MOUNT_IN_CLAMP)

_MOUNT_IN_CLAMP_BACK_UPPER: Location = mount_in_clamp_for(
    RAIL_PAD_CLAMP_BACK_UPPER_POS,
    RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
    VERTEX_BACK_UPPER_POS,
)
_MOUNT_BACK_XYZ_BACK_UPPER = _mount_back_in_clamp(_MOUNT_IN_CLAMP_BACK_UPPER)


# --- Parametric sanity --------------------------------------------------

for _label, _xyz in (
    ("below-left", _MOUNT_BACK_XYZ),
    ("back-upper", _MOUNT_BACK_XYZ_BACK_UPPER),
):
    assert _xyz[1] > BASE_H, (
        f"{_label}: mount-back in clamp-local +Y must clear the clamp "
        f"base (BASE_H = {BASE_H}); got Y={_xyz[1]:.1f}"
    )
    assert abs(_xyz[0]) < 2.0, (
        f"{_label}: mount-back in clamp-local +X should sit near 0; "
        f"got X={_xyz[0]:.2f}"
    )
    assert abs(_xyz[2]) < 2.0, (
        f"{_label}: mount-back in clamp-local +Z should sit near 0; "
        f"got Z={_xyz[2]:.2f}"
    )

# Clamp bolts must clear the beam footprint on the base.
assert _CLAMP_BOLT_HALF > BEAM_W / 2 + WALL_MIN, (
    f"clamp bolt half-spacing {_CLAMP_BOLT_HALF:.1f} mm leaves "
    f"{_CLAMP_BOLT_HALF - BEAM_W / 2:.1f} mm wall to the beam edge; "
    f"must be ≥ {WALL_MIN} mm"
)

# Clamp bolt must sit inside the base footprint with an edge wall.
_edge_x = BASE_L / 2 - _CLAMP_BOLT_HALF - CLAMP_BOLT_HOLE_D / 2
assert _edge_x >= WALL_MIN, (
    f"clamp bolt-to-base-edge wall {_edge_x:.2f} mm < {WALL_MIN} mm"
)

# Plate must be at least as big as the PCB, with a flange margin around
# both the PCB outline and the M2 corner holes.
assert PLATE_SIZE >= PCB_W and PLATE_SIZE >= PCB_H, (
    f"plate {PLATE_SIZE} smaller than PCB ({PCB_W} × {PCB_H})"
)
assert (PLATE_SIZE - PCB_W) / 2 >= WALL_MIN, (
    f"plate-PCB long-edge flange {(PLATE_SIZE - PCB_W) / 2:.2f} mm "
    f"< {WALL_MIN} mm"
)
_m2_to_edge_x = PLATE_SIZE / 2 - PCB_HOLE_PITCH_X / 2 - PCB_HOLE_D / 2
_m2_to_edge_y = PLATE_SIZE / 2 - PCB_HOLE_PITCH_Y / 2 - PCB_HOLE_D / 2
assert _m2_to_edge_x >= WALL_MIN, (
    f"M2-to-plate-edge wall (long axis) {_m2_to_edge_x:.2f} mm < {WALL_MIN} mm"
)
assert _m2_to_edge_y >= WALL_MIN, (
    f"M2-to-plate-edge wall (short axis) {_m2_to_edge_y:.2f} mm < {WALL_MIN} mm"
)

# M12 lens body must clear the M2 screw heads on the PCB front face.
# M2 corner holes sit at radial distance √((PITCH_X/2)² + (PITCH_Y/2)²)
# from the PCB centre; lens cylinder is centred on origin. ~Ø4 mm
# typical M2 pan-head footprint.
_m2_radial = math.hypot(PCB_HOLE_PITCH_X / 2, PCB_HOLE_PITCH_Y / 2)
_lens_to_m2 = _m2_radial - M12_OD / 2
assert _lens_to_m2 >= 2.0, (
    f"lens body radial gap to M2 hole centre {_lens_to_m2:.2f} mm "
    f"leaves no room for an M2 screw head"
)


# --- Part functions -----------------------------------------------------

def clamp_mount(
    base_l: float = BASE_L,
    base_w: float = BASE_W,
    base_h: float = BASE_H,
    bolt_half: float = _CLAMP_BOLT_HALF,
    clamp_hole_d: float = CLAMP_BOLT_HOLE_D,
    beam_w: float = BEAM_W,
    beam_t: float = BEAM_T,
    plate_size: float = PLATE_SIZE,
    plate_t: float = CAMERA_MOUNT_PLATE_T,
    pcb_hole_pitch_x: float = PCB_HOLE_PITCH_X,
    pcb_hole_pitch_y: float = PCB_HOLE_PITCH_Y,
    pcb_hole_d: float = PCB_HOLE_D,
    mount_in_clamp: Location = _MOUNT_IN_CLAMP,
) -> Part:
    """Single printed body: clamp base (on rail) + beam + 50 × 50 ×
    plate_t mount plate. ESP32-CAM PCB bolts to the plate's front face
    at mount-local z = 0 via four M2 screws on the corner pattern.

    `mount_in_clamp` supplies the mount-local → clamp-local Location;
    defaults to the below-left pose. Beam tilt + axial length are
    derived from this Location, so all four rev-5 instances build from
    the same body — only the mount-rotation differs, which parameterises
    the plate orientation."""
    _mb_x, _mb_y, _mb_z = _mount_back_in_clamp(mount_in_clamp)
    beam_tilt_deg = math.degrees(math.atan2(_mb_z, _mb_y))
    beam_axial_len = (
        math.hypot(_mb_y, _mb_z)   # length to mount plate back face
        + plate_t                  # punch through the plate
        + 10.0                     # overshoot margin
    )

    # --- 1. Clamp base ---
    body = Pos(0, base_h / 2, 0) * Box(base_l, base_h, base_w)

    # --- 2. Beam ---
    beam = (
        Rot(beam_tilt_deg, 0, 0)
        * Pos(0, beam_axial_len / 2, 0)
        * Box(beam_w, beam_axial_len, beam_t)
    )
    body = body + beam

    # Trim base of beam at y < 0 so the clamp sits cleanly on the rail.
    _TRIM = 1000.0
    body = body - (Pos(0, -_TRIM / 2, 0) * Box(_TRIM, _TRIM, _TRIM))

    # --- 3. Mount plate (in mount-local) ---
    # Plate occupies mount-local z ∈ [-plate_t, 0]; centred at -plate_t/2.
    # Front face at z = 0 is the PCB rest plane.
    plate_cz_mount_local = -plate_t / 2
    plate = mount_in_clamp * (
        Pos(0, 0, plate_cz_mount_local) * Box(plate_size, plate_size, plate_t)
    )
    body = body + plate

    # Trim everything past the plate front face (mount-local z > 0).
    # The beam overshoots into the plate so the union is one continuous
    # solid; cutting at the front face leaves a clean flat PCB-seating
    # surface.
    _CUT_BOX_SIZE = 1000.0
    front_cutter = mount_in_clamp * (
        Pos(0, 0, _CUT_BOX_SIZE / 2)
        * Box(_CUT_BOX_SIZE, _CUT_BOX_SIZE, _CUT_BOX_SIZE)
    )
    body = body - front_cutter

    # --- 4. Rail clamp bolts (vertical through base, axis along clamp -Y) ---
    for sx in (-1, 1):
        body -= (
            Pos(sx * bolt_half, base_h / 2, 0)
            * Rot(90, 0, 0)
            * Cylinder(clamp_hole_d / 2, base_h * 3)
        )

    # --- 5. PCB-mount M2 holes (along mount-local +Z, lens axis) ---
    # Corner pattern on a 35 × 22 mm rectangle. Holes drilled through
    # the plate; long bore (200 mm) matches the rev-4 convention.
    _hole_len = 200.0
    cx = pcb_hole_pitch_x / 2
    cy = pcb_hole_pitch_y / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            body -= mount_in_clamp * (
                Pos(sx * cx, sy * cy, 0)
                * Cylinder(pcb_hole_d / 2, _hole_len)
            )

    return body


# --- Wedge variant (back-upper only) ------------------------------------
#
# The back-upper instance lives at the rail C/D L-junction. Its
# 50 × 50 mount plate sits in the lower-corner L volume between the two
# side gussets, so the printed body grows into a wedge that grips both
# rail C's -Y face (existing clamp base) and rail D's -Z face (new
# horizontal flange). Together with the mirrored side-gusset pair, the
# joint becomes a 4-print cluster (2 gussets + 1 wedge mount + camera)
# with bolts on all four T-slot faces.
#
# Geometry sits in the same clamp-local frame as `clamp_mount(...)`
# (Rot(180, 90, 0) carries clamp axes onto world for back-upper):
#     clamp +X = world +Z (along rail C)
#     clamp +Y = world -Y (away from rail C, into mount)
#     clamp +Z = world +X (across rail)
# so the flange sits at clamp +X just below rail D's -Z face and runs
# along clamp +Y away from rail C.

WEDGE_FLANGE_L = 60.0           # along rail D (clamp +Y), from rail C -Y face
WEDGE_FLANGE_T = 6.0             # below rail D's -Z face (clamp -X thickness)
WEDGE_CORNER_GAP = 0.3           # CAD butt-joint gap (matches skeleton_t_frame)
WEDGE_FLANGE_BOLT_Y_OFFSETS = (15.0, 50.0)  # along clamp +Y from rail C -Y face

# Rail D's -Z face position in clamp-local +X (back-upper clamp pose).
# The clamp origin's world-Z = mount-back-Z (= RAIL_PAD_CLAMP_BACK_UPPER_POS[2])
# and clamp +X is world +Z, so the offset is the difference.
_RAIL_D_BOTTOM_IN_CLAMP_X = (
    T_FRAME_RAIL_D_BOTTOM_Z - RAIL_PAD_CLAMP_BACK_UPPER_POS[2]
)


# --- Wedge parametric sanity --------------------------------------------

assert _RAIL_D_BOTTOM_IN_CLAMP_X > BASE_L / 2, (
    f"rail D's -Z face in clamp-local +X "
    f"({_RAIL_D_BOTTOM_IN_CLAMP_X:.2f} mm) must sit above the clamp "
    f"base's +X edge (BASE_L/2 = {BASE_L / 2}); check "
    f"RAIL_PAD_CLAMP_BACK_UPPER_POS / T_FRAME_RAIL_D_BOTTOM_Z"
)
assert _RAIL_D_BOTTOM_IN_CLAMP_X - WEDGE_CORNER_GAP - WEDGE_FLANGE_T > 0, (
    f"flange thickness {WEDGE_FLANGE_T} mm too large for clamp-local "
    f"X-budget below rail D bottom face at {_RAIL_D_BOTTOM_IN_CLAMP_X:.2f} mm"
)
for _dy in WEDGE_FLANGE_BOLT_Y_OFFSETS:
    assert (
        WALL_MIN + CLAMP_BOLT_HOLE_D / 2
        <= _dy
        <= WEDGE_FLANGE_L - WALL_MIN - CLAMP_BOLT_HOLE_D / 2
    ), (
        f"flange bolt at clamp Y={_dy} needs WALL_MIN edge wall inside "
        f"flange length [0, {WEDGE_FLANGE_L}]"
    )
assert (BASE_W - CLAMP_BOLT_HOLE_D) / 2 >= WALL_MIN, (
    f"flange bolt-to-side wall {(BASE_W - CLAMP_BOLT_HOLE_D) / 2:.2f} mm "
    f"< {WALL_MIN} mm"
)


# --- Part functions (continued) -----------------------------------------

def wedge_clamp_mount(
    base_l: float = BASE_L,
    base_w: float = BASE_W,
    base_h: float = BASE_H,
    bolt_half: float = _CLAMP_BOLT_HALF,
    clamp_hole_d: float = CLAMP_BOLT_HOLE_D,
    beam_w: float = BEAM_W,
    beam_t: float = BEAM_T,
    plate_size: float = PLATE_SIZE,
    plate_t: float = CAMERA_MOUNT_PLATE_T,
    pcb_hole_pitch_x: float = PCB_HOLE_PITCH_X,
    pcb_hole_pitch_y: float = PCB_HOLE_PITCH_Y,
    pcb_hole_d: float = PCB_HOLE_D,
    mount_in_clamp: Location = _MOUNT_IN_CLAMP_BACK_UPPER,
    flange_l: float = WEDGE_FLANGE_L,
    flange_t: float = WEDGE_FLANGE_T,
    rail_d_bottom_x: float = _RAIL_D_BOTTOM_IN_CLAMP_X,
    flange_bolt_y_offsets: tuple[float, ...] = WEDGE_FLANGE_BOLT_Y_OFFSETS,
    corner_gap: float = WEDGE_CORNER_GAP,
) -> Part:
    """Back-upper variant of `clamp_mount`. Adds a horizontal flange on
    rail D's -Z face that turns the clamp into an L-bracket spanning
    the C/D joint.

    The clamp base, beam, mount plate, M2 PCB holes, and rail-C clamp
    bolts are inherited verbatim from `clamp_mount(...)`. The wedge
    adds (a) a flange of size `flange_t × flange_l × base_w` pressed
    against rail D's -Z face with `corner_gap` clearance and (b) two
    M4 clearance holes through the flange in clamp-local ±X for T-nuts
    in rail D's bottom T-slot. The rail-D contact face sits at clamp
    +X = `rail_d_bottom_x - corner_gap`."""
    body = clamp_mount(
        base_l=base_l, base_w=base_w, base_h=base_h,
        bolt_half=bolt_half, clamp_hole_d=clamp_hole_d,
        beam_w=beam_w, beam_t=beam_t,
        plate_size=plate_size, plate_t=plate_t,
        pcb_hole_pitch_x=pcb_hole_pitch_x,
        pcb_hole_pitch_y=pcb_hole_pitch_y,
        pcb_hole_d=pcb_hole_d,
        mount_in_clamp=mount_in_clamp,
    )

    # Flange: occupies clamp-local
    #   X ∈ [flange_top_x - flange_t, flange_top_x]
    #   Y ∈ [0, flange_l]   (clamp +Y = world -Y, forward along rail D)
    #   Z ∈ [-base_w/2, base_w/2]   (matching rail D's 20 mm width)
    flange_top_x = rail_d_bottom_x - corner_gap
    flange_cx = flange_top_x - flange_t / 2
    flange_cy = flange_l / 2
    body = body + (
        Pos(flange_cx, flange_cy, 0) * Box(flange_t, flange_l, base_w)
    )

    # M4 T-nut clearance holes through the flange (axis = clamp +X).
    for dy in flange_bolt_y_offsets:
        body -= (
            Pos(flange_cx, dy, 0)
            * Rot(0, 90, 0)
            * Cylinder(clamp_hole_d / 2, flange_t * 3)
        )

    return body


def camera_pcb_stub(
    w: float = PCB_W, h: float = PCB_H, t: float = PCB_T
) -> Part:
    """ESP32-CAM PCB stand-in. Rear face at mount-local z=0 (against
    plate front face); body at z ∈ [0, t]. Bought, not printed."""
    return Pos(0, 0, t / 2) * Box(w, h, t)


def m12_lens_stub(
    od: float = M12_OD,
    length: float = M12_L,
    pcb_t: float = PCB_T,
) -> Part:
    """M12 lens + holder column stand-in. Cylinder centred on the
    mount-local +Z axis (lens axis), starting at the PCB front face
    (z = pcb_t) and extending forward. Bought, not printed."""
    z_bot = pcb_t
    z_top = z_bot + length
    cz = (z_bot + z_top) / 2
    return Pos(0, 0, cz) * Cylinder(od / 2, length)


# --- Assembly -----------------------------------------------------------

# PCB sits flush on the merged body's front face (tangent contact at
# mount-local z = 0). Lens column extends forward from the PCB top
# face. assert_no_interference treats tangency as zero overlap volume.
assembly = (
    Assembly()
    .add("clamp_mount", clamp_mount())
    .add("pcb", camera_pcb_stub(), location=_MOUNT_IN_CLAMP)
    .add("lens", m12_lens_stub(), location=_MOUNT_IN_CLAMP)
    .assert_no_interference("clamp_mount", "pcb")
    .assert_no_interference("clamp_mount", "lens")
    .assert_no_interference("pcb", "lens")
)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: +Y up (clamp base on the build plate). FDM
    # default up_axis is +Z, so override.
    #
    # Threshold raised to ~90° to clear the same beam ↔ plate
    # tessellation sliver that the rev-4 mirror_clamp_mount /
    # camera_clamp_bracket dodged: the vertical beam meets the tilted
    # plate at an oblique angle, and the boolean cut at the plate
    # front face leaves a degenerate facet whose mesh tangent reports
    # near-90°. The genuine overhangs all sit comfortably under 90°,
    # so this still catches anything pathological.
    #
    # Wedge variant (back-upper) runs first so its diagnostic gets
    # written even though the default below-left body has a known
    # pre-existing wall_min failure that ends the standalone run.
    inspect(
        wedge_clamp_mount(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=90.0
        ),
        out="outputs",
        name="wedge_clamp_mount_back_upper",
    )
    inspect(
        clamp_mount(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=90.0
        ),
        out="outputs",
        name="clamp_mount",
    )
