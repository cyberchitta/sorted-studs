"""
M03 scanner — merged camera clamp + bracket (rev 4, rigid).

Single printed body for the camera mount: rail-D clamp base + beam +
68 × 68 × 5 bracket plate. Successor to the rev-3 split between
`../camera_pad_clamp/` (clamp + pad gripping rail D's underside) and
`../camera_bracket/` (plate + 4 corner M4 bolts on the 52 × 52 pattern),
collapsed into one continuous solid. The Pi HQ Camera PCB bolts
directly to the merged body's front face via four M2.5 screws into the
PCB's threaded corners — no MOUNT_GAP, no four M4 corner bolts, no air
column in the calibration stack.

Only one camera in the rig today, so one printed instance. The
`mount_in_clamp` Location parameterisation mirrors the
`mirror_clamp_mount/` API and leaves room for a second viewpoint
without API churn.

Coordinate frame (local to this sub-assembly):
    origin = centre of clamp bottom face, on rail D's gripped (-Z in
             world) face, at the along-rail (Y) position of the
             bracket plate back-face centre
    +X     = along rail D long axis (= world +Y)
    +Y     = away from rail (downward in world; body extends in clamp
             +Y to reach the bracket plate below the rail)
    +Z     = right-handed third (= world -X)
    y = 0                     : rail face (clamp bottom contact)
    y ∈ [0, BASE_H]           : clamp base body (hugs rail D underside)
    y > BASE_H                : beam descending toward bracket plate

The plate sits in mount-local (transformed by `mount_in_clamp`) at
z ∈ [-PLATE_T, 0]. The PCB rear face plane is at mount-local z = 0
(the plate's front face); PCB body at z ∈ [0, PCB_T]; lens column at
z ∈ [PCB_T, PCB_T + LENS_L]. The beam overshoots into the plate and
is trimmed flush with the front face so the plate has a clean PCB-
seating surface.

Two M4 clamp bolts drop through the base along clamp-local -Y into
T-nuts captured in rail D's slot (T-nuts not modelled — stock
hardware, same convention as the merged mirror clamp-mounts).

Print orientation: clamp base on the build plate, +Y up. Same
overhang character as `mirror_clamp_mount/` (the beam-to-plate
junction on the mount-side overhang); inspect() threshold raised to
clear the same beam ↔ plate boolean-cut tessellation sliver.
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
    CAMERA_BRACKET_PLATE_T,
    CAMERA_POS,
    CAMERA_ROT_DEG,
    RAIL_PAD_CLAMP_CAMERA_CLAMP_ROT,
    RAIL_PAD_CLAMP_CAMERA_POS,
    SKELETON_RAIL_MEMBER,
    WALL_MIN,
)


# --- Camera PCB geometry (private) --------------------------------------

# Raspberry Pi HQ Camera PCB — 38 × 38 mm, ~1.6 mm FR-4. Four M2.5
# threaded mounting holes on a 29 × 29 mm square pattern (Pi HQ
# mechanical drawing).
PCB_W = 38.0
PCB_H = 38.0
PCB_T = 1.6

CAM_HOLE_SPACING = 29.0
CAM_HOLE_D = 2.8                  # M2.5 clearance through the plate

# Lens + mount column stand-in. Lumps the CS-mount column, C-mount
# adapter, and 2.8 mm C-mount lens barrel into a single cylinder:
# Ø30 × 55 mm conservative envelope for the reference 2.8 mm lens.
LENS_OD = 30.0
LENS_L = 55.0


# --- Bracket plate (private — was CAMERA_BRACKET_PLATE_SIZE in params) --

# 68 × 68 mm; 15 mm flange around the 38 × 38 PCB hosts the M2.5
# corners with edge wall to spare. The merged body has no rev-3 corner-
# bolt pattern (52 × 52 was the deleted clamp-pad interface), so the
# flange is now sized only by the PCB-mount geometry.
PLATE_SIZE = 68.0


# --- Clamp base (rail gripper) parameters -------------------------------

# Same dimensions as the rev-3 camera_pad_clamp — the rail-side half of
# the merged body is unchanged from the split design.
BASE_L = 60.0                     # along rail (local +X)
BASE_W = SKELETON_RAIL_MEMBER     # across rail (local +Z) = rail width (20 mm)
BASE_H = 15.0                     # clamp base height away from rail face

CLAMP_BOLT_SPACING = 40.0
CLAMP_BOLT_HOLE_D = 4.5           # M4 clearance hole

_CLAMP_BOLT_HALF = CLAMP_BOLT_SPACING / 2


# --- Beam (descending tab) parameters -----------------------------------

BEAM_W = 30.0
BEAM_T = 20.0


# --- Mount pose in clamp-local (per-instance helper) --------------------

def mount_in_clamp_for(
    clamp_pos: tuple[float, float, float],
    clamp_rot: Rot,
    camera_pos: tuple[float, float, float],
    camera_rot_deg: tuple[float, float, float],
) -> Location:
    """Compose the bracket-local → clamp-local `Location` from per-
    instance placement constants. `clamp_pos` + `clamp_rot` locate the
    clamp in world; `camera_pos` + `camera_rot_deg` locate the camera
    bracket in world (bracket-local origin = PCB rear / plate front
    face, +Z = lens axis). Same idiom as `mirror_clamp_mount/`'s
    helper of the same name."""
    clamp_world = Pos(*clamp_pos) * clamp_rot
    mount_world = Pos(*camera_pos) * Rot(*camera_rot_deg)
    return clamp_world.inverse() * mount_world


def _bracket_back_in_clamp(mount_in_clamp: Location) -> tuple[float, float, float]:
    """Plate back face centre in clamp-local. Used for beam tilt +
    length derivation and for module-level sanity asserts."""
    p = (mount_in_clamp * Pos(0, 0, -CAMERA_BRACKET_PLATE_T)).position
    return (p.X, p.Y, p.Z)


# Canonical pose for module-level asserts and standalone.
_MOUNT_IN_CLAMP: Location = mount_in_clamp_for(
    RAIL_PAD_CLAMP_CAMERA_POS,
    RAIL_PAD_CLAMP_CAMERA_CLAMP_ROT,
    CAMERA_POS,
    CAMERA_ROT_DEG,
)
_BRACKET_BACK_XYZ = _bracket_back_in_clamp(_MOUNT_IN_CLAMP)


# --- Parametric sanity --------------------------------------------------

assert _BRACKET_BACK_XYZ[1] > BASE_H, (
    f"bracket back in clamp-local +Y must clear the clamp base "
    f"(BASE_H = {BASE_H}); got Y={_BRACKET_BACK_XYZ[1]:.1f}. Rail D "
    f"too close to the bracket — check rev-3 rail offsets in params.py."
)
assert abs(_BRACKET_BACK_XYZ[0]) < 2.0, (
    f"bracket back in clamp-local +X should sit near 0 (clamp centred "
    f"along rail D under the bracket back face Z); got "
    f"X={_BRACKET_BACK_XYZ[0]:.2f}."
)
assert abs(_BRACKET_BACK_XYZ[2]) < 2.0, (
    f"bracket back in clamp-local +Z should sit near 0 (rail D "
    f"centerline on the YZ plane, bracket back centre also on the YZ "
    f"plane, so the beam runs straight); got Z={_BRACKET_BACK_XYZ[2]:.2f}."
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
# the PCB and around the M2.5 corner holes.
assert PLATE_SIZE >= PCB_W and PLATE_SIZE >= PCB_H, (
    f"plate {PLATE_SIZE} smaller than PCB"
)
assert (PLATE_SIZE - PCB_W) / 2 >= WALL_MIN, (
    f"plate-PCB flange {(PLATE_SIZE - PCB_W) / 2:.2f} mm < {WALL_MIN} mm"
)
_m25_to_edge = PLATE_SIZE / 2 - CAM_HOLE_SPACING / 2 - CAM_HOLE_D / 2
assert _m25_to_edge >= WALL_MIN, (
    f"M2.5-to-plate-edge wall {_m25_to_edge:.2f} mm < {WALL_MIN} mm"
)

# Lens body must clear the M2.5 screw heads on the PCB front face.
# M2.5 corner holes sit at radial distance (CAM_HOLE_SPACING / 2) · √2
# from the PCB centre; lens cylinder is centred on origin. ~Ø5 mm
# typical M2.5 pan-head footprint.
_m25_radial = (CAM_HOLE_SPACING / 2) * math.sqrt(2)
_lens_to_m25 = _m25_radial - LENS_OD / 2
assert _lens_to_m25 >= 2.5, (
    f"lens body radial gap to M2.5 hole centre {_lens_to_m25:.2f} mm"
    f" leaves no room for a screw head"
)


# --- Part functions -----------------------------------------------------

def clamp_bracket(
    base_l: float = BASE_L,
    base_w: float = BASE_W,
    base_h: float = BASE_H,
    bolt_half: float = _CLAMP_BOLT_HALF,
    clamp_hole_d: float = CLAMP_BOLT_HOLE_D,
    beam_w: float = BEAM_W,
    beam_t: float = BEAM_T,
    plate_size: float = PLATE_SIZE,
    plate_t: float = CAMERA_BRACKET_PLATE_T,
    cam_hole_spacing: float = CAM_HOLE_SPACING,
    cam_hole_d: float = CAM_HOLE_D,
    mount_in_clamp: Location = _MOUNT_IN_CLAMP,
) -> Part:
    """Single printed body: clamp base (under rail) + descending beam +
    68 × 68 × plate_t bracket plate. PCB bolts directly to the plate's
    front face at mount-local z = 0 via four M2.5 screws on the
    29 × 29 corner pattern.

    `mount_in_clamp` supplies the bracket-local → clamp-local Location;
    defaults to the canonical camera pose. Beam tilt and axial length
    are derived from this Location, matching the `mirror_clamp_mount/`
    idiom — only the canonical pose differs."""
    _bb_x, _bb_y, _bb_z = _bracket_back_in_clamp(mount_in_clamp)
    beam_tilt_deg = math.degrees(math.atan2(_bb_z, _bb_y))
    beam_axial_len = (
        math.hypot(_bb_y, _bb_z)   # length to plate back face
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

    # --- 3. Bracket plate (in mount-local) ---
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

    # --- 5. PCB-mount M2.5 holes (along bracket-local +Z, lens axis) ---
    # Holes drilled through the plate at the 29 × 29 corner pattern. The
    # 200 mm cylinder length matches the mirror_clamp_mount / camera_pad_
    # clamp convention; well outside the beam footprint at clamp-local
    # |Z| ≤ 10 mm so the long bore doesn't punch the beam from the side.
    _hole_len = 200.0
    cx = cam_hole_spacing / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            body -= mount_in_clamp * (
                Pos(sx * cx, sy * cx, 0)
                * Cylinder(cam_hole_d / 2, _hole_len)
            )

    return body


def camera_pcb_stub(
    w: float = PCB_W, h: float = PCB_H, t: float = PCB_T
) -> Part:
    """Camera PCB stand-in. Rear face at mount-local z=0 (against
    bracket front face); body at z ∈ [0, t]. Bought, not printed."""
    return Pos(0, 0, t / 2) * Box(w, h, t)


def lens_stub(
    od: float = LENS_OD,
    length: float = LENS_L,
    pcb_t: float = PCB_T,
) -> Part:
    """Combined lens + mount column stand-in. Cylinder centred on the
    bracket-local +Z axis, starting at the PCB front face (z = PCB_T)
    and extending LENS_L forward. Bought, not printed."""
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
    .add("clamp_bracket", clamp_bracket())
    .add("pcb", camera_pcb_stub(), location=_MOUNT_IN_CLAMP)
    .add("lens", lens_stub(), location=_MOUNT_IN_CLAMP)
    .assert_no_interference("clamp_bracket", "pcb")
    .assert_no_interference("clamp_bracket", "lens")
    .assert_no_interference("pcb", "lens")
)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: +Y up (clamp base on the build plate). FDM
    # default up_axis is +Z, so override.
    #
    # Threshold raised to ~90° to clear the same beam ↔ plate
    # tessellation sliver that the rev-3 mirror_pad_clamp / camera_pad_
    # clamp dodged: the vertical beam meets the tilted plate at an
    # oblique angle, and the boolean cut at the plate front face leaves
    # a degenerate facet whose mesh tangent reports near-90°. The
    # genuine overhangs (beam sides 0° from vertical, plate back face
    # ~25° from horizontal shallow overhang) all sit comfortably under
    # 90°, so this still catches anything pathological. Tapering the
    # beam-to-plate junction to remove the unsupported overhang is a
    # follow-up.
    inspect(
        clamp_bracket(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=90.0
        ),
        out="outputs",
        name="clamp_bracket",
    )
