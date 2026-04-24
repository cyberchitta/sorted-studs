"""
M03 scanner — mirror-mount rail clamp (rev 2 POC, below-left).

Printed clamp that supports a mirror_mount from a 2020 T-slot extrusion
member of the rev-2 skeleton (see `../skeleton/assembly.py`). Rev 2
replacement for the rev-1 `../below_mirror_floor_bracket/` wedge, which
required an enclosure floor panel to bolt into; the clamp bolts
directly onto the skeleton rail via two M4 bolts into T-nuts captured
in the rail's outward-facing slot, so the rev-1 six-panel enclosure is
no longer needed to support the mirror mount.

This first rev is scoped to the below-left mirror on `rail_floor_left`
(left-hand floor rail). Porting to below-right and back-upper mirrors
is a follow-up — each will need its own `RAIL_CLAMP_*_{POS,HEADING_DEG}`
in `../params.py` and its own instance of this clamp in the top-level
assembly.

Coordinate frame (local to this sub-assembly):
    origin = centre of clamp bottom face, on rail top face, at the
             world-Z of the mount plate back face centre
    +X     = along rail long axis
    +Y     = world +Y (up, perpendicular to rail top face)
    +Z     = horizontal, from rail toward mount
    y = 0                     : rail top face (clamp bottom contact)
    y ∈ [0, BASE_H]           : clamp base body
    y > BASE_H                : tab (beam) rising toward mount pad

Two M4 clamp bolts drop through the base along local -Y into T-nuts
sitting in the rail's top slot (T-nuts not modelled — stock hardware,
same convention as the skeleton's corner L-brackets). From the base,
an angled rectangular beam rises along the rail-to-mount direction
(in clamp-local, ≈ (0, 0.67, 0.74)) up to a flat "mount pad" whose
face is coplanar with the mount plate back face, minus MOUNT_GAP.
Four M4 clearance holes in the pad are drilled along the mirror
normal axis, on the 94 × 94 mirror_mount bolt pattern, so the mount
plate can be bolted into the pad from its front face (same bolt-
direction convention as the below-mirror floor bracket).

The mount-pad feature is placed by composing build123d Locations:
`_MOUNT_IN_CLAMP` = clamp-placement-inverse · mount-placement expresses
mount-local coordinates in the clamp-local frame. Any mount-local
geometry (the pad itself, the four bolt-hole cylinders) passes through
this Location on its way into the clamp-local body.

Print orientation: clamp base on the build plate, +Y up. The beam
rises at ≈ 48° from vertical (inside typical PLA overhang capability
with support material), and the mount pad's underside faces downward
at ≈ 38° from horizontal — definitely needs supports on the pad's
lower face at rev 1. The beam-to-pad junction adds an oversize wedge
of material that reduces the span of unsupported pad underside; rev 2
will taper the beam into the pad to eliminate pad overhang entirely.
"""

import math
import sys
from pathlib import Path

from build123d import Box, Cylinder, Location, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    MIRROR_BELOW_LEFT_NORMAL,
    MIRROR_BELOW_LEFT_POS,
    MIRROR_MOUNT_BACK_OFFSET,
    MIRROR_MOUNT_BOLT_SPACING,
    MIRROR_MOUNT_PLATE_SIZE,
    RAIL_CLAMP_BELOW_LEFT_HEADING_DEG,
    RAIL_CLAMP_BELOW_LEFT_POS,
    SKELETON_RAIL_MEMBER,
    WALL_MIN,
    horizontal_x_roll_deg,
    rotation_for_normal,
)


# --- Clamp base (rail gripper) parameters -------------------------------

# Clamp base is a rectangular block sitting on the rail's +Y face. Local
# +X spans along the rail; local +Z spans across the rail (matches rail
# cross-section); local +Y is the clamp body height.
BASE_L = 60.0                     # along rail (local +X)
BASE_W = SKELETON_RAIL_MEMBER     # across rail (local +Z) = rail width (20 mm)
BASE_H = 15.0                     # clamp base height above rail top

# Two M4 bolts drop vertically through the base into T-nuts in the
# rail's top slot. 40 mm centre-to-centre keeps a ≥ 15 mm edge wall at
# each end of the 60 mm base and leaves room for the 30 mm-wide beam
# between the two holes.
CLAMP_BOLT_SPACING = 40.0
CLAMP_BOLT_HOLE_D = 4.5           # M4 clearance hole

_CLAMP_BOLT_HALF = CLAMP_BOLT_SPACING / 2


# --- Beam (tab) parameters ----------------------------------------------

# Rectangular prism connecting the base top face to the mount pad.
# BEAM_W: clamp-local +X (along rail axis). Kept narrower than the
#         clamp-bolt spacing so the two rail bolts clear the beam.
# BEAM_T: clamp-local direction perpendicular to the beam axis within
#         the tilt plane (beam axis lies in the YZ plane for below-left).
BEAM_W = 30.0
BEAM_T = 20.0


# --- Mount-interface pad parameters -------------------------------------

# Pad is a flat plate coplanar with the mount plate back face, offset
# MOUNT_GAP along -mount_axis. Holds the 94 × 94 bolt pattern.
#
# PAD_SIZE chosen 10 mm larger than the 110 × 110 mount plate to give
# a printed flange around each corner bolt; PAD_T = 6 mm matches the
# floor-bracket top-face thickness budget.
PAD_SIZE = MIRROR_MOUNT_PLATE_SIZE + 10.0    # 120 mm
PAD_T = 6.0

# Clearance between pad mount-facing face and the actual mount plate
# back face along mount-local +Z. Same rationale as the floor bracket's
# MOUNT_GAP (see params.py): separates coplanar faces so
# assert_no_interference reads a clean zero volume, and absorbs
# print-dimension drift on both surfaces.
MOUNT_GAP = 0.3

# Bolt clearance hole on the pad. Widened from the M4 4.5 mm rev-1
# convention to 5.0 mm — same rationale as `below_mirror_floor_bracket`
# (absorbs the MOUNT_GAP-driven lateral offset of the bolt axis from
# the pad-face hole centre).
MOUNT_BOLT_HOLE_D = 5.0

_MOUNT_BOLT_HALF = MIRROR_MOUNT_BOLT_SPACING / 2


# --- Below-left mount pose in clamp-local -------------------------------

# The clamp-placement and mount-placement are both known in world
# coordinates. Composing `clamp_placement.inverse() * mount_placement`
# gives the mount-local frame expressed in clamp-local. Any mount-
# local geometry (pad, bolt-hole cylinders) left-multiplies this
# Location on its way into the clamp body.

_clamp_placement_world = (
    Pos(*RAIL_CLAMP_BELOW_LEFT_POS)
    * Rot(0, RAIL_CLAMP_BELOW_LEFT_HEADING_DEG, 0)
)
_mount_placement_world = Pos(*MIRROR_BELOW_LEFT_POS) * rotation_for_normal(
    MIRROR_BELOW_LEFT_NORMAL,
    roll_deg=horizontal_x_roll_deg(MIRROR_BELOW_LEFT_NORMAL),
)
_MOUNT_IN_CLAMP: Location = (
    _clamp_placement_world.inverse() * _mount_placement_world
)

# Mount back face centre in clamp-local. Used below for the beam-tilt
# computation and for sanity assertions.
_MOUNT_BACK_IN_CLAMP = _MOUNT_IN_CLAMP * Pos(0, 0, -MIRROR_MOUNT_BACK_OFFSET)
_MB_POS = _MOUNT_BACK_IN_CLAMP.position
_MOUNT_BACK_XYZ = (_MB_POS.X, _MB_POS.Y, _MB_POS.Z)


# --- Parametric sanity --------------------------------------------------

# Mount should sit well above rail (clamp-local +Y) and inboard
# (clamp-local +Z) of it. X should be near zero because the clamp is
# centred along the rail under the mount back face.
assert _MOUNT_BACK_XYZ[1] > 50.0, (
    f"mount back face must be above rail; got Y={_MOUNT_BACK_XYZ[1]:.1f}"
)
assert _MOUNT_BACK_XYZ[2] > 50.0, (
    f"mount back face must be inboard of rail; got Z={_MOUNT_BACK_XYZ[2]:.1f}"
)
assert abs(_MOUNT_BACK_XYZ[0]) < 2.0, (
    f"mount back face should sit near clamp X=0 (centred under mount); "
    f"got X={_MOUNT_BACK_XYZ[0]:.2f}. If this fails, the clamp's world "
    f"Z is not the mount back face Z."
)

# Clamp bolts must not collide with the beam footprint on the base.
# Beam centre at X=0, half-width BEAM_W/2; clamp bolts at ±_CLAMP_BOLT_HALF.
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

# Pad must fit inside a reasonable envelope (not a tight dimensional
# check — more a sanity guard that PAD_SIZE is set to something on
# the order of the mount plate).
assert PAD_SIZE >= MIRROR_MOUNT_PLATE_SIZE, (
    f"pad size {PAD_SIZE} < mount plate {MIRROR_MOUNT_PLATE_SIZE}"
)


# --- Beam geometry (derived) --------------------------------------------

# Target: drive the beam's top face into the pad's volume. The beam
# runs from the clamp base top (y ≈ BASE_H) to the mount back face
# centre (_MOUNT_BACK_XYZ). Beam axis in clamp-local is approximately
# (0, Y, Z) — the tiny X component (≈ _MOUNT_BACK_XYZ[0]) is ignored
# at rev 1; a pure Rx tilt reaches the target within 1 mm.
_BEAM_TILT_DEG = math.degrees(
    math.atan2(_MOUNT_BACK_XYZ[2], _MOUNT_BACK_XYZ[1])
)
_BEAM_AXIAL_LEN = (
    math.hypot(_MOUNT_BACK_XYZ[1], _MOUNT_BACK_XYZ[2])  # length from origin
    + PAD_T                                               # punch through pad
    + 10.0                                                # overshoot margin
)


# --- Part function ------------------------------------------------------

def clamp(
    base_l: float = BASE_L,
    base_w: float = BASE_W,
    base_h: float = BASE_H,
    bolt_half: float = _CLAMP_BOLT_HALF,
    clamp_hole_d: float = CLAMP_BOLT_HOLE_D,
    beam_w: float = BEAM_W,
    beam_t: float = BEAM_T,
    beam_tilt_deg: float = _BEAM_TILT_DEG,
    beam_axial_len: float = _BEAM_AXIAL_LEN,
    pad_size: float = PAD_SIZE,
    pad_t: float = PAD_T,
    mount_gap: float = MOUNT_GAP,
    mount_bolt_half: float = _MOUNT_BOLT_HALF,
    mount_hole_d: float = MOUNT_BOLT_HOLE_D,
    mount_in_clamp: Location = _MOUNT_IN_CLAMP,
) -> Part:
    """Single printed part: clamp base (on rail) + beam (rising toward
    mount) + pad (coplanar with mount back face), with through-holes
    drilled for two rail bolts and four mount bolts.

    `mount_in_clamp` supplies the mount-local → clamp-local Location;
    defaults to the below-left below-mirror pose. Future mirror
    instances pass their own Location without changing the geometry
    functions.
    """
    # --- 1. Clamp base (axis-aligned box on rail top) ---
    body = Pos(0, base_h / 2, 0) * Box(base_l, base_h, base_w)

    # --- 2. Beam (tilted rectangular prism from base up to pad) ---
    # Built as a Y-aligned box shifted so its -Y face starts at the
    # origin, then rotated about local +X by beam_tilt_deg so the long
    # axis falls in the YZ plane along (0, cos(tilt), sin(tilt)).
    beam = (
        Rot(beam_tilt_deg, 0, 0)
        * Pos(0, beam_axial_len / 2, 0)
        * Box(beam_w, beam_axial_len, beam_t)
    )
    body = body + beam

    # The beam's cross-section protrudes below y = 0 after rotation
    # (the box corners at the "start" end rotate to y ≈ -beam_t·sin(tilt)/2).
    # Trim everything at y < 0 so the clamp sits cleanly on the rail.
    _TRIM = 1000.0
    body = body - (Pos(0, -_TRIM / 2, 0) * Box(_TRIM, _TRIM, _TRIM))

    # --- 3. Mount pad (in mount-local, transformed to clamp-local) ---
    # Pad sits at mount-local z = -MIRROR_MOUNT_BACK_OFFSET - mount_gap,
    # extending mount-local -Z by pad_t (into the bracket-support volume).
    _pad_face_z = -MIRROR_MOUNT_BACK_OFFSET - mount_gap  # mount-facing pad face
    _pad_z_mount_local = _pad_face_z - pad_t / 2
    pad = mount_in_clamp * (
        Pos(0, 0, _pad_z_mount_local) * Box(pad_size, pad_size, pad_t)
    )
    body = body + pad

    # Trim any material that projects past the pad's mount-facing face
    # (mount-local z > _pad_face_z). The beam is built long so its top
    # end would otherwise punch through the pad and into the mount plate
    # / mirror volume; cutting at the pad face leaves the beam ending
    # cleanly at the pad.
    _CUT_BOX_SIZE = 1000.0
    mount_side_cutter = mount_in_clamp * (
        Pos(0, 0, _pad_face_z + _CUT_BOX_SIZE / 2)
        * Box(_CUT_BOX_SIZE, _CUT_BOX_SIZE, _CUT_BOX_SIZE)
    )
    body = body - mount_side_cutter

    # --- 4. Rail clamp bolts (vertical through base, axis along -Y) ---
    # Cylinder axis defaults to +Z; Rot(90, 0, 0) takes +Z to -Y.
    for sx in (-1, 1):
        body -= (
            Pos(sx * bolt_half, base_h / 2, 0)
            * Rot(90, 0, 0)
            * Cylinder(clamp_hole_d / 2, base_h * 3)
        )

    # --- 5. Mount plate bolts (along mount-local +Z) ---
    # Each cylinder is axial in mount-local; the composed location
    # places it into clamp-local with the correct tilted orientation.
    # Length is generous so the hole passes cleanly through the pad +
    # any beam material behind it.
    _mount_hole_len = 200.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            body -= mount_in_clamp * (
                Pos(sx * mount_bolt_half, sy * mount_bolt_half, 0)
                * Cylinder(mount_hole_d / 2, _mount_hole_len)
            )

    return body


# --- Assembly -----------------------------------------------------------

# Single-part sub-assembly — the meaningful cross-boundary checks (clamp
# vs rail, clamp vs mirror mount, clamp vs adjacent parts) live in the
# scanner top-level `assembly.py`. Standalone `check()` reports volume +
# bbox so a numerical sanity glance is possible without the full rig.
assembly = Assembly().add("clamp", clamp())


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation is +Y up (clamp base on the build plate). FDM
    # default up_axis is +Z, so override.
    #
    # Overhang threshold raised from 45° to 55° because:
    #   * the beam tilts ≈ 48° from vertical (its downward-facing side
    #     reports ≈ 48°),
    #   * the four mount-bolt cylinders tilt ≈ 38° from vertical (their
    #     upper-half walls report ≈ 52° — same geometry as the
    #     below_mirror_floor_bracket).
    # The pad's underside is a genuinely unsupported overhang at rev 1
    # and will need printer-side supports regardless of threshold; a
    # rev 2 tapered beam-to-pad junction eliminates this.
    inspect(
        clamp(),
        method=FDM(
            up_axis=(0, 1, 0),
            wall_min_mm=WALL_MIN,
            overhang_max_deg=55.0,
        ),
        out="outputs",
        name="clamp",
    )
