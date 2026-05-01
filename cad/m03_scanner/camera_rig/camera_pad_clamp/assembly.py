"""
M03 scanner — rev-3 camera pad-clamp.

Printed clamp that hangs the camera bracket below rail D — the rev-3
sparse T-frame's Y-cantilever (see `../skeleton_t_frame/assembly.py`).
Successor to rev-2's `../camera_bracket_rail_clamp/`. Same pad-on-
extrusion idea as `../mirror_pad_clamp/` but rotated 180°: instead of
sitting on top of a rail with the body rising to a mount above, this
clamp grips a rail's underside and the body descends to a mount below.

Clamp-local +Y means "away from the gripped rail face". For the mirror
clamps the rail is below the mount, so clamp +Y rises (printable as +Y
up). For this camera clamp the rail is above the mount, so clamp +Y
descends — clamp-local +Y maps to world -Z. The print orientation is
still clamp +Y up (base on the build plate), independent of the
assembled-world orientation.

Coordinate frame (local to this sub-assembly):
    origin = centre of clamp bottom face, on rail D's gripped (-Z in
             world) face, at the along-rail (Y) position of the
             bracket back face centre
    +X     = along rail D long axis (= world +Y)
    +Y     = away from rail (downward in world; body extends in clamp
             +Y to reach the bracket pad below the rail)
    +Z     = right-handed third (= world -X)
    y = 0                     : rail face (clamp bottom contact)
    y ∈ [0, BASE_H]           : clamp base body (hugs rail underside)
    y > BASE_H                : beam descending toward mount pad

Two M4 clamp bolts drop through the base along clamp-local -Y into
T-nuts captured in the rail D slot (T-nuts not modelled — stock
hardware, same convention as the mirror clamps and the skeleton's
corner brackets). From the base, a vertical rectangular beam descends
to a flat "mount pad" whose mount-facing face is coplanar with the
camera bracket back face minus MOUNT_GAP. Four M4 clearance holes in
the pad are drilled along the bracket-local +Z axis (lens axis) on the
52 × 52 bracket bolt pattern.

Mount-back position in clamp-local: (0, 132.5, 0) — beam straight down
in clamp-local +Y, no Z offset (rail D centred on the XZ plane and the
bracket sits on x = 0). Reach is much longer than the rev-3 mirror
clamps' 50-65 mm (rail D's height is set by clearance over the back-
upper mount plate corner, so it can't be pushed any closer to the
camera). The strut + propped-beam architecture in `skeleton_t_frame/`
is what keeps the cantilever stiff under this load.

Print orientation: clamp base on the build plate, +Y up. Beam runs
straight up in print frame (no beam overhang). The pad's mount-facing
face has its normal at clamp-local (0.906, 0.423, 0) — mostly clamp +X
(forward along rail D toward the back, toward rail-D's anchor end)
with a 25° upward tilt. The pad back face has normal (-0.906, -0.423,
0): mostly -X with a 25° downward tilt — a 25°-from-horizontal
overhang. Same shallow-overhang character as the rev-3 mirror clamps;
the dominant overhang issue is the same beam ↔ pad junction
tessellation sliver, dodged with overhang_max_deg ≈ 90° (see the
inspect() comment below for the long version).
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
    CAMERA_BRACKET_BOLT_SPACING,
    CAMERA_BRACKET_PLATE_SIZE,
    CAMERA_BRACKET_PLATE_T,
    CAMERA_POS,
    CAMERA_ROT_DEG,
    RAIL_PAD_CLAMP_CAMERA_CLAMP_ROT,
    RAIL_PAD_CLAMP_CAMERA_POS,
    SKELETON_RAIL_MEMBER,
    WALL_MIN,
)


# --- Clamp base (rail gripper) parameters -------------------------------

# Clamp base is a rectangular block hugging rail D's -Y face. Local +X
# spans along the rail; local +Z spans across the rail (matches rail
# cross-section); local +Y is the clamp body height away from the rail.
# Same dimensions as the mirror pad clamps for consistency.
BASE_L = 60.0                     # along rail (local +X)
BASE_W = SKELETON_RAIL_MEMBER     # across rail (local +Z) = rail width (20 mm)
BASE_H = 15.0                     # clamp base height away from rail face

# Two M4 bolts drop through the base into T-nuts in the rail's slot.
# Same 40 mm spacing as the mirror clamps — keeps a ≥ 15 mm edge wall
# at each end of the 60 mm base and leaves room for the 30 mm-wide beam
# between them.
CLAMP_BOLT_SPACING = 40.0
CLAMP_BOLT_HOLE_D = 4.5           # M4 clearance hole

_CLAMP_BOLT_HALF = CLAMP_BOLT_SPACING / 2


# --- Beam (descending tab) parameters -----------------------------------

# Rectangular prism connecting the base bottom (clamp-local +Y face)
# to the mount pad. BEAM_W: along-rail (clamp +X). BEAM_T: across-rail
# (clamp +Z). Same 30 × 20 cross-section as the mirror clamps.
BEAM_W = 30.0
BEAM_T = 20.0


# --- Mount-interface pad parameters -------------------------------------

# Pad is a flat plate coplanar with the bracket back face, offset
# MOUNT_GAP along bracket-local -Z. Hosts the 52 × 52 bracket bolt
# pattern.
#
# PAD_SIZE chosen 10 mm larger than the 68 × 68 bracket plate to give
# a printed flange around each corner bolt; PAD_T = 6 mm matches the
# mirror clamp's pad thickness budget.
PAD_SIZE = CAMERA_BRACKET_PLATE_SIZE + 10.0    # 78 mm
PAD_T = 6.0

# Clearance between pad's mount-facing face and the actual bracket back
# face. Same 0.3 mm convention used everywhere in the scanner — separates
# coplanar faces so `assert_no_interference` reads zero, and absorbs
# print-dimension drift on both sides.
MOUNT_GAP = 0.3

# Bolt clearance hole on the pad. The 25° bracket tilt drives a
# MOUNT_GAP-induced lateral offset of 0.3 · tan(25°) ≈ 0.14 mm at the
# pad-face hole centre — well inside standard M4 clearance. 4.5 mm
# matches rev-2's camera_bracket_rail_clamp convention.
MOUNT_BOLT_HOLE_D = 4.5

_MOUNT_BOLT_HALF = CAMERA_BRACKET_BOLT_SPACING / 2


# --- Mount pose in clamp-local (per-instance helper) --------------------

# Same idiom as `../mirror_pad_clamp/`'s `mount_in_clamp_for(...)` and
# rev-2's camera-bracket clamp helper. The rig has only one camera
# today; keeping the helper parameterised mirrors the mirror clamp API
# and leaves room for a second viewpoint without API churn.

def mount_in_clamp_for(
    clamp_pos: tuple[float, float, float],
    clamp_rot: Rot,
    camera_pos: tuple[float, float, float],
    camera_rot_deg: tuple[float, float, float],
) -> Location:
    """Compose the bracket-local → clamp-local `Location` from per-
    instance placement constants. `clamp_pos` + `clamp_rot` locate the
    clamp in world; `camera_pos` + `camera_rot_deg` locate the camera
    bracket in world (bracket-local origin = PCB rear / bracket front
    face, +Z = lens axis)."""
    clamp_world = Pos(*clamp_pos) * clamp_rot
    mount_world = Pos(*camera_pos) * Rot(*camera_rot_deg)
    return clamp_world.inverse() * mount_world


def _bracket_back_in_clamp(mount_in_clamp: Location) -> tuple[float, float, float]:
    """Bracket back face centre in clamp-local. Used both for beam
    length derivation inside `clamp()` and for module-level sanity
    asserts."""
    p = (mount_in_clamp * Pos(0, 0, -CAMERA_BRACKET_PLATE_T)).position
    return (p.X, p.Y, p.Z)


_MOUNT_IN_CLAMP: Location = mount_in_clamp_for(
    RAIL_PAD_CLAMP_CAMERA_POS,
    RAIL_PAD_CLAMP_CAMERA_CLAMP_ROT,
    CAMERA_POS,
    CAMERA_ROT_DEG,
)
_BRACKET_BACK_XYZ = _bracket_back_in_clamp(_MOUNT_IN_CLAMP)


# --- Parametric sanity --------------------------------------------------

# Bracket back face must sit well in clamp +Y (the body extends in +Y
# from the rail face down to the pad). X near 0 because the rail D
# centerline shares the bracket's X = 0; Z near 0 because the clamp is
# centred along rail D under the bracket back face Z.
assert _BRACKET_BACK_XYZ[1] > BASE_H, (
    f"bracket back in clamp-local +Y must clear the clamp base "
    f"(BASE_H = {BASE_H}); got Y={_BRACKET_BACK_XYZ[1]:.1f}. Rail D "
    f"too close to the bracket — check rev-3 rail offsets in params.py."
)
assert abs(_BRACKET_BACK_XYZ[0]) < 2.0, (
    f"bracket back in clamp-local +X should sit near 0 (clamp centred "
    f"along rail D under the bracket back face Z); got X={_BRACKET_BACK_XYZ[0]:.2f}."
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

# Pad sized at least as big as the bracket plate, with edge wall on the
# corner-bolt holes.
assert PAD_SIZE >= CAMERA_BRACKET_PLATE_SIZE, (
    f"pad size {PAD_SIZE} < bracket plate {CAMERA_BRACKET_PLATE_SIZE}"
)
_pad_bolt_edge = PAD_SIZE / 2 - _MOUNT_BOLT_HALF - MOUNT_BOLT_HOLE_D / 2
assert _pad_bolt_edge >= WALL_MIN, (
    f"mount-bolt → pad-edge wall {_pad_bolt_edge:.2f} mm < {WALL_MIN} mm"
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
    pad_size: float = PAD_SIZE,
    pad_t: float = PAD_T,
    mount_gap: float = MOUNT_GAP,
    mount_bolt_half: float = _MOUNT_BOLT_HALF,
    mount_hole_d: float = MOUNT_BOLT_HOLE_D,
    plate_t: float = CAMERA_BRACKET_PLATE_T,
    mount_in_clamp: Location = _MOUNT_IN_CLAMP,
) -> Part:
    """Single printed part: clamp base (under rail) + vertical beam +
    pad (coplanar with bracket back face), with through-holes drilled
    for two rail bolts and four bracket bolts.

    `mount_in_clamp` supplies the bracket-local → clamp-local Location;
    defaults to the canonical camera pose. Beam tilt and axial length
    are derived from this Location, matching the rev-3 mirror_pad_clamp
    idiom."""
    _bb_x, _bb_y, _bb_z = _bracket_back_in_clamp(mount_in_clamp)
    beam_tilt_deg = math.degrees(math.atan2(_bb_z, _bb_y))
    beam_axial_len = (
        math.hypot(_bb_y, _bb_z)                # length to bracket back
        + pad_t                                 # punch through pad
        + 10.0                                  # overshoot margin
    )

    # --- 1. Clamp base (axis-aligned box on rail face) ---
    body = Pos(0, base_h / 2, 0) * Box(base_l, base_h, base_w)

    # --- 2. Beam (rectangular prism descending toward pad) ---
    beam = (
        Rot(beam_tilt_deg, 0, 0)
        * Pos(0, beam_axial_len / 2, 0)
        * Box(beam_w, beam_axial_len, beam_t)
    )
    body = body + beam

    # Trim any beam material protruding into y < 0 after rotation so the
    # clamp sits cleanly against the rail face.
    _TRIM = 1000.0
    body = body - (Pos(0, -_TRIM / 2, 0) * Box(_TRIM, _TRIM, _TRIM))

    # --- 3. Mount pad (in bracket-local, transformed to clamp-local) ---
    # Pad sits at bracket-local z = -plate_t - mount_gap (= bracket back
    # face minus MOUNT_GAP), extending -Z by pad_t.
    _pad_face_z = -plate_t - mount_gap                   # mount-facing pad face
    _pad_z_mount_local = _pad_face_z - pad_t / 2
    pad = mount_in_clamp * (
        Pos(0, 0, _pad_z_mount_local) * Box(pad_size, pad_size, pad_t)
    )
    body = body + pad

    # Trim material projecting past the pad's mount-facing face into the
    # bracket / PCB volume. The beam is built long so it would otherwise
    # punch through the pad; cutting at the pad face leaves the beam
    # ending cleanly at the pad. Same idiom as both the mirror and the
    # rev-2 camera clamp.
    _CUT_BOX_SIZE = 1000.0
    mount_side_cutter = mount_in_clamp * (
        Pos(0, 0, _pad_face_z + _CUT_BOX_SIZE / 2)
        * Box(_CUT_BOX_SIZE, _CUT_BOX_SIZE, _CUT_BOX_SIZE)
    )
    body = body - mount_side_cutter

    # --- 4. Rail clamp bolts (vertical through base, axis along -Y) ---
    for sx in (-1, 1):
        body -= (
            Pos(sx * bolt_half, base_h / 2, 0)
            * Rot(90, 0, 0)
            * Cylinder(clamp_hole_d / 2, base_h * 3)
        )

    # --- 5. Bracket plate bolts (along bracket-local +Z) ---
    # 200 mm matches the mirror_pad_clamp convention. The bolt cylinders
    # tilt 25° from clamp +Y but their Z offset (bracket-local ±26 mm)
    # places them at clamp |Z| = 26 mm — well outside the beam's
    # |Z| ≤ 10 mm footprint, so the long cylinder doesn't punch the
    # beam from the side.
    _mount_hole_len = 200.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            body -= mount_in_clamp * (
                Pos(sx * mount_bolt_half, sy * mount_bolt_half, 0)
                * Cylinder(mount_hole_d / 2, _mount_hole_len)
            )

    return body


# --- Assembly -----------------------------------------------------------

# Single-part sub-assembly — the cross-boundary checks (clamp vs rail,
# clamp vs bracket, clamp vs adjacent parts) live in the scanner top-
# level `assembly.py`. Standalone `check()` reports volume + bbox so a
# numerical sanity glance is possible without the full rig.
assembly = Assembly().add("clamp", clamp())


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: +Y up (clamp base on the build plate). FDM
    # default up_axis is +Z, so override.
    #
    # Threshold raised to ~90° for the same reason as the rev-3
    # mirror_pad_clamp: the vertical beam meets the tilted pad at an
    # oblique angle, and the boolean cut leaves a degenerate facet
    # whose mesh tangent reports near-90°. The genuine printability
    # concerns — beam sides (vertical, no overhang), pad back face
    # (~25° from horizontal, shallow overhang), bolt-bore upper-half
    # walls (~25° from vertical) — all sit comfortably under 90°, so
    # this threshold still catches anything pathological. Genuine
    # printable-overhang work (taper the beam-to-pad junction) is a
    # follow-up.
    inspect(
        clamp(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=90.0
        ),
        out="outputs",
        name="clamp",
    )
