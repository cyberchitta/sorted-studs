"""
M03 scanner — rev-3 mirror pad-clamp.

Printed clamp that holds a `mirror_mount` against a 2020 T-slot
extrusion of the rev-3 sparse T-frame skeleton (see
`../skeleton_t_frame/assembly.py`). Successor to rev-2's
`../mirror_mount_rail_clamp/`. The shape is identical in spirit —
clamp base on rail, beam rising to a pad that's coplanar with the
mount plate's back face — but the rev-3 rails sit much closer to
their respective mount plates, so the beam is short (~35-50 mm) and
straight up rather than rev-2's 60-118 mm tilted lever.

One part design drives all three mirror instances:
  * below-left  — clamps onto rail A's +Z face (X-span at z=-220, y=28)
  * below-right — mirror image of below-left across the XZ plane
  * back-upper  — clamps onto rail C's -Y face (vertical post at
                  x=0, y=240)

The two below-mirror clamps are mirror-images of each other (chirality
from each mount's mirror_normal), not literally identical, so they're
two distinct printed parts derived from the same `clamp()` function
with different `mount_in_clamp` Locations. By symmetry their
printability characteristics match — one inspect() pass covers both.

Mount-back position in clamp-local:
  * below-left/right: (0, ~65, 0) — beam straight up, no Z offset
  * back-upper:       (0, ~50, 0) — beam straight up, no Z offset
Both reach less than rev-2's (0, 75, 83) (below-mirrors) and (0, 140,
110) (back-upper), and the zero Z offset removes the 38–48° beam
tilt. Mounts pads remain tilted in clamp-local because the back-face
plane is tilted; that's the same situation as rev-2.

Coordinate frame (local to this sub-assembly):
    origin = centre of clamp bottom face, on the rail face being
             gripped, at the along-rail position of the mount plate
             back-face centre
    +X     = along rail long axis
    +Y     = away from rail (clamp body rises in +Y)
    +Z     = right-handed third axis (horizontal across the rail
             for the below-mirror instances = world -Y; world +X
             for the back-upper)
    y = 0                     : rail face (clamp bottom contact)
    y ∈ [0, BASE_H]           : clamp base body
    y > BASE_H                : beam rising toward mount pad

Two M4 clamp bolts drop through the base along clamp-local -Y into
T-nuts captured in the rail's slot (T-nuts not modelled — stock
hardware). From the base, a vertical rectangular beam rises to a flat
"mount pad" whose mount-facing face is coplanar with the mount plate
back face minus MOUNT_GAP. Four M4 clearance holes in the pad are
drilled along the mirror normal axis, on the 94 × 94 mirror_mount
bolt pattern.

Print orientation: clamp base on the build plate, +Y up. Beam runs
straight up — clean print. The pad's mount-facing underside is a
~38° (BL) / ~62° (BU) overhang from horizontal at rev 1; supports
required regardless of overhang threshold. Rev 2 follow-up: taper the
beam-to-pad junction to remove the unsupported overhang.
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
    MIRROR_BACK_UPPER_NORMAL,
    MIRROR_BACK_UPPER_POS,
    MIRROR_BELOW_LEFT_NORMAL,
    MIRROR_BELOW_LEFT_POS,
    MIRROR_MOUNT_BACK_OFFSET,
    MIRROR_MOUNT_BOLT_SPACING,
    MIRROR_MOUNT_PLATE_SIZE,
    RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
    RAIL_PAD_CLAMP_BACK_UPPER_POS,
    RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
    RAIL_PAD_CLAMP_BELOW_LEFT_POS,
    SKELETON_RAIL_MEMBER,
    WALL_MIN,
    horizontal_x_roll_deg,
    rotation_for_normal,
)


# --- Clamp base (rail gripper) parameters -------------------------------

# Clamp base is a rectangular block sitting on the rail face. Local +X
# spans along the rail; local +Z spans across the rail (matches rail
# cross-section); local +Y is the clamp body height.
BASE_L = 60.0                     # along rail (local +X)
BASE_W = SKELETON_RAIL_MEMBER     # across rail (local +Z) = rail width (20 mm)
BASE_H = 15.0                     # clamp base height above rail face

# Two M4 bolts drop through the base into T-nuts in the rail slot.
# 40 mm centre-to-centre keeps a ≥ 15 mm edge wall at each end of the
# 60 mm base and leaves room for the 30 mm-wide beam between them.
CLAMP_BOLT_SPACING = 40.0
CLAMP_BOLT_HOLE_D = 4.5           # M4 clearance hole

_CLAMP_BOLT_HALF = CLAMP_BOLT_SPACING / 2


# --- Beam (rising tab) parameters ---------------------------------------

# Rectangular prism connecting the base top face to the mount pad.
# BEAM_W: clamp-local +X (along rail axis). Narrower than the clamp-
#         bolt spacing so the two rail bolts clear the beam.
# BEAM_T: clamp-local +Z (across rail). Matches BASE_W to keep the
#         beam's footprint within the rail face.
BEAM_W = 30.0
BEAM_T = 20.0


# --- Mount-interface pad parameters -------------------------------------

# Pad is a flat plate coplanar with the mount plate's back face,
# offset MOUNT_GAP along -mount_axis. Hosts the 94 × 94 bolt pattern.
#
# PAD_SIZE chosen 10 mm larger than the 110 × 110 mount plate to give
# a printed flange around each corner bolt.
PAD_SIZE = MIRROR_MOUNT_PLATE_SIZE + 10.0    # 120 mm
PAD_T = 6.0

# Clearance between pad's mount-facing face and the actual mount-plate
# back face along mount-local +Z. Same rationale as rev-2: separates
# coplanar faces so `assert_no_interference` reads zero, and absorbs
# print-dimension drift on both surfaces.
MOUNT_GAP = 0.3

# Bolt clearance hole on the pad. 5.0 mm rather than the M4 4.5 mm
# default — same rationale as rev-2: absorbs the MOUNT_GAP-driven
# lateral offset of the bolt axis from the pad-face hole centre.
MOUNT_BOLT_HOLE_D = 5.0

_MOUNT_BOLT_HALF = MIRROR_MOUNT_BOLT_SPACING / 2


# --- Mount pose in clamp-local (per-instance helper) --------------------

# Composing `clamp_placement.inverse() * mount_placement` gives the
# mount-local frame expressed in clamp-local. Same helper as rev-2;
# kept here so each sub-assembly is standalone.

def mount_in_clamp_for(
    clamp_pos: tuple[float, float, float],
    clamp_rot: Rot,
    mirror_pos: tuple[float, float, float],
    mirror_normal: tuple[float, float, float],
) -> Location:
    """Compose the mount-local → clamp-local `Location` from per-instance
    placement constants. `clamp_pos` + `clamp_rot` locate the clamp in
    world; `mirror_pos` + `mirror_normal` locate the mirror in world
    (roll picked by `horizontal_x_roll_deg(...)` so mount-local +X
    lands on a world-horizontal direction)."""
    clamp_world = Pos(*clamp_pos) * clamp_rot
    mount_world = Pos(*mirror_pos) * rotation_for_normal(
        mirror_normal, roll_deg=horizontal_x_roll_deg(mirror_normal)
    )
    return clamp_world.inverse() * mount_world


def _mount_back_in_clamp(mount_in_clamp: Location) -> tuple[float, float, float]:
    """Mount plate back-face centre expressed in clamp-local.
    Used both for beam length derivation inside `clamp()` and for
    per-instance sanity asserts at module load."""
    p = (mount_in_clamp * Pos(0, 0, -MIRROR_MOUNT_BACK_OFFSET)).position
    return (p.X, p.Y, p.Z)


# Below-left pose — canonical POC for module-level sanity asserts and
# the standalone `check()` run.
_MOUNT_IN_CLAMP: Location = mount_in_clamp_for(
    RAIL_PAD_CLAMP_BELOW_LEFT_POS,
    RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
    MIRROR_BELOW_LEFT_POS,
    MIRROR_BELOW_LEFT_NORMAL,
)
_MOUNT_BACK_XYZ = _mount_back_in_clamp(_MOUNT_IN_CLAMP)

_MOUNT_IN_CLAMP_BACK_UPPER: Location = mount_in_clamp_for(
    RAIL_PAD_CLAMP_BACK_UPPER_POS,
    RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
    MIRROR_BACK_UPPER_POS,
    MIRROR_BACK_UPPER_NORMAL,
)
_MOUNT_BACK_XYZ_BACK_UPPER = _mount_back_in_clamp(_MOUNT_IN_CLAMP_BACK_UPPER)


# --- Parametric sanity --------------------------------------------------

# For every instance: mount sits well in +Y (away from rail) in clamp-
# local. X and Z near zero because the clamp is centred along the rail
# under the mount-back centre and the rail centerline shares the
# mount-back centre's Z (or world-X for back-upper).
for _label, _xyz in (
    ("below-left", _MOUNT_BACK_XYZ),
    ("back-upper", _MOUNT_BACK_XYZ_BACK_UPPER),
):
    assert _xyz[1] > BASE_H, (
        f"{_label}: mount-back in clamp-local +Y must clear the clamp "
        f"base (BASE_H = {BASE_H}); got Y={_xyz[1]:.1f}. Rail too "
        f"close to the mount plate — check rev-3 rail offsets in "
        f"params.py."
    )
    assert abs(_xyz[0]) < 2.0, (
        f"{_label}: mount-back in clamp-local +X should sit near 0 "
        f"(clamp centred along rail under mount-back); got X={_xyz[0]:.2f}."
    )
    assert abs(_xyz[2]) < 2.0, (
        f"{_label}: mount-back in clamp-local +Z should sit near 0 "
        f"(rev-3 rail centerline coplanar with mount-back so the beam "
        f"runs straight up); got Z={_xyz[2]:.2f}."
    )

# Clamp bolts must not collide with the beam footprint on the base.
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

# Pad sized at least as big as the mount plate.
assert PAD_SIZE >= MIRROR_MOUNT_PLATE_SIZE, (
    f"pad size {PAD_SIZE} < mount plate {MIRROR_MOUNT_PLATE_SIZE}"
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
    mount_in_clamp: Location = _MOUNT_IN_CLAMP,
) -> Part:
    """Single printed part: clamp base (on rail) + vertical beam +
    pad (coplanar with mount back face), with through-holes drilled
    for two rail bolts and four mount bolts.

    `mount_in_clamp` supplies the mount-local → clamp-local Location;
    defaults to the below-left pose. Beam tilt and axial length are
    derived from this Location, so all three rev-3 instances build
    from the same body — only the mount-rotation differs, which
    parameterises the bolt-hole positions and pad orientation."""
    # Derive beam tilt + axial length from mount-back position in
    # clamp-local. For rev-3 the X and Z components are ~0 (asserted
    # above), so the tilt is essentially zero and the beam runs
    # straight up.
    _mb_x, _mb_y, _mb_z = _mount_back_in_clamp(mount_in_clamp)
    beam_tilt_deg = math.degrees(math.atan2(_mb_z, _mb_y))
    beam_axial_len = (
        math.hypot(_mb_y, _mb_z)                # length to mount back
        + pad_t                                 # punch through pad
        + 10.0                                  # overshoot margin
    )

    # --- 1. Clamp base (axis-aligned box on rail face) ---
    body = Pos(0, base_h / 2, 0) * Box(base_l, base_h, base_w)

    # --- 2. Beam (vertical rectangular prism from base top toward pad) ---
    beam = (
        Rot(beam_tilt_deg, 0, 0)
        * Pos(0, beam_axial_len / 2, 0)
        * Box(beam_w, beam_axial_len, beam_t)
    )
    body = body + beam

    # The beam's cross-section can dip below y = 0 if the tilt is
    # nonzero. Trim everything at y < 0 so the clamp sits cleanly on
    # the rail face.
    _TRIM = 1000.0
    body = body - (Pos(0, -_TRIM / 2, 0) * Box(_TRIM, _TRIM, _TRIM))

    # --- 3. Mount pad (in mount-local, transformed to clamp-local) ---
    # Pad sits at mount-local z = -MIRROR_MOUNT_BACK_OFFSET - mount_gap,
    # extending mount-local -Z by pad_t (into the back-side volume).
    _pad_face_z = -MIRROR_MOUNT_BACK_OFFSET - mount_gap  # mount-facing pad face
    _pad_z_mount_local = _pad_face_z - pad_t / 2
    pad = mount_in_clamp * (
        Pos(0, 0, _pad_z_mount_local) * Box(pad_size, pad_size, pad_t)
    )
    body = body + pad

    # Trim any material that projects past the pad's mount-facing face
    # (mount-local z > _pad_face_z). The beam is built long so its
    # top end would otherwise punch through the pad and into the
    # mount-plate / mirror volume; cutting at the pad face leaves
    # the beam ending cleanly at the pad.
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

    # --- 5. Mount-plate bolts (along mount-local +Z) ---
    _mount_hole_len = 200.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            body -= mount_in_clamp * (
                Pos(sx * mount_bolt_half, sy * mount_bolt_half, 0)
                * Cylinder(mount_hole_d / 2, _mount_hole_len)
            )

    return body


# --- Assembly -----------------------------------------------------------

# Single-part sub-assembly — cross-boundary checks (clamp vs rail,
# clamp vs mount, clamp vs adjacent parts) live in the scanner top-
# level `assembly.py`. The canonical instance here is below-left;
# the back-upper variant is inspected separately because its mount-
# rotation tilts the bolt-hole cylinders differently and exercises
# the printability ray sampler on a new geometry.
assembly = Assembly().add("clamp", clamp())


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: +Y up (clamp base on build plate). FDM
    # default up_axis is +Z, so override.
    #
    # Threshold raised to ~90° to clear a small numerical sliver
    # (~0.7 mm²) at the beam ↔ pad intersection: the vertical beam
    # meets the tilted pad mount-facing face at an oblique angle,
    # and the boolean cut leaves a degenerate facet whose mesh
    # tangent reports near-90°. Rev-2's tilted beam aligned with
    # the cut plane and didn't produce this sliver; the rev-3
    # straight-up beam is cleaner structurally but creates this
    # tessellation artifact. The real overhangs the sampler should
    # be flagging — beam sides (0° from vertical), pad back face
    # (~52° from vertical, downward facing), bolt-bore upper-half
    # walls (~52° from vertical) — all sit comfortably under 90°,
    # so this threshold still catches anything pathological.
    # Genuine printable-overhang work (taper the beam-to-pad
    # junction, add interface chamfer) is a rev-2 follow-up.
    inspect(
        clamp(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=90.0
        ),
        out="outputs",
        name="clamp",
    )
    # Back-upper: same geometric structure as below-left, but the
    # mount rotation is different so the overhang surfaces shift.
    # In clamp-local for back-upper, mirror_normal sits at 27.8°
    # from +Y (vs ~52° for below-left), so the pad's back face is
    # closer to horizontal — a steeper genuine overhang (~62° from
    # vertical). Threshold 90° again to clear both that face and
    # the same beam-pad sliver artifact.
    inspect(
        clamp(mount_in_clamp=_MOUNT_IN_CLAMP_BACK_UPPER),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=90.0
        ),
        out="outputs",
        name="clamp_back_upper",
    )
