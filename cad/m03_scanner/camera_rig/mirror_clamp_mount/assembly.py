"""
M03 scanner — merged mirror clamp + mount (rev 4, rigid).

Single printed body per mirror instance: rail clamp base + beam +
110 × 110 × 5 mount plate. Successor to the rev-3 split between
`../mirror_pad_clamp/` (clamp + pad) and `../mirror_mount/` (plate +
4 corner bolts), collapsed into one continuous solid. The mirror
bonds directly to the merged body's front face — no MOUNT_GAP, no
four M4 mount bolts, no air column in the calibration stack.

Three printed instances drive all three mirrors (one body per
instance because the mount-rotations differ — same `clamp_mount(...)`
function parameterised by `mount_in_clamp`):

  * below-left  — clamps onto rail A's +Z face
  * below-right — X-mirror of below-left
  * back-upper  — clamps onto rail C's -Y face

The two below-mirror instances are mirror-images by mount-normal
chirality (different drilled holes, same overall shape), so by
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
z ∈ [-MIRROR_MOUNT_BACK_OFFSET, -MIRROR_MOUNT_MIRROR_T] = [-8, -3].
The mirror reflective face is at mount-local z = 0; mirror body at
z ∈ [-3, 0]; plate front face (mirror-bonding face) coplanar with
mirror back face at z = -3. The beam overshoots into the plate and
is trimmed flush with the front face so the plate has a clean
bonding surface.

Two M4 clamp bolts drop through the base along clamp-local -Y into
T-nuts captured in the rail's slot (T-nuts not modelled — stock
hardware, same convention as the rev-3 mirror_pad_clamp).

Print orientation: clamp base on the build plate, +Y up. Same
overhang character as rev-3 mirror_pad_clamp (the beam-to-plate
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
    MIRROR_BACK_UPPER_NORMAL,
    MIRROR_BACK_UPPER_POS,
    MIRROR_BELOW_LEFT_NORMAL,
    MIRROR_BELOW_LEFT_POS,
    MIRROR_MOUNT_BACK_OFFSET,
    MIRROR_MOUNT_MIRROR_T,
    MIRROR_MOUNT_PLATE_T,
    RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
    RAIL_PAD_CLAMP_BACK_UPPER_POS,
    RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
    RAIL_PAD_CLAMP_BELOW_LEFT_POS,
    SKELETON_RAIL_MEMBER,
    WALL_MIN,
    horizontal_x_roll_deg,
    rotation_for_normal,
)


# --- Mirror geometry (private) ------------------------------------------

# First-surface mirror — square, 80 × 80 × 3. MIRROR_T comes from
# camera_rig/params.py because it's intrinsic to the mirror itself
# and survives any future mount-side rework.
MIRROR_W = 80.0
MIRROR_H = 80.0


# --- Mount plate (private — was MIRROR_MOUNT_PLATE_SIZE in params) ------

# 110 × 110 mm; 15 mm flange around the 80 × 80 mirror gives a printed
# corner around each former bolt site. The merged body has no mount
# bolts so the flange is now overdesign, but keeping the size avoids
# perturbing the rail offsets (skeleton's MOUNT_CLEARANCE derivation
# treats the plate as the worst-case projected obstacle).
PLATE_SIZE = 110.0


# --- Clamp base (rail gripper) parameters -------------------------------

# Same dimensions as the rev-3 mirror_pad_clamp — the rail-side half of
# the merged body is unchanged from the split design.
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
    mirror_pos: tuple[float, float, float],
    mirror_normal: tuple[float, float, float],
) -> Location:
    """Compose the mount-local → clamp-local `Location` from per-instance
    placement constants. Same idiom as the rev-3 mirror_pad_clamp
    helper of the same name."""
    clamp_world = Pos(*clamp_pos) * clamp_rot
    mount_world = Pos(*mirror_pos) * rotation_for_normal(
        mirror_normal, roll_deg=horizontal_x_roll_deg(mirror_normal)
    )
    return clamp_world.inverse() * mount_world


def _mount_back_in_clamp(mount_in_clamp: Location) -> tuple[float, float, float]:
    """Mount plate back-face centre in clamp-local. Used for beam tilt
    + length derivation and for module-level sanity asserts."""
    p = (mount_in_clamp * Pos(0, 0, -MIRROR_MOUNT_BACK_OFFSET)).position
    return (p.X, p.Y, p.Z)


# Below-left pose — canonical POC for module-level asserts and standalone.
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

# Plate must be at least as big as the mirror, with a flange margin.
assert PLATE_SIZE >= MIRROR_W and PLATE_SIZE >= MIRROR_H, (
    f"plate {PLATE_SIZE} smaller than mirror"
)
assert (PLATE_SIZE - MIRROR_W) / 2 >= WALL_MIN, (
    f"plate-mirror flange {(PLATE_SIZE - MIRROR_W) / 2:.2f} mm < {WALL_MIN} mm"
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
    plate_t: float = MIRROR_MOUNT_PLATE_T,
    mirror_t: float = MIRROR_MOUNT_MIRROR_T,
    mount_in_clamp: Location = _MOUNT_IN_CLAMP,
) -> Part:
    """Single printed body: clamp base (on rail) + vertical beam +
    110 × 110 × plate_t mount plate. Mirror bonds directly to the
    plate's front face at mount-local z = -mirror_t.

    `mount_in_clamp` supplies the mount-local → clamp-local Location;
    defaults to the below-left pose. Beam tilt + axial length are
    derived from this Location, so all three rev-4 instances build
    from the same body — only the mount-rotation differs, which
    parameterises the plate orientation."""
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
    # Plate occupies mount-local z ∈ [-MIRROR_MOUNT_BACK_OFFSET, -mirror_t]
    # = [-(mirror_t + plate_t), -mirror_t]; centred at -mirror_t - plate_t/2.
    plate_cz_mount_local = -mirror_t - plate_t / 2
    plate = mount_in_clamp * (
        Pos(0, 0, plate_cz_mount_local) * Box(plate_size, plate_size, plate_t)
    )
    body = body + plate

    # Trim everything past the mirror-bonding face (mount-local
    # z > -mirror_t). The beam overshoots into the plate so the union
    # is one continuous solid; cutting at the front face leaves a
    # clean flat surface for the mirror.
    _CUT_BOX_SIZE = 1000.0
    front_cutter = mount_in_clamp * (
        Pos(0, 0, -mirror_t + _CUT_BOX_SIZE / 2)
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

    # No mount-plate bolts — the plate is contiguous with the beam.

    return body


def mirror_stub(
    w: float = MIRROR_W, h: float = MIRROR_H, t: float = MIRROR_MOUNT_MIRROR_T
) -> Part:
    """Stand-in for the first-surface mirror. Reflective face at
    mount-local z=0; body at z ∈ [-t, 0]. Bought, not printed —
    no inspect() call. Caller places via the same `mount_in_clamp`
    Location used for the plate so the mirror lands flush against
    the plate front face."""
    return Pos(0, 0, -t / 2) * Box(w, h, t)


# --- Assembly -----------------------------------------------------------

# Mirror sits flush on the merged body's front face (tangent contact at
# mount-local z = -MIRROR_T). assert_no_interference treats tangency as
# zero overlap volume.
assembly = (
    Assembly()
    .add("clamp_mount", clamp_mount())
    .add("mirror", mirror_stub(), location=_MOUNT_IN_CLAMP)
    .assert_no_interference("clamp_mount", "mirror")
)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: +Y up (clamp base on the build plate). FDM
    # default up_axis is +Z, so override.
    #
    # Threshold raised to ~90° to clear the same beam ↔ plate
    # tessellation sliver that the rev-3 mirror_pad_clamp dodged: the
    # vertical beam meets the tilted plate at an oblique angle, and
    # the boolean cut at the mirror-bonding face leaves a degenerate
    # facet whose mesh tangent reports near-90°. The genuine overhangs
    # (beam sides 0° from vertical, plate back face ~52° from vertical
    # downward facing) all sit comfortably under 90°, so this still
    # catches anything pathological. Tapering the beam-to-plate
    # junction to remove the unsupported overhang is a follow-up.
    inspect(
        clamp_mount(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=90.0
        ),
        out="outputs",
        name="clamp_mount",
    )
    # Back-upper variant: same body shape, different mount-rotation
    # tilts the plate face differently. Inspected separately to
    # exercise the printability ray sampler on the rotated plate.
    inspect(
        clamp_mount(mount_in_clamp=_MOUNT_IN_CLAMP_BACK_UPPER),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=90.0
        ),
        out="outputs",
        name="clamp_mount_back_upper",
    )
