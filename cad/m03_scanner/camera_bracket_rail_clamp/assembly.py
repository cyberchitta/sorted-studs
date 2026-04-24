"""
M03 scanner — camera-bracket rail clamp (rev 2).

Printed clamp that supports the camera bracket from the rev-2 skeleton's
`rail_front_mid` transverse (see `../skeleton/assembly.py`). Rev 2
replacement for the rev-1 `../camera_bracket_adapter/` wedge, which
required a flat enclosure front panel to bolt into; the clamp bolts
directly onto the skeleton rail via two M4 bolts into T-nuts captured
in the rail's slot, so the rev-1 six-panel enclosure is no longer
needed to support the camera.

Analogous to `../mirror_mount_rail_clamp/` but with camera-bracket
geometry substituted: the pad holds the camera bracket's 52 × 52
corner-bolt pattern (vs. the mirror mount's 94 × 94), and the pad
offset from the mount-local origin is the bracket plate thickness
(5 mm) rather than the mirror mount's plate + glass thickness (8 mm).

Coordinate frame (local to this sub-assembly):
    origin = centre of clamp bottom face, on the rail face being
             gripped (the rail's +Z world face for `rail_front_mid`),
             at the along-rail position of the bracket back face centre
    +X     = along rail long axis
    +Y     = away from rail (clamp body rises in +Y)
    +Z     = from rail toward mount
    y = 0                     : rail face (clamp bottom contact)
    y ∈ [0, BASE_H]           : clamp base body
    y > BASE_H                : tab (beam) rising toward mount pad

Placement rotation at the top level is Rx(+90°): clamp-local +X →
world +X, +Y → world +Z (away from front rail toward brick), +Z →
world -Y. With the rail centerline Y equal to the bracket back face Y,
the mount back face in clamp-local lands at ≈ (0, 49, 0) — pure +Y
reach, ~0° beam tilt. The bracket's 25° world-X tilt survives in
clamp-local as a pad tilt in the YZ plane; the pad's mount-facing
face normal falls on clamp-local (0, -cos25°, -sin25°) ≈
(0, -0.906, -0.423).

Two M4 clamp bolts drop through the base along local -Y into T-nuts
sitting in the rail's slot (T-nuts not modelled — stock hardware,
same convention as the skeleton's corner L-brackets and the mirror
clamp's rail bolts). From the base, a rectangular beam rises nearly
axial along local +Y up to a flat "mount pad" whose mount-facing face
is coplanar with the bracket back face minus MOUNT_GAP. Four M4
clearance holes in the pad are drilled along the bracket-local +Z axis
(lens axis) on the 52 × 52 bracket bolt pattern, so the bracket plate
can be bolted into the pad from its front face (same bolt-direction
convention as the rev-1 adapter's bracket-side bolt group).

Print orientation: clamp base on the build plate, +Y up. Beam runs
nearly vertical (≈ 0° tilt from vertical) — no beam overhang. The
mount pad's underside (the face coplanar with the bracket back face
minus MOUNT_GAP) is a genuinely unsupported overhang: its normal sits
25° from clamp -Y (= 65° from vertical), so the pad's downward-facing
side tilts 25° from horizontal. Same geometry as `../mirror_mount_
rail_clamp/`'s below-mirror instance where the pad tilts ≈ 38° (and
the mount-bolt bores, tilted 38° from vertical, also show up as
overhang). The rev-1 convention there is to raise `overhang_max_deg`
to 55° or 65° depending on the instance and note that the pad's
underside needs printer-side supports regardless of threshold; the
rev-2 follow-up is a lofted beam-to-pad taper that eliminates the
overhang. For the camera clamp the bolt-bore tilt is 25° from
vertical and the pad underside is 65° from vertical, so 65° matches.
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
    CAMERA_BRACKET_BOLT_SPACING,
    CAMERA_BRACKET_PLATE_SIZE,
    CAMERA_BRACKET_PLATE_T,
    CAMERA_POS,
    CAMERA_ROT_DEG,
    RAIL_CLAMP_CAMERA_CLAMP_ROT,
    RAIL_CLAMP_CAMERA_POS,
    SKELETON_RAIL_MEMBER,
    WALL_MIN,
)


# --- Clamp base (rail gripper) parameters -------------------------------

# Clamp base is a rectangular block sitting on the rail's gripped face.
# Local +X spans along the rail; local +Z spans across the rail (matches
# rail cross-section); local +Y is the clamp body height above the rail.
BASE_L = 60.0                     # along rail (local +X)
BASE_W = SKELETON_RAIL_MEMBER     # across rail (local +Z) = rail width (20 mm)
BASE_H = 15.0                     # clamp base height above rail face

# Two M4 bolts drop vertically through the base into T-nuts in the
# rail's gripped-face slot. 40 mm centre-to-centre keeps a ≥ 15 mm edge
# wall at each end of the 60 mm base and leaves room for the 30 mm-wide
# beam between the two holes. Same values as the mirror rail clamp.
CLAMP_BOLT_SPACING = 40.0
CLAMP_BOLT_HOLE_D = 4.5           # M4 clearance hole

_CLAMP_BOLT_HALF = CLAMP_BOLT_SPACING / 2


# --- Beam (tab) parameters ----------------------------------------------

# Rectangular prism connecting the base top face to the mount pad.
# BEAM_W: clamp-local +X (along rail axis). Kept narrower than the
#         clamp-bolt spacing so the two rail bolts clear the beam.
# BEAM_T: clamp-local +Z direction (across rail). Beam runs nearly
#         axial in clamp +Y (≈ 0° tilt); cross-section 30 × 20 matches
#         the mirror rail clamp convention.
BEAM_W = 30.0
BEAM_T = 20.0


# --- Mount-interface pad parameters -------------------------------------

# Pad is a flat plate coplanar with the bracket back face, offset
# MOUNT_GAP along bracket-local -Z. Holds the 52 × 52 bracket bolt
# pattern.
#
# PAD_SIZE chosen 10 mm larger than the 68 × 68 bracket plate to give
# a printed flange around each corner bolt; PAD_T = 6 mm matches the
# mirror clamp's pad thickness budget.
PAD_SIZE = CAMERA_BRACKET_PLATE_SIZE + 10.0    # 78 mm
PAD_T = 6.0

# Clearance between pad mount-facing face and the actual bracket back
# face along mount-local +Z. Same 0.3 mm convention used throughout the
# scanner assembly (floor bracket, mirror rail clamp, adapter↔wall);
# separates coplanar faces so assert_no_interference reads a clean zero
# volume, and absorbs sub-millimetre print-dimension drift.
MOUNT_GAP = 0.3

# Bolt clearance hole on the pad. M4 → 4.5 mm. The bracket tilt is 25°,
# so the MOUNT_GAP-driven lateral bolt-axis offset at the pad hole
# centre is 0.3 · tan(25°) ≈ 0.14 mm — well inside the standard M4
# clearance (4.5 − 4.0 = 0.5 mm total, 0.25 mm per side). No need to
# widen the hole (unlike `../below_mirror_floor_bracket/` where the
# 52° tilt forces a larger offset and a 5.0 mm hole).
MOUNT_BOLT_HOLE_D = 4.5

_MOUNT_BOLT_HALF = CAMERA_BRACKET_BOLT_SPACING / 2


# --- Mount pose in clamp-local (per-instance helper) --------------------

# The clamp-placement and bracket-placement are both known in world
# coordinates. Composing `clamp_placement.inverse() * bracket_placement`
# gives the bracket-local frame expressed in clamp-local. Any bracket-
# local geometry (pad, bolt-hole cylinders) left-multiplies this
# Location on its way into the clamp body.
#
# Exposed as a public helper for symmetry with
# `../mirror_mount_rail_clamp/`'s `mount_in_clamp_for(...)`. The scanner
# rig has only one camera instance today, so the top-level passes the
# canonical `CAMERA_*` constants; keeping the helper parameterised
# leaves room for a second camera viewpoint (no plan yet) without
# reshaping the API.

def mount_in_clamp_for(
    clamp_pos: tuple[float, float, float],
    clamp_rot: Rot,
    camera_pos: tuple[float, float, float],
    camera_rot_deg: tuple[float, float, float],
) -> Location:
    """Compose the bracket-local → clamp-local `Location` from per-
    instance placement constants. `clamp_pos` + `clamp_rot` locate the
    clamp in world; `camera_pos` + `camera_rot_deg` locate the camera
    bracket in world (the bracket's local origin is the PCB rear /
    bracket front face, and its local +Z maps to the lens axis)."""
    clamp_world = Pos(*clamp_pos) * clamp_rot
    mount_world = Pos(*camera_pos) * Rot(*camera_rot_deg)
    return clamp_world.inverse() * mount_world


def _bracket_back_in_clamp(mount_in_clamp: Location) -> tuple[float, float, float]:
    """Bracket back face centre expressed in clamp-local coordinates.
    Used both for the beam tilt/length derivation inside `clamp()` and
    for module-level sanity assertions."""
    p = (mount_in_clamp * Pos(0, 0, -CAMERA_BRACKET_PLATE_T)).position
    return (p.X, p.Y, p.Z)


_MOUNT_IN_CLAMP: Location = mount_in_clamp_for(
    RAIL_CLAMP_CAMERA_POS,
    RAIL_CLAMP_CAMERA_CLAMP_ROT,
    CAMERA_POS,
    CAMERA_ROT_DEG,
)
_BRACKET_BACK_XYZ = _bracket_back_in_clamp(_MOUNT_IN_CLAMP)


# --- Parametric sanity --------------------------------------------------

# Bracket back face should sit well +Y-ward of the clamp base in clamp-
# local (tab reach is entirely in +Y). X should be ≈ 0 (clamp centered
# along rail under the bracket). Z should be small (bracket tilts the
# pad in the YZ plane but the centre stays near the clamp X=0 plane).
assert _BRACKET_BACK_XYZ[1] > 40.0, (
    f"bracket back face must be away from rail (+Y); "
    f"got Y={_BRACKET_BACK_XYZ[1]:.2f}"
)
assert abs(_BRACKET_BACK_XYZ[0]) < 1.0, (
    f"bracket back face should sit near clamp X=0 (centred along rail "
    f"under bracket); got X={_BRACKET_BACK_XYZ[0]:.2f}"
)
assert abs(_BRACKET_BACK_XYZ[2]) < 1.0, (
    f"bracket back face centre should sit near clamp Z=0 (rail "
    f"centerline Y equals bracket back face Y by construction); "
    f"got Z={_BRACKET_BACK_XYZ[2]:.2f}"
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

# Pad must be at least as wide as the bracket plate.
assert PAD_SIZE >= CAMERA_BRACKET_PLATE_SIZE, (
    f"pad size {PAD_SIZE} < bracket plate {CAMERA_BRACKET_PLATE_SIZE}"
)

# Bolt pattern must fit inside the pad with a reasonable edge wall.
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
    """Single printed part: clamp base (on rail) + beam (rising toward
    bracket) + pad (coplanar with bracket back face), with through-
    holes drilled for two rail bolts and four bracket bolts.

    `mount_in_clamp` supplies the bracket-local → clamp-local Location;
    defaults to the canonical camera pose. Beam tilt and axial length
    are derived from this Location inside the function — same idiom as
    `../mirror_mount_rail_clamp/`'s `clamp()`."""
    # Derive beam tilt + length from bracket-back position in clamp-local.
    # The beam runs from the clamp base top (y ≈ base_h) to the bracket
    # back face centre; a pure Rx tilt lands within 1 mm of the target
    # (the X component of the bracket-back in clamp-local is ≤ 1 mm by
    # the sanity block above).
    _bb_x, _bb_y, _bb_z = _bracket_back_in_clamp(mount_in_clamp)
    beam_tilt_deg = math.degrees(math.atan2(_bb_z, _bb_y))
    beam_axial_len = (
        math.hypot(_bb_y, _bb_z)                # length from origin
        + pad_t                                 # punch through pad
        + 10.0                                  # overshoot margin
    )

    # --- 1. Clamp base (axis-aligned box on rail face) ---
    body = Pos(0, base_h / 2, 0) * Box(base_l, base_h, base_w)

    # --- 2. Beam (rectangular prism from base up toward pad) ---
    beam = (
        Rot(beam_tilt_deg, 0, 0)
        * Pos(0, beam_axial_len / 2, 0)
        * Box(beam_w, beam_axial_len, beam_t)
    )
    body = body + beam

    # Trim any beam material protruding below y = 0 after rotation so
    # the clamp sits cleanly on the rail face.
    _TRIM = 1000.0
    body = body - (Pos(0, -_TRIM / 2, 0) * Box(_TRIM, _TRIM, _TRIM))

    # --- 3. Mount pad (in bracket-local, transformed to clamp-local) ---
    # Pad sits at bracket-local z = -plate_t - mount_gap (= bracket back
    # face minus MOUNT_GAP), extending -Z by pad_t (into the clamp-
    # support volume).
    _pad_face_z = -plate_t - mount_gap                   # mount-facing pad face
    _pad_z_mount_local = _pad_face_z - pad_t / 2
    pad = mount_in_clamp * (
        Pos(0, 0, _pad_z_mount_local) * Box(pad_size, pad_size, pad_t)
    )
    body = body + pad

    # Trim any material that projects past the pad's mount-facing face
    # (bracket-local z > _pad_face_z). The beam is built long so its
    # top end would otherwise punch through the pad into the bracket /
    # PCB volume; cutting at the pad face leaves the beam ending
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

    # --- 5. Bracket plate bolts (along bracket-local +Z) ---
    # Each cylinder is axial in bracket-local; the composed location
    # places it into clamp-local with the correct tilted orientation.
    # Length picked so the hole passes cleanly through the pad thickness
    # (6 mm) with ~5 mm of overshoot on each side — enough to punch out
    # both faces, short enough that the extension past the pad's back
    # face stays inside the empty region directly below the pad. A
    # longer cylinder (the mirror rail clamp's 200 mm convention) would
    # work for the mirrors whose bolt lines sit far from the clamp
    # base; here the 25° tilt plus ~53 mm clamp-local reach lands the
    # two lower bolts' axes inside the base volume, so the cylinder
    # must stop short.
    _mount_hole_len = 30.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            body -= mount_in_clamp * (
                Pos(sx * mount_bolt_half, sy * mount_bolt_half, 0)
                * Cylinder(mount_hole_d / 2, _mount_hole_len)
            )

    return body


# --- Assembly -----------------------------------------------------------

# Single-part sub-assembly — the meaningful cross-boundary checks (clamp
# vs rail, clamp vs camera bracket, clamp vs adjacent parts) live in the
# scanner top-level `assembly.py`. Standalone `check()` reports volume +
# bbox so a numerical sanity glance is possible without the full rig.
assembly = Assembly().add("clamp", clamp())


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation is +Y up (clamp base on the build plate). FDM
    # default up_axis is +Z, so override.
    #
    # Overhang threshold raised from 45° to 66° because:
    #   * the pad's mount-facing face (underside when printed +Y up)
    #     sits 25° off horizontal → normal lies 65° from vertical,
    #   * the mount-bolt cylinders tilt 25° from vertical → their
    #     upper-half walls report ≈ 65° (same geometry as the
    #     below-mirror rail clamp's 62° reading at 62° tilt).
    # 66° (vs 65° for the back-upper mirror clamp) adds 1° of float
    # headroom: the camera's exact 25° bracket tilt puts the overhang
    # angle at exactly 65.0°, so an equal threshold trips on epsilon.
    # The pad's underside is a genuinely unsupported overhang at rev 1
    # and will need printer-side supports regardless of threshold; a
    # rev 2 tapered beam-to-pad junction eliminates it.
    inspect(
        clamp(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=66.0
        ),
        out="outputs",
        name="clamp",
    )
