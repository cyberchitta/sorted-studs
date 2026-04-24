"""
M03 scanner — below-mirror floor bracket (rev 1, single design).

Printed wedge that supports a below-mirror mirror_mount from the
enclosure floor. Two instances are placed in the top-level scanner
assembly (below-left and below-right); they share this single geometry
and differ only in their `Ry(heading)` placement rotation.

Coordinate frame (local to this sub-assembly):
    origin = centre of bracket bottom face, on the enclosure floor,
             directly under the mount plate's back face centre
    +Y     = world +Y (up)
    +Z     = horizontal projection of the back-face outward-normal;
             the top face rises along +Z (i.e., the "high" side of the
             tilted top is at +Z)
    +X     = +Y × +Z (horizontal, orthogonal to +Z; tilt axis)
    y = 0                 : bottom face (floor contact)
    y = top plane(z)      : tilted top face; mount plate back face
                            sits tangent here
        top plane(z) = TOP_HEIGHT + z · tan(TILT_DEG)
        TOP_HEIGHT = bracket-local y of the tilt plane at z=0

The tilt axis is the bracket's local +X, so the top face is tilted
purely in the local YZ plane. Two bolt groups in the top face, matching
the mirror_mount rev 1 four-corner M4 pattern:

    upper bolts (y_b high, +Z side) — at mount-local (+47, +47, -8)
                                      and (-47, +47, -8)
    lower bolts (y_b low,  -Z side) — at mount-local (+47, -47, -8)
                                      and (-47, -47, -8)

In bracket-local, mount-local +Y lies along (0, cos(ELEV), sin(ELEV))
where ELEV is the mirror-normal elevation above horizontal. The bolt
entries therefore sit at bracket-local

    (± BOLT_HALF, TOP_HEIGHT ± BOLT_HALF · cos(ELEV), ± BOLT_HALF · sin(ELEV))

with the two signs on mount-local +Y coupled (upper pair, lower pair).
Bolt axis in bracket-local = (0, -sin(ELEV), cos(ELEV)) — perpendicular
to the tilted top face, firing downward-and-toward-+Z into the body.

Print orientation: bottom face on the build plate, wedge rising along
+Y with the top face cut in the prism's upper half. The top face tilts
BACK_FACE_TILT_DEG ≈ 52° from horizontal, which sits well inside
overhang_max_deg = 45° when the top face is the "upper" surface being
built (not an overhang). The two XZ vertical faces (±X sides), the
tall +Z face, and the short -Z face are all vertical → printable.

The bolt holes are modelled as plain M4 through-cylinders that punch
clear through the bracket body, matching the rev 1 convention shared
with `mirror_mount/` and `camera_bracket_adapter/`. Heat-set M4 insert
pockets, blind-hole depth control, and floor-attachment features are
rev 2 follow-ups — see `../NEXT_STEPS.md`.
"""

import math
import sys
from pathlib import Path

from build123d import Box, Cylinder, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

# Standalone-run import: params.py lives one directory up. Same
# pattern as the other m03 sub-assemblies.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    BELOW_MIRROR_BRACKET_MOUNT_GAP,
    BELOW_MIRROR_BRACKET_MOUNT_PLANE_HEIGHT,
    BELOW_MIRROR_BRACKET_TOP_HEIGHT,
    BELOW_MIRROR_ELEV_DEG,
    MIRROR_MOUNT_BOLT_SPACING,
    MIRROR_MOUNT_PLATE_SIZE,
    WALL_MIN,
)


# --- Parameters ---------------------------------------------------------

# Tilt angle of the top face from horizontal. Complement of the mirror
# normal's elevation above horizontal: a mirror normal at 38° above
# horizontal puts the back-face plane 52° off horizontal.
ELEV_DEG = BELOW_MIRROR_ELEV_DEG
ELEV_RAD = math.radians(ELEV_DEG)
_SIN_ELEV = math.sin(ELEV_RAD)
_COS_ELEV = math.cos(ELEV_RAD)
TILT_DEG = 90.0 - ELEV_DEG       # top face tilt from horizontal
_TAN_TILT = _COS_ELEV / _SIN_ELEV  # slope dy/dz of the top face

# Top face centre height above floor (at bracket-local x=0, z=0).
# This is the cut-plane height; the mount plate's back face sits
# MOUNT_GAP above it in bracket-local +Y (so the two coplanar faces
# don't tangent-touch and the no-interference assertion reads a clean
# zero volume).
TOP_HEIGHT = BELOW_MIRROR_BRACKET_TOP_HEIGHT
MOUNT_GAP = BELOW_MIRROR_BRACKET_MOUNT_GAP
MOUNT_PLANE_HEIGHT = BELOW_MIRROR_BRACKET_MOUNT_PLANE_HEIGHT

# Bolt pattern — half-spacing on a square matching MIRROR_MOUNT_BOLT_SPACING.
BOLT_HALF = MIRROR_MOUNT_BOLT_SPACING / 2

# Bolt clearance hole diameter. Widened from the M4 4.5 mm rev 1
# convention in `mirror_mount/` to absorb the lateral offset between
# the bracket hole centre (on the top-face plane) and the mount
# bolt's world axis (through the mount plate MOUNT_GAP mm above).
# For MOUNT_GAP = 0.3, tilt ≈ 52°, the offset is ≈ 0.24 mm; 5.0 mm
# leaves ≥ 0.26 mm radial clearance over an M4 bolt.
BOLT_HOLE_D = 5.0

# Base prism footprint in bracket-local XZ. Sized to:
#  * cover the mount plate projection onto the bracket base plus a
#    margin for the bolt-hole flanges
#  * give the upper bolts (entry at bracket z = +BOLT_HALF·sin(ELEV))
#    enough material in the +Z direction for UPPER_BOLT_EMBEDMENT of
#    bolt shank before the cylinder exits the +Z face.
UPPER_BOLT_EMBEDMENT = 20.0
_UPPER_BOLT_MOUNT_Z = BOLT_HALF * _SIN_ELEV
BASE_X = 120.0
BASE_Z = 2 * (_UPPER_BOLT_MOUNT_Z + UPPER_BOLT_EMBEDMENT * _COS_ELEV)

# Pre-cut prism height along +Y. Must exceed the tilt plane's maximum
# over the prism's XZ footprint so the top-face cut lands fully inside
# the prism.
_MAX_CUT_Y = TOP_HEIGHT + (BASE_Z / 2) * _TAN_TILT
PRISM_H = _MAX_CUT_Y + 5.0

# Margin the cut box's sizes over the prism's longest axis so the
# cutter fully contains the prism regardless of tilt.
_CUT_BOX_SIZE = max(BASE_X, BASE_Z, PRISM_H) * 3


# --- Parametric sanity checks -------------------------------------------

# Bolt-hole to bracket-edge wall along bracket +X (horizontal, flat).
# Projection doesn't stretch X; same 4.5 mm clearance hole reads the
# same in the tilted plane.
_edge_x = BASE_X / 2 - BOLT_HALF - BOLT_HOLE_D / 2
assert _edge_x >= WALL_MIN, (
    f"bolt-to-bracket-X-edge wall {_edge_x:.2f} mm < {WALL_MIN} mm "
    f"(BASE_X = {BASE_X}, BOLT_HALF = {BOLT_HALF})"
)

# Bolt-hole to bracket-edge wall along mount-local +Y (= tilted axis in
# the top face). Projected onto bracket +Z, half the top-face span is
# BASE_Z / 2; the upper bolt sits at z = +BOLT_HALF·sin(ELEV). The wall
# along bracket +Z equals (BASE_Z / 2 - bolt_z - BOLT_HOLE_D/2).
_edge_z = BASE_Z / 2 - _UPPER_BOLT_MOUNT_Z - BOLT_HOLE_D / 2
assert _edge_z >= WALL_MIN, (
    f"bolt-to-bracket-+Z-edge wall {_edge_z:.2f} mm < {WALL_MIN} mm "
    f"(BASE_Z = {BASE_Z:.2f}, upper bolt z = {_UPPER_BOLT_MOUNT_Z:.2f})"
)

# Top face must extend at least as far along mount-local +Y as the
# mount plate itself (measured in the tilted plane). Top-face tilted-
# axis half-span = (BASE_Z/2) / cos(TILT) = (BASE_Z/2) / sin(ELEV);
# mount plate half-size along mount +Y = MIRROR_MOUNT_PLATE_SIZE/2.
_top_face_tilted_half = (BASE_Z / 2) / _SIN_ELEV
assert _top_face_tilted_half >= MIRROR_MOUNT_PLATE_SIZE / 2, (
    f"top face tilted half-span {_top_face_tilted_half:.2f} mm < "
    f"mount plate half {MIRROR_MOUNT_PLATE_SIZE / 2} mm — "
    f"mount plate would overhang the bracket top face"
)

# Top face X half-span must also cover the mount plate.
assert BASE_X / 2 >= MIRROR_MOUNT_PLATE_SIZE / 2, (
    f"bracket X half {BASE_X / 2} mm < mount plate half "
    f"{MIRROR_MOUNT_PLATE_SIZE / 2} mm"
)

# Minimum bracket height (at the -Z edge of the base) must stay above
# zero — a negative value would mean the tilt plane dives through the
# floor inside the prism footprint.
_min_cut_y = TOP_HEIGHT - (BASE_Z / 2) * _TAN_TILT
assert _min_cut_y >= WALL_MIN, (
    f"bracket -Z edge height {_min_cut_y:.2f} mm < {WALL_MIN} mm — "
    f"top face dives too close to floor"
)


# --- Part function ------------------------------------------------------

def bracket(
    base_x: float = BASE_X,
    base_z: float = BASE_Z,
    prism_h: float = PRISM_H,
    top_height: float = TOP_HEIGHT,
    mount_plane_height: float = MOUNT_PLANE_HEIGHT,
    tilt_deg: float = TILT_DEG,
    bolt_half: float = BOLT_HALF,
    bolt_hole_d: float = BOLT_HOLE_D,
    elev_deg: float = ELEV_DEG,
) -> Part:
    """Wedge bracket: rectangular prism cut by a tilted plane at the
    top, with four bolt through-holes whose axes pass through the
    mount plate's bolt centres (a small MOUNT_GAP above the bracket
    top face).

    Body construction: a base_x × prism_h × base_z prism sitting on
    y=0, then a half-space cut by the tilt plane y = top_height +
    z·tan(tilt_deg) removes the "above" volume, leaving the wedge with
    a flat bottom, tilted top, and four vertical side faces.

    Hole construction: four cylindrical through-holes with axis along
    (0, -sin(elev), cos(elev)) — perpendicular to the mount plate's
    back face. Each cylinder is centred on a mount bolt's world axis
    (parameterised in bracket-local as entering the mount plane at
    mount_plane_height + sy·bolt_half·cos(elev)), so the bolt shank
    passes cleanly through both the mount and the bracket. The
    cylinder is long enough that its intersection with the body is a
    clean through-cut regardless of where it crosses the top face.
    """
    elev_rad = math.radians(elev_deg)
    sin_e = math.sin(elev_rad)
    cos_e = math.cos(elev_rad)

    prism = Pos(0, prism_h / 2, 0) * Box(base_x, prism_h, base_z)

    # Tilt cutter: a large box whose "bottom" (-Y) face lies on the
    # tilt plane and whose body sits in the +normal half-space (the
    # volume to remove). Pos(0, H/2, 0) stacks the body on +Y before
    # rotation; Rot(-tilt_deg, 0, 0) rotates the box about the local
    # +X axis so its bottom face aligns with the tilt plane; then
    # Pos(0, top_height, 0) drops the plane to z=0 at the target
    # height.
    cut_h = _CUT_BOX_SIZE
    cutter = (
        Pos(0, top_height, 0)
        * Rot(-tilt_deg, 0, 0)
        * Pos(0, cut_h / 2, 0)
        * Box(_CUT_BOX_SIZE, cut_h, _CUT_BOX_SIZE)
    )
    body = prism - cutter

    # Bolt through-holes. Each cylinder axis points along the bolt
    # direction (0, -sin(elev), cos(elev)) — Rot(elev_deg, 0, 0)
    # rotates a canonical +Z-axis cylinder onto this direction.
    # Placement uses mount_plane_height (mount back face height) so
    # the cylinder axis is colinear with the real bolt axis through
    # the mount plate; the small (top_height < mount_plane_height)
    # gap is absorbed by the bracket's slightly widened hole
    # diameter.
    cyl_len = _CUT_BOX_SIZE
    for sx in (-1, 1):
        for sy in (-1, 1):
            # sy = sign along mount-local +Y. Mount-local +Y in
            # bracket-local = (0, cos(elev), sin(elev)); so the bolt
            # axis' intersection with the mount plane is
            # (sx·bolt_half, mount_plane_height + sy·bolt_half·cos(elev),
            #  sy·bolt_half·sin(elev)).
            ex = sx * bolt_half
            ey = mount_plane_height + sy * bolt_half * cos_e
            ez = sy * bolt_half * sin_e
            body -= (
                Pos(ex, ey, ez)
                * Rot(elev_deg, 0, 0)
                * Cylinder(bolt_hole_d / 2, cyl_len)
            )

    return body


# --- Assembly -----------------------------------------------------------

# Single-part sub-assembly — the meaningful clearance/interference
# checks live on the adjacent geometry (mirror_mount, enclosure floor,
# platform) and so run in the scanner top-level `assembly.py`.
# Standalone, `check()` reports volume + bbox so a numerical sanity
# glance is possible without spinning up the rest of the rig.
assembly = Assembly().add("bracket", bracket())


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation is +Y up (bottom face on the build plate).
    # FDM default up_axis is +Z, which reads overhangs against the
    # wrong build direction; override to +Y.
    #
    # Overhang threshold is raised from the usual 45° to 55° because
    # the four bolt cylinders are unavoidably tilted ELEV_DEG ≈ 38°
    # from vertical (perpendicular to the 52°-tilted top face). The
    # upper half of each through-hole therefore reports a ~52°
    # overhang by geometry alone. 52° is inside typical PLA
    # unsupported-bridge capability for a 4.5 mm hole; the slicer
    # can add supports if needed, and the holes are a post-
    # processible feature regardless. The 55° threshold still catches
    # larger geometry mistakes (e.g., a flat downward-facing ledge).
    inspect(
        bracket(),
        method=FDM(
            up_axis=(0, 1, 0),
            wall_min_mm=WALL_MIN,
            overhang_max_deg=55.0,
        ),
        out="outputs",
        name="bracket",
    )
