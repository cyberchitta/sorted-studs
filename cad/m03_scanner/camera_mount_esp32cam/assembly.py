"""
M03 scanner — ESP32-CAM clamp + mount (Phase-2 integrated frame).

Single printed body per camera vertex: rail clamp base + beam + plate
that holds an AI-Thinker ESP32-CAM (M12 lens variant) by 4 corner M2
bolts. One `clamp_mount(...)` function parameterised by
`mount_in_clamp` covers all four poses; the rev-3 wedge variant
(corner clamp gripping rail C's -Y + rail D's -Z) was demolished in
Phase 2 along with rail C — back-upper is now a plain cantilever-
underside clamp like front-low.

Four printed instances drive all four cameras (one body per instance
because the mount-rotations differ):

  * front-low   — clamps onto rail D's -Z (underside) face
  * back-upper  — clamps onto rail D's -Z (underside) face
                  (Phase 2; was rail C's -Y face)
  * below-left  — clamps onto the shared back rail's +Z (top) face
                  (Phase 2; was rail A's +Z face — same Y, +1.2 mm
                  in Z)
  * below-right — X-mirror of below-left

The bodies differ in more than the plate's pose: the plate is shaped
per pose against the print direction (edge chamfers, a back wedge,
teardrop M2 bores) and, at back-upper, notched clear of a clamp bolt.
The clamp base — the rail interface: block, bolt spacing, bores — is
identical in all four, and `build_camera_rig` asserts it intact per
camera. Every body is inspected (`printability.py`); none is assumed
from another's symmetry.

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

Print orientation: clamp base on the build plate, +Y up (`PRINT_UP`).
`clamp_mount` shapes each plate so no face overhangs past
`PRINT_OVERHANG_MAX_DEG` in that orientation, and `printability.py`
inspects at the FDM default 45° with no waiver.

PCB hole pattern: dimensions are nominal-pending-verification. Online
sources disagree on the AI-Thinker ESP32-CAM mounting hole pitch
(~22 × 35 mm cited most commonly); calipers on a real board are the
gate before printing four for installation. M2 holes are oversized to
Ø2.4 mm so a ±0.3 mm pitch error doesn't require a reprint.
"""

import math

from build123d import (
    Box, Cylinder, Location, Part, Plane, Polyline, Pos, Rot, Vector,
    extrude, make_face,
)

from cad_khana.mechanism.assembly import Assembly


from ..params import (
    CAMERA_MOUNT_PLATE_T,
    RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
    RAIL_PAD_CLAMP_BACK_UPPER_POS,
    RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
    RAIL_PAD_CLAMP_BELOW_LEFT_POS,
    SKELETON_RAIL_MEMBER,
    VERTEX_BACK_UPPER_POS,
    VERTEX_BELOW_LEFT_POS,
    WALL_MIN,
    horizontal_x_roll_deg,
    lens_axis_unit,
    rotation_for_normal,
)


# --- AI-Thinker ESP32-CAM PCB geometry (private) ------------------------

# PCB outline (rectangular). Source: _notes/decisions/m03-cameras.md, pending
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

# M4 clamp bolt: ISO 4762 M4 SHCS, head Ø7 × 4. The bolt drops in
# vertically (clamp-local -Y) onto the base top, so it needs a clear
# drop-in corridor above its seat: Ø8 (head + 0.5 mm radial) tall
# enough to lower an M4×20 (15 mm base + ~5 mm T-nut engagement) plus
# its head in from above — which also covers an L-key's short arm
# seated in the head. The printed body must stand clear of that
# corridor by CLAMP_BOLT_ACCESS_MARGIN at every pose (a keep-out per
# bolt per camera in `build_camera_rig`). The camera board and lens are NOT in
# the corridor's claim: the clamp is bolted to the rail before the
# ESP32-CAM goes on its plate.
CLAMP_BOLT_L = 20.0
CLAMP_BOLT_HEAD_H = 4.0
CLAMP_BOLT_ACCESS_D = 8.0
CLAMP_BOLT_ACCESS_H = CLAMP_BOLT_L + CLAMP_BOLT_HEAD_H
CLAMP_BOLT_ACCESS_MARGIN = 0.5


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
    + length derivation and for `clamp_mount`'s pose sanity asserts."""
    p = (mount_in_clamp * Pos(0, 0, -CAMERA_MOUNT_PLATE_T)).position
    return (p.X, p.Y, p.Z)


# Below-left pose — the standalone's canonical instance and
# `clamp_mount`'s default. The back-upper pose used to be evaluated
# here too, purely to feed a module-level two-variant assert loop;
# that check now lives inside `clamp_mount` over its `mount_in_clamp`
# argument, so every instance the m03 root builds is covered and the
# hardcoded second variant is no longer needed.
_MOUNT_IN_CLAMP: Location = mount_in_clamp_for(
    RAIL_PAD_CLAMP_BELOW_LEFT_POS,
    RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
    VERTEX_BELOW_LEFT_POS,
)


# --- Print orientation ------------------------------------------------
#
# Every instance prints clamp base on the bed, clamp-local +Y up — the
# only orientation with a large flat bed face that keeps all four
# bodies near-clean (orientation audit 2026-09-27: every axis-aligned
# and plate-aligned alternative leaves ≥ 58 mm² past 45°, most
# thousands). The plate's pose differs per camera, so which plate faces
# overhang differs too; `clamp_mount` shapes the plate against this
# direction instead of the printability run waiving what it finds.
# `printability.py` inspects with this same `PRINT_UP`.
PRINT_UP = (0.0, 1.0, 0.0)
# Half a degree inside the FDM check's 45°, so a face designed onto the
# limit can't read over it.
PRINT_OVERHANG_MAX_DEG = 44.5


def _print_up_in_mount(
    mount_in_clamp: Location, print_up: tuple[float, float, float]
) -> Vector:
    """`print_up` (clamp-local) as a mount-local unit vector."""
    inv = mount_in_clamp.inverse()
    v = (inv * Pos(*print_up)).position - inv.position
    return v.normalized()


def _mount_plate(
    up_m: Vector, plate_size: float, plate_t: float, overhang_max_deg: float,
) -> Part:
    """The mount plate in mount-local (z ∈ [-plate_t, 0], front face at
    z = 0), shaped so no face overhangs past `overhang_max_deg` when
    printed with mount-local `up_m` up. Only the edges and the back
    move; the front (PCB seat) face, the M2 corner lands and the
    footprint the PCB covers are never cut.

    * An edge face whose outward normal points too far down is
      chamfered off its front corner: a plane through the edge's back
      corner line, its normal tipped toward +z just enough to reach the
      limit (front-low's low edge 65° → 44.5°, trimming ~2 mm of seat
      face; below-*'s 55.4° → ~1 mm).
    * A back face that overhangs (back-upper's, 45.5°) is backed by a
      wedge: a new back plane tipped toward the plate's uphill direction
      through the uphill back corner, so the plate only thickens (by
      ~0.9 mm at its downhill edge), never thins below `plate_t`."""
    s = math.sin(math.radians(overhang_max_deg))
    half = plate_size / 2
    plate = Pos(0, 0, -plate_t / 2) * Box(plate_size, plate_size, plate_t)

    # Back wedge. Back-face normal is -z; it overhangs by asin(up_m.Z).
    up_perp = Vector(up_m.X, up_m.Y, 0)
    if up_m.Z > s and up_perp.length > 1e-9:
        g = up_perp.normalized()
        alpha = math.asin(min(1.0, up_m.Z))
        psi = alpha - math.radians(overhang_max_deg)
        n = Vector(0, 0, -1) * math.cos(psi) + g * math.sin(psi)
        c = max(
            n.dot(Vector(sx * half, sy * half, -plate_t))
            for sx in (-1, 1) for sy in (-1, 1)
        )
        slab = Pos(0, 0, -plate_t - half / 2) * Box(plate_size, plate_size, half)
        beyond = (
            Plane(origin=tuple(n * c), z_dir=tuple(n)).location
            * Pos(0, 0, 500)
            * Box(1000, 1000, 1000)
        )
        plate = plate + (slab - beyond)

    # Edge chamfers, each off its front corner.
    for e, along in (
        (Vector(1, 0, 0), Vector(0, 1, 0)),
        (Vector(-1, 0, 0), Vector(0, 1, 0)),
        (Vector(0, 1, 0), Vector(1, 0, 0)),
        (Vector(0, -1, 0), Vector(1, 0, 0)),
    ):
        a = -e.dot(up_m)
        if a <= s:
            continue
        r = math.hypot(a, up_m.Z)
        assert up_m.Z > 0 and s < r, (
            f"edge {tuple(e)} overhangs {math.degrees(math.asin(a)):.1f}° "
            f"and a front-corner chamfer can't reach {overhang_max_deg}°"
        )
        phi = math.acos(s / r) - math.atan2(up_m.Z, a)
        width = plate_t * math.tan(phi)
        # The chamfer may trim flange only: it must stop short of the
        # PCB outline on the seat face.
        pcb_half = PCB_W / 2 if abs(e.X) > 0.5 else PCB_H / 2
        assert half - width > pcb_half + WALL_MIN, (
            f"edge chamfer {width:.2f} mm reaches within "
            f"{half - width - pcb_half:.2f} mm of the PCB outline"
        )
        # Triangle (edge back corner, edge front corner, point `width`
        # in along the front face), overshot past the edge and the
        # front face so no face is left coplanar.
        pts = [
            e * (half + 1) + Vector(0, 0, -plate_t - 1e-3),
            e * half + Vector(0, 0, -plate_t),
            e * (half - width) + Vector(0, 0, 0),
            e * (half - width) + Vector(0, 0, 1),
            e * (half + 1) + Vector(0, 0, 1),
        ]
        prism = extrude(
            make_face(Polyline(*[tuple(p) for p in pts], close=True)),
            amount=plate_size + 2, dir=tuple(along),
        )
        plate = plate - Pos(*(along * -(half + 1))) * prism
    return plate


def _plate_edge_notch(
    intrusion: Part, mount_in_clamp: Location, plate_size: float,
) -> Part | None:
    """A mount-local slot through the plate that removes `intrusion`
    (plate material inside a corridor, clamp-local), opened to the
    plate edge nearest it, full plate depth (back wedge included). None
    when there is nothing to remove. Asserts the slot stays outside the
    PCB footprint (+ WALL_MIN), which also keeps it off the M2 corners."""
    if intrusion.volume < 1e-6:
        return None
    half = plate_size / 2
    bb = (mount_in_clamp.inverse() * intrusion).bounding_box()
    lo, hi = bb.min, bb.max
    reach = {
        "+y": half - hi.Y, "-y": lo.Y + half,
        "+x": half - hi.X, "-x": lo.X + half,
    }
    side = min(reach, key=reach.get)
    x0, x1, y0, y1 = lo.X, hi.X, lo.Y, hi.Y
    if side == "+y":
        y1 = half + 1
        clear = y0 - PCB_H / 2
    elif side == "-y":
        y0 = -half - 1
        clear = -y1 - PCB_H / 2
    elif side == "+x":
        x1 = half + 1
        clear = x0 - PCB_W / 2
    else:
        x0 = -half - 1
        clear = -x1 - PCB_W / 2
    assert clear >= WALL_MIN, (
        f"clamp-bolt corridor notch ({side} edge) comes within "
        f"{clear:.2f} mm of the PCB footprint; must be ≥ {WALL_MIN}"
    )
    z0, z1 = -half, 1.0          # through the plate and any back wedge
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        x1 - x0, y1 - y0, z1 - z0
    )


def _teardrop_bore(r: float, length: float, up_m: Vector) -> Part:
    """A bore along mount-local z, centred on the origin, with a 45°
    teardrop roof pointing at the in-plane component of `up_m`. The
    roof faces overhang asin(|up in-plane| / √2) ≤ 45° — within the
    check at any lens-axis tilt (front-low 40°, below-* 35.6°). Round
    when the axis is near-vertical."""
    bore = Cylinder(r, length)
    d = Vector(up_m.X, up_m.Y, 0)
    if d.length < 1e-6:
        return bore
    d = d.normalized()
    ang = math.degrees(math.atan2(d.Y, d.X))
    roof = (
        Pos(*(d * (r / math.sqrt(2))))
        * Rot(0, 0, ang + 45)
        * Box(r, r, length)
    )
    return bore + roof


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
    print_up: tuple[float, float, float] = PRINT_UP,
    overhang_max_deg: float = PRINT_OVERHANG_MAX_DEG,
) -> Part:
    """Single printed body: clamp base (on rail) + beam + 50 × 50 ×
    plate_t mount plate. ESP32-CAM PCB bolts to the plate's front face
    at mount-local z = 0 via four M2 screws on the corner pattern.

    `mount_in_clamp` supplies the mount-local → clamp-local Location;
    defaults to the below-left pose. Beam tilt + axial length are
    derived from this Location, so all four rev-5 instances build from
    the same function. The plate's pose then drives its print shaping
    against `print_up` (clamp-local): `_mount_plate`'s edge chamfers
    and back wedge, the teardrop direction of the M2 bores, and any
    `_plate_edge_notch` that clears a clamp bolt's drop-in corridor.
    None of these touches the clamp base, the plate's front (PCB seat)
    face or its M2 corner lands — so the camera's pose, the rail
    interface and the board seat are the same at every instance."""
    _mb_x, _mb_y, _mb_z = _mount_back_in_clamp(mount_in_clamp)
    # Pose sanity for the supplied `mount_in_clamp`: the mount back
    # must clear the clamp base in +Y and sit on the clamp's YZ axis
    # in X/Z, or the derived beam tilt below is meaningless. This
    # replaces a module-level loop over two hardcoded variants —
    # checking the argument covers every instance actually built,
    # which is all four cameras (the m03 root passes each its own
    # pose) rather than the two the loop happened to name.
    assert _mb_y > base_h, (
        f"mount-back in clamp-local +Y must clear the clamp base "
        f"(base_h = {base_h}); got Y={_mb_y:.1f}"
    )
    assert abs(_mb_x) < 2.0, (
        f"mount-back in clamp-local +X should sit near 0; got X={_mb_x:.2f}"
    )
    assert abs(_mb_z) < 2.0, (
        f"mount-back in clamp-local +Z should sit near 0; got Z={_mb_z:.2f}"
    )
    # Bolt-hole and flange wall sanity for this body — all WALL_MIN
    # arithmetic over subtracted negative space, so none is a part-pair
    # claim. Over the arguments, so overrides are guarded too.
    assert bolt_half > beam_w / 2 + WALL_MIN, (
        f"clamp bolt half-spacing {bolt_half:.1f} mm leaves "
        f"{bolt_half - beam_w / 2:.1f} mm wall to the beam edge; "
        f"must be ≥ {WALL_MIN} mm"
    )
    _edge_x = base_l / 2 - bolt_half - clamp_hole_d / 2
    assert _edge_x >= WALL_MIN, (
        f"clamp bolt-to-base-edge wall {_edge_x:.2f} mm < {WALL_MIN} mm"
    )
    # Plate must be at least as big as the PCB, with a flange margin
    # around both the PCB outline and the M2 corner holes.
    assert plate_size >= PCB_W and plate_size >= PCB_H, (
        f"plate {plate_size} smaller than PCB ({PCB_W} × {PCB_H})"
    )
    assert (plate_size - PCB_W) / 2 >= WALL_MIN, (
        f"plate-PCB long-edge flange {(plate_size - PCB_W) / 2:.2f} mm "
        f"< {WALL_MIN} mm"
    )
    _m2_to_edge_x = plate_size / 2 - pcb_hole_pitch_x / 2 - pcb_hole_d / 2
    _m2_to_edge_y = plate_size / 2 - pcb_hole_pitch_y / 2 - pcb_hole_d / 2
    assert _m2_to_edge_x >= WALL_MIN, (
        f"M2-to-plate-edge wall (long axis) {_m2_to_edge_x:.2f} mm < {WALL_MIN} mm"
    )
    assert _m2_to_edge_y >= WALL_MIN, (
        f"M2-to-plate-edge wall (short axis) {_m2_to_edge_y:.2f} mm < {WALL_MIN} mm"
    )

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
    # Plate occupies mount-local z ∈ [-plate_t, 0]; front face at z = 0
    # is the PCB rest plane. Built whole in mount-local first — its
    # print-driven edge chamfers and back wedge (see `_mount_plate`)
    # belong to the plate, so the beam, unioned after, keeps its own
    # faces where it passes the plate's edge.
    up_m = _print_up_in_mount(mount_in_clamp, print_up)
    plate = mount_in_clamp * _mount_plate(
        up_m, plate_size, plate_t, overhang_max_deg,
    )
    body = body + plate

    # Trim everything past the plate front face (mount-local z > 0).
    # The beam overshoots into the plate so the union is one continuous
    # solid; cutting at the front face leaves a clean flat PCB-seating
    # surface. The clamp base block is exempt: the halfspace is sized
    # for the plate, and unbounded it sliced the rail interface at
    # three of the four poses (below-*: 3108 mm³ off the base's outer
    # end, the M4 head left no seat and a 0.70 mm wall at its bore;
    # back-upper: 515 mm³ off its -x end).
    _CUT_BOX_SIZE = 1000.0
    front_cutter = mount_in_clamp * (
        Pos(0, 0, _CUT_BOX_SIZE / 2)
        * Box(_CUT_BOX_SIZE, _CUT_BOX_SIZE, _CUT_BOX_SIZE)
    )
    body = body - (front_cutter - clamp_base_block(base_l, base_w, base_h))

    # --- 4. Rail clamp bolts (vertical through base, axis along clamp -Y) ---
    for sx in (-1, 1):
        body -= (
            Pos(sx * bolt_half, base_h / 2, 0)
            * Rot(90, 0, 0)
            * Cylinder(clamp_hole_d / 2, base_h * 3)
        )

    # --- 5. Clamp-bolt drop-in corridors ---
    # Where the plate reaches into an M4's drop-in corridor (Ø8 head
    # envelope, `CLAMP_BOLT_ACCESS_H` above its seat, grown by
    # `CLAMP_BOLT_ACCESS_MARGIN`), the plate is notched from its nearest
    # edge: a mount-aligned slot, so its faces meet the plate's at 90°
    # (a cylindrical relief through a 45° plate leaves a knife-edge —
    # measured 0.96 mm at alignment 0.18). Only back-upper's plate
    # reaches one: its low edge lies across the -x bolt, which could
    # not be dropped in or seated. The beam and the base block are
    # exempt — the beam already clears every corridor, and the base
    # top is the bolt's seat.
    for sx in (-1, 1):
        corridor = (
            Pos(sx * bolt_half, base_h + CLAMP_BOLT_ACCESS_H / 2, 0)
            * Rot(90, 0, 0)
            * Cylinder(
                CLAMP_BOLT_ACCESS_D / 2 + CLAMP_BOLT_ACCESS_MARGIN,
                CLAMP_BOLT_ACCESS_H,
            )
        )
        notch = _plate_edge_notch(corridor & plate, mount_in_clamp, plate_size)
        if notch is not None:
            body -= mount_in_clamp * notch - beam - clamp_base_block(
                base_l, base_w, base_h
            )

    # --- 6. PCB-mount M2 holes (along mount-local +Z, lens axis) ---
    # Corner pattern on a 35 × 22 mm rectangle. Holes drilled through
    # the plate; long bore (200 mm) matches the rev-4 convention. Each
    # is a teardrop pointed at print-up: the lens axis leans 25-45°
    # off horizontal at these poses, so a round Ø2.4 bore's crown
    # faces down at up to 65°.
    _hole_len = 200.0
    cx = pcb_hole_pitch_x / 2
    cy = pcb_hole_pitch_y / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            body -= mount_in_clamp * (
                Pos(sx * cx, sy * cy, 0)
                * _teardrop_bore(pcb_hole_d / 2, _hole_len, up_m)
            )

    return body


def clamp_base_block(
    base_l: float = BASE_L, base_w: float = BASE_W, base_h: float = BASE_H,
) -> Part:
    """The clamp base as designed — the rail interface block, before
    bores. Clamp-local, on the rail face at y = 0."""
    return Pos(0, base_h / 2, 0) * Box(base_l, base_h, base_w)


def clamp_bolt_keepout(
    sx: int, bolt_half: float = _CLAMP_BOLT_HALF, base_h: float = BASE_H,
    margin: float = CLAMP_BOLT_ACCESS_MARGIN,
) -> Part:
    """The Ø`CLAMP_BOLT_ACCESS_D` drop-in corridor above the `sx` clamp
    bolt's seat (the base top), `CLAMP_BOLT_ACCESS_H` tall, grown by
    `margin` radially and at the top so the clamp can be held clear of
    it at 0 while it stands on the seat. Clamp-local; a keep-out for
    `assert_clear_of`, never placed in an assembly."""
    h = CLAMP_BOLT_ACCESS_H + margin
    return (
        Pos(sx * bolt_half, base_h + h / 2, 0)
        * Rot(90, 0, 0)
        * Cylinder(CLAMP_BOLT_ACCESS_D / 2 + margin, h)
    )


def clamp_base_shortfall_mm3(body: Part) -> float:
    """Clamp-base material the body lacks beyond its two bolt bores —
    0 when the rail interface block is intact. A cut sized for the
    plate that reaches into the base (the unbounded front trim did,
    until 2026-09-27) shows up here."""
    block = clamp_base_block()
    bores = [
        Pos(sx * _CLAMP_BOLT_HALF, BASE_H / 2, 0)
        * Rot(90, 0, 0)
        * Cylinder(CLAMP_BOLT_HOLE_D / 2, BASE_H)
        for sx in (-1, 1)
    ]
    drilled = block - bores[0] - bores[1]
    return drilled.volume - (body & block).volume


# --- Part functions (continued) -----------------------------------------

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
    # The lens body must clear the M2 screw heads on the PCB front
    # face. M2 corner holes sit at radial distance
    # √((PITCH_X/2)² + (PITCH_Y/2)²) from the PCB centre; the lens
    # cylinder is centred on origin. ~Ø4 mm typical M2 pan-head
    # footprint. The screw heads are not modelled, so this is a bare
    # parametric bound rather than a part-pair claim.
    _m2_radial = math.hypot(PCB_HOLE_PITCH_X / 2, PCB_HOLE_PITCH_Y / 2)
    _lens_to_m2 = _m2_radial - od / 2
    assert _lens_to_m2 >= 2.0, (
        f"lens body radial gap to M2 hole centre {_lens_to_m2:.2f} mm "
        f"leaves no room for an M2 screw head"
    )

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
    .with_part("clamp_mount", clamp_mount())
    .with_part("pcb", camera_pcb_stub(), location=_MOUNT_IN_CLAMP)
    .with_part("lens", m12_lens_stub(), location=_MOUNT_IN_CLAMP)
    .assert_no_interference("clamp_mount", "pcb")
    .assert_no_interference("clamp_mount", "lens")
    .assert_no_interference("pcb", "lens")
)
