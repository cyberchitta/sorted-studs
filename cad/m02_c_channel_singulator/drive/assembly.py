"""
M02 C-channel singulator — drive sub-assembly (rim gear on a rim bearing).

The rotor is supported at its rim and driven at its rim
(`_notes/decisions/m02-rotor-drive.md`, 2026-09-11). Everything here
hangs below the V-bottom plane, under the rotor's ring-gear flange:

  - bowl_0..2  — printed, in thirds (`../seams.py`): a floor annulus, r ∈ [PLATE_R_IN, PLATE_R_OUT],
                 with a cylindrical skirt on its rim rising to
                 SKIRT_TOP_Z (`m02-chain.md` §5, 2026-09-16). The floor
                 is the stator foot ring extended inward: the turntable
                 bearing sits on its top face, the motor bolts to its
                 underside through a pilot hole + four M3 clearance
                 holes at the NEMA-17 pattern. The stator wall is a
                 liner resting on the skirt's top edge. The skirt is
                 slotted through the exit arc for the stator's tongue,
                 down to SKIRT_CUT_BOTTOM_Z; below that a stub of skirt
                 keeps the ring beam closed. Split at three radial
                 seams (phase per instance, clear of the exit arc and
                 the motor — asserted in ``build_drive``), each sector
                 carrying a bolted ear at both seams outside the skirt.
                 On a stage whose liner the exit opening cuts through,
                 the skirt beside each side of the slot is drilled
                 (teardrop, countersunk) for the screw fastening the
                 liner's free end (``../arc_fastening.py``).
  - bearing    — bought aluminium turntable, modelled as one annulus
                 (BEARING_ID / OD / H). Upper ring bolts to the rotor
                 flange, lower ring to the plate; both bolt circles
                 are detail-layer.
  - pinion     — printed; pitch cylinder (no teeth, like the m05 base
                 floor) on a hub that reaches down to just above the
                 plate, drilled radially for an M3 set screw
                 (``set_screw``, bought, in the same sub-assembly so it
                 turns with the hub). Bored for the 5 mm motor shaft.
                 Its own sub-assembly with a spin joint about its axis.
  - motor      — NEMA 17 stub: body + pilot boss + shaft, front face
                 up against the plate's underside, its four tapped
                 holes taking ``motor_screw_0..3`` (bought ISO 4762 M3)
                 down through the floor's counterbores.

Coordinate frame:
    origin = rotor axis ∩ rotor base plane (shared with rotor/)
    +X     = exit-arc azimuth (the pinion sits at MOTOR_AZIMUTH_DEG
             = 180°, the -X side, away from the exit)
    +Z     = up
    z=0    = rotor base / stator V-bottom. Everything here is below.

Claims declared here (they propagate to the module root): bearing
and motor rest on the floor (tangent contact), the pinion hub clears
the floor, the pinion bore clears the shaft, and the pinion's tip
circle stays inside the skirt — measured radially to a datum plane on
the skirt's inner face, because the plain distance pinion↔bowl is the
0.5 mm floor gap, not the radial one. The set screw's point bears on
the shaft; each motor screw bears on its counterbore and the motor's
tapped hole, and its key's corridor is held clear of the drive's own
parts (``_with_motor_screws`` says why only those). Rotor-vs-drive and
stator-vs-drive claims live in
../assembly.py.

Printed parts: the three bowl sectors (floor down, no support) — inspected
per stage in `cad/m02_chain/printability.py`, since each stage cuts its
own — and the pinion, in `printability.py`. The bearing and motor are
bought.
"""

import math
from functools import cache
from typing import cast

from bd_warehouse.fastener import SetScrew, SocketHeadCapScrew
from build123d import Align, Axis, Box, Circle, Compound, Cylinder, Location, Part, Plane, Polygon, Pos, Rot, extrude
from cad_khana.mechanism.assembly import Assembly, RevoluteJoint

from .. import arc_fastening as _fastening
from ..materials import material_of
from ..params import (
    ADDENDUM,
    BEARING_BOTTOM_Z,
    BEARING_H,
    BEARING_ID,
    BEARING_OD,
    BOWL_SEAM_PHASE_DEG,
    CENTER_DIST,
    EXIT_ARC_DEG,
    GEAR_RATIO,
    HOLE_ROOF_DEG,
    MOTOR_AZIMUTH_DEG,
    MOTOR_BOTTOM_Z,
    MOTOR_FACE_Z,
    MOTOR_SCREW_L,
    MOTOR_SCREW_SIZE,
    MOTOR_SCREW_TOOL_D,
    MOTOR_SCREW_TOOL_L,
    MOTOR_SCREW_TOOL_MARGIN,
    MOTOR_SHAFT_BORE_CLEARANCE,
    NEMA17_BODY_LEN,
    NEMA17_BOLT_CBORE_D,
    NEMA17_BOLT_CBORE_DEPTH,
    NEMA17_BOLT_HOLE_D,
    NEMA17_BOLT_PATTERN,
    NEMA17_BOLT_TAP_DEPTH,
    NEMA17_PILOT_D,
    NEMA17_PILOT_H,
    NEMA17_SHAFT_D,
    NEMA17_SHAFT_LEN,
    NEMA17_SIDE,
    PINION_BOTTOM_Z,
    PINION_FACE_W,
    PINION_HUB_BOTTOM_Z,
    PINION_HUB_OD,
    PINION_HUB_PLATE_CLEAR,
    PINION_PITCH_D,
    PINION_TOP_Z,
    PLATE_BOTTOM_Z,
    PLATE_MOTOR_HOLE_D,
    PLATE_R_IN,
    PLATE_R_OUT,
    PLATE_T,
    PLATE_TOP_Z,
    SET_SCREW_L,
    SET_SCREW_RECESS,
    SET_SCREW_SIZE,
    SKIRT_CUT_BOTTOM_Z,
    SKIRT_R_IN,
    SKIRT_TOP_Z,
)
from ..seams import SEAM_COUNT, SEAM_EAR_T, angular_gap_deg, seam_azimuths, sector_index, split_ring, with_seam_bolts
from ..teardrop import teardrop_cutter

WALL_MIN = 1.5

# Pinion / motor axis in drive-local XY.
_AZ = math.radians(MOTOR_AZIMUTH_DEG)
PINION_AXIS_XY = (CENTER_DIST * math.cos(_AZ), CENTER_DIST * math.sin(_AZ))

# The motor screws' corners on the pattern square, motor-local.
_CORNERS = ((1, 1), (1, -1), (-1, 1), (-1, -1))


def _column(d: float, h: float, at: Location) -> Part:
    """A cylinder of diameter ``d`` rising ``h`` from its base at ``at``."""
    return Cylinder(d / 2.0, h, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(at)


# --- Bought hardware (bd_warehouse) -----------------------------------


@cache
def _motor_screw() -> SocketHeadCapScrew:
    return SocketHeadCapScrew(size=MOTOR_SCREW_SIZE, length=MOTOR_SCREW_L, fastener_type="iso4762", simple=True)


@cache
def _set_screw() -> SetScrew:
    return SetScrew(size=SET_SCREW_SIZE, length=SET_SCREW_L, fastener_type="iso4026", simple=True)


# The set screw's hole in the hub: its tap drill for a soft material,
# threaded in the print; and its axis height, mid-hub.
_TAP_HOLES = cast("dict[str, float]", _set_screw().tap_hole_diameters)  # bd_warehouse leaves the table untyped
SET_SCREW_HOLE_D = _TAP_HOLES["Soft"]
SET_SCREW_Z = (PINION_HUB_BOTTOM_Z + PINION_BOTTOM_Z) / 2.0


# --- Part functions ----------------------------------------------------


def _skirt(*, r_out: float, skirt_r_in: float, skirt_top_z: float) -> Part:
    """The bowl's skirt ring, from the floor top to ``skirt_top_z``."""
    return Pos(0, 0, PLATE_TOP_Z) * extrude(
        Circle(r_out) - Circle(skirt_r_in),
        amount=skirt_top_z - PLATE_TOP_Z,
    )


def _motor_holes(
    *,
    t: float,
    motor_xy: tuple[float, float],
    motor_rot_deg: float,
    pilot_hole_d: float,
    bolt_pattern: float,
    bolt_hole_d: float,
    bolt_cbore_d: float,
    bolt_cbore_depth: float,
    skirt_top_z: float,
) -> Part:
    """The motor's floor cutters, placed: the pilot hole, the four M3
    holes, and each counterbore carried from its floor up past
    ``skirt_top_z`` — the column the screw's head comes down, which
    grooves the skirt's inner face where an outboard one reaches past
    it (owner, 2026-11-09)."""
    half = bolt_pattern / 2.0
    cut_h = t + 2.0
    cbore_z0 = t / 2.0 - bolt_cbore_depth
    cbore_h = skirt_top_z + 1.0 - (PLATE_BOTTOM_Z + t / 2.0) - cbore_z0
    holes = Cylinder(pilot_hole_d / 2.0, cut_h)
    for sx, sy in _CORNERS:
        holes += Pos(sx * half, sy * half, 0) * Cylinder(bolt_hole_d / 2.0, cut_h)
        holes += _column(bolt_cbore_d, cbore_h, Pos(sx * half, sy * half, cbore_z0))
    place = Pos(motor_xy[0], motor_xy[1], PLATE_BOTTOM_Z + t / 2.0) * Rot(0, 0, motor_rot_deg)
    return place * holes


def bowl(
    *,
    r_in: float = PLATE_R_IN,
    r_out: float = PLATE_R_OUT,
    t: float = PLATE_T,
    skirt_r_in: float = SKIRT_R_IN,
    skirt_top_z: float = SKIRT_TOP_Z,
    skirt_cut_bottom_z: float = SKIRT_CUT_BOTTOM_Z,
    exit_arc_deg: float = EXIT_ARC_DEG,
    motor_xy: tuple[float, float] = PINION_AXIS_XY,
    motor_rot_deg: float = MOTOR_AZIMUTH_DEG,
    pilot_hole_d: float = PLATE_MOTOR_HOLE_D,
    bolt_pattern: float = NEMA17_BOLT_PATTERN,
    bolt_hole_d: float = NEMA17_BOLT_HOLE_D,
    bolt_cbore_d: float = NEMA17_BOLT_CBORE_D,
    bolt_cbore_depth: float = NEMA17_BOLT_CBORE_DEPTH,
    arc_screw_holes: bool = True,
) -> Part:
    """Floor annulus (top face at z=PLATE_TOP_Z) with the motor's pilot
    hole and M3 pattern cut through at ``motor_xy`` (pattern square
    aligned radially by ``motor_rot_deg``, heads counterbored flush
    from the top), and a skirt on its rim
    from the floor top to ``skirt_top_z``, slotted through the exit
    arc from ``skirt_cut_bottom_z`` up for the stator's tongue.

    With ``arc_screw_holes`` the skirt also carries, beside each side
    of the slot, the countersunk hole for the screw that fastens the
    liner's free end there (``../arc_fastening.py``) — the unit root
    passes it on the stages whose liner the exit opening cuts through.

    Prints floor-down: the skirt is a plain vertical wall and the
    slot's faces are vertical, so nothing overhangs; the screw holes
    are teardrops roofed toward +Z.
    """
    assert r_in < r_out and t > 0.0
    assert skirt_r_in < r_out and skirt_top_z > PLATE_TOP_Z, "skirt must stand on the floor's rim"
    assert PLATE_TOP_Z < skirt_cut_bottom_z < skirt_top_z, "the tongue slot must stay inside the skirt"
    assert r_out - skirt_r_in >= WALL_MIN, "skirt wall too thin"
    # Bolt holes must stay clear of the pilot hole and of each other.
    assert bolt_pattern / 2.0 - bolt_hole_d / 2.0 - pilot_hole_d / 2.0 >= WALL_MIN, (
        "motor bolt holes break into the pilot hole"
    )
    # The pattern's outboard corners (radially aligned square: corner
    # at (+half, ±half) from the axis) must keep the through-hole
    # inside the skirt's inner face, and the counterbore's column —
    # which may groove the skirt's inner face (see params
    # NEMA17_BOLT_CBORE_D) — a wall inside the floor's and skirt's
    # outer face.
    half = bolt_pattern / 2.0
    _corner_r = math.hypot(math.hypot(*motor_xy) + half, half)
    assert _corner_r + bolt_hole_d / 2.0 <= skirt_r_in, (
        f"motor bolt hole (edge r {_corner_r + bolt_hole_d / 2.0:.2f}) runs under the skirt (r {skirt_r_in})"
    )
    assert _corner_r + bolt_cbore_d / 2.0 + WALL_MIN <= r_out, "motor bolt counterbore breaks out of the floor's rim"
    assert 0.0 < bolt_cbore_depth <= t - WALL_MIN, "counterbore leaves no floor under the head"
    plate = Pos(0, 0, PLATE_BOTTOM_Z) * extrude(
        Circle(r_out) - Circle(r_in),
        amount=t,
    )
    skirt = _skirt(r_out=r_out, skirt_r_in=skirt_r_in, skirt_top_z=skirt_top_z)
    # Exit-arc slot through the skirt — the same isoceles cutter the
    # stator wall uses (duplicated, not imported: cad/CLAUDE.md
    # principle 2), from the slot bottom up past the skirt's top.
    h = math.radians(exit_arc_deg / 2.0)
    r_cut = (r_out + 10.0) / math.cos(h)
    wedge_sketch = Polygon(
        (0.0, 0.0),
        (r_cut * math.cos(-h), r_cut * math.sin(-h)),
        (r_cut * math.cos(+h), r_cut * math.sin(+h)),
        align=None,
    )
    wedge = Pos(0, 0, skirt_cut_bottom_z) * extrude(
        wedge_sketch,
        amount=skirt_top_z - skirt_cut_bottom_z + 1.0,
    )
    holes = _motor_holes(
        t=t,
        motor_xy=motor_xy,
        motor_rot_deg=motor_rot_deg,
        pilot_hole_d=pilot_hole_d,
        bolt_pattern=bolt_pattern,
        bolt_hole_d=bolt_hole_d,
        bolt_cbore_d=bolt_cbore_d,
        bolt_cbore_depth=bolt_cbore_depth,
        skirt_top_z=skirt_top_z,
    )
    # Holes after the skirt: the two outboard counterbores reach 0.26
    # past its inner face, so their columns groove it top to bottom and
    # a head comes straight down (`_with_motor_screws`' entry claim).
    # Cut before it, the skirt's foot bridged their rims instead and
    # stood in the head's way.
    body = ((plate + skirt) - holes) - wedge
    if arc_screw_holes:
        for sign in _fastening.FREE_END_SIGNS:
            body = body - _fastening.skirt_hole(sign)
    return body


# The bowl's ears run the full height of floor and skirt.
BOWL_EAR_Z = (PLATE_BOTTOM_Z, SKIRT_TOP_Z)


def bowl_sectors(
    *,
    phase_deg: float = BOWL_SEAM_PHASE_DEG,
    motor_azimuth_deg: float = MOTOR_AZIMUTH_DEG,
    arc_screw_holes: bool = True,
) -> list[Part]:
    """The bowl for a motor at ``motor_azimuth_deg``, cut into
    SEAM_COUNT sectors at ``phase_deg`` with ears the full height of
    floor and skirt, the skirt drilled for the liner's free-end screws
    when ``arc_screw_holes``. Sector k is ``bowl_k``."""
    az = math.radians(motor_azimuth_deg)
    whole = bowl(
        motor_xy=(CENTER_DIST * math.cos(az), CENTER_DIST * math.sin(az)),
        motor_rot_deg=motor_azimuth_deg,
        arc_screw_holes=arc_screw_holes,
    )
    return split_ring(
        whole,
        phase_deg=phase_deg,
        r_out=PLATE_R_OUT,
        ear_z=BOWL_EAR_Z,
        hole_apex_up=True,  # printed floor down
    )


def bowl_part(k: int) -> str:
    """Leaf name of bowl sector k."""
    return f"bowl_{k}"


BOWL_PARTS = tuple(bowl_part(k) for k in range(SEAM_COUNT))


def bowl_part_at(az_deg: float, phase_deg: float = BOWL_SEAM_PHASE_DEG) -> str:
    """Leaf name of the bowl sector under m02-local azimuth ``az_deg``."""
    return bowl_part(sector_index(az_deg, phase_deg))


def bearing(
    *,
    od: float = BEARING_OD,
    id_: float = BEARING_ID,
    h: float = BEARING_H,
) -> Part:
    """Turntable bearing stub: one annulus, bottom face at
    z=BEARING_BOTTOM_Z. Bought — not inspected."""
    assert 0.0 < id_ < od
    return Pos(0, 0, BEARING_BOTTOM_Z) * extrude(
        Circle(od / 2.0) - Circle(id_ / 2.0),
        amount=h,
    )


def pinion(
    *,
    pitch_d: float = PINION_PITCH_D,
    face_w: float = PINION_FACE_W,
    hub_od: float = PINION_HUB_OD,
    bore_d: float = NEMA17_SHAFT_D + 2.0 * MOTOR_SHAFT_BORE_CLEARANCE,
) -> Part:
    """Pitch cylinder z ∈ [PINION_BOTTOM_Z, PINION_TOP_Z] on a hub
    reaching down to PINION_HUB_BOTTOM_Z, bored through for the motor
    shaft, and drilled radially along +X at SET_SCREW_Z for the set
    screw: a teardrop over SET_SCREW_HOLE_D roofed toward the print's
    up (−Z: it prints gear-face down). Built on its own axis
    (pinion-local Z), so the spin joint is the local Z axis."""
    assert bore_d < hub_od - 2.0 * WALL_MIN, "hub wall too thin around the bore"
    assert hub_od < pitch_d - 2.0 * ADDENDUM * 1.25, "hub wider than the root circle"
    hub_h = PINION_BOTTOM_Z - PINION_HUB_BOTTOM_Z
    assert SET_SCREW_HOLE_D + 2.0 * WALL_MIN <= hub_h, "set screw hole leaves no hub above or below it"
    gear = Pos(0, 0, PINION_BOTTOM_Z) * extrude(Circle(pitch_d / 2.0), amount=face_w)
    hub = Pos(0, 0, PINION_HUB_BOTTOM_Z) * extrude(Circle(hub_od / 2.0), amount=hub_h)
    total_h = PINION_TOP_Z - PINION_HUB_BOTTOM_Z
    bore = Pos(0, 0, PINION_HUB_BOTTOM_Z - 1.0) * extrude(
        Circle(bore_d / 2.0),
        amount=total_h + 2.0,
    )
    set_hole = teardrop_cutter(
        SET_SCREW_HOLE_D,
        (bore_d / 2.0 - 1.0, 0.0, SET_SCREW_Z),
        (1, 0, 0),
        (hub_od - bore_d) / 2.0 + 2.0,
        apex_up=False,
        roof_deg=HOLE_ROOF_DEG,
    )
    return (gear + hub) - bore - set_hole  # pyright: ignore[reportReturnType]  # build123d types a boolean as Compound | Wire


@cache
def set_screw() -> Part:
    """bd_warehouse's ISO 4026 set screw in pinion-local coordinates, in
    its hole along +X: flat point on the shaft's surface, socket end
    out. Its outer end sits at least SET_SCREW_RECESS under the hub's
    face, so nothing stands proud of the spinning hub. Bought, not
    inspected."""
    tip_x = NEMA17_SHAFT_D / 2.0
    assert tip_x + SET_SCREW_L <= PINION_HUB_OD / 2.0 - SET_SCREW_RECESS, "set screw stands proud of the hub"
    # bd_warehouse draws it socket end at z 0, point down −Z; this turns
    # +Z onto +X.
    return _set_screw().moved(Pos(tip_x + SET_SCREW_L, 0.0, SET_SCREW_Z) * Rot(0, 90, 0))


@cache
def motor_screw() -> Part:
    """bd_warehouse's ISO 4762 screw on its own axis: head on z 0 rising
    +Z, thread down −Z — fused with a thread-Ø cylinder over its length,
    because its simple shank is drawn under that Ø and the cylinder is
    what bears in the motor's tapped hole. Bought, not inspected."""
    screw = _motor_screw()
    body = screw + _column(screw.thread_diameter, MOTOR_SCREW_L, Pos(0, 0, -MOTOR_SCREW_L))
    assert len(body.solids()) == 1, "motor screw did not fuse into one body"
    return body  # pyright: ignore[reportReturnType]  # build123d types a boolean as Compound | Wire


def motor_screw_tools() -> Part:
    """The key's way in to a motor screw: MOTOR_SCREW_TOOL_D, straight up
    from the head's top MOTOR_SCREW_TOOL_L, in ``motor_screw``'s frame. A
    keep-out, never placed."""
    head_top = _motor_screw().head_height
    return _column(MOTOR_SCREW_TOOL_D, MOTOR_SCREW_TOOL_L, Pos(0, 0, head_top))


def motor_screw_entry() -> Part:
    """The way the screw goes in: its head's column straight up its
    axis, MOTOR_SCREW_TOOL_L long, in ``motor_screw``'s frame. It starts
    at the head's mid-height, so it does not count the counterbore
    floor the head is seated on. A keep-out, never placed."""
    screw = _motor_screw()
    z0 = screw.head_height / 2.0
    return _column(screw.head_diameter, MOTOR_SCREW_TOOL_L, Pos(0, 0, z0))


def motor_screw_name(i: int) -> str:
    """Leaf name of the motor screw at pattern corner i."""
    return f"motor_screw_{i}"


MOTOR_SCREW_NAMES = tuple(motor_screw_name(i) for i in range(len(_CORNERS)))


def motor_screw_locations(on_axis: Location) -> tuple[Location, ...]:
    """Where each ``motor_screw()`` goes: on the pattern square about the
    motor axis ``on_axis`` (which carries its rotation), its head on its
    counterbore's floor."""
    half = NEMA17_BOLT_PATTERN / 2.0
    seat_z = PLATE_TOP_Z - NEMA17_BOLT_CBORE_DEPTH
    return tuple(on_axis * Pos(sx * half, sy * half, seat_z) for sx, sy in _CORNERS)


def motor() -> Part:
    """NEMA 17 stub on its own axis: square body hanging below
    MOTOR_FACE_Z, pilot boss and shaft rising from the face, and the
    face's four M3 holes NEMA17_BOLT_TAP_DEPTH deep on the bolt pattern
    — drawn at the thread's major Ø, the envelope the screws are drawn
    at, so a screw in one bears on its wall."""
    half = NEMA17_BOLT_PATTERN / 2.0
    tap_d = _motor_screw().thread_diameter
    taps = [
        _column(tap_d, NEMA17_BOLT_TAP_DEPTH + 1.0, Pos(sx * half, sy * half, MOTOR_FACE_Z - NEMA17_BOLT_TAP_DEPTH))
        for sx, sy in _CORNERS
    ]
    body = Pos(0, 0, MOTOR_BOTTOM_Z + NEMA17_BODY_LEN / 2.0) * Box(
        NEMA17_SIDE,
        NEMA17_SIDE,
        NEMA17_BODY_LEN,
    ) - Compound(taps)
    pilot = Pos(0, 0, MOTOR_FACE_Z + NEMA17_PILOT_H / 2.0) * Cylinder(
        NEMA17_PILOT_D / 2.0,
        NEMA17_PILOT_H,
    )
    shaft = Pos(0, 0, MOTOR_FACE_Z + NEMA17_SHAFT_LEN / 2.0) * Cylinder(
        NEMA17_SHAFT_D / 2.0,
        NEMA17_SHAFT_LEN,
    )
    return body + pilot + shaft


# --- Assembly ----------------------------------------------------------


def build_pinion() -> Assembly:
    """The pinion and its set screw in their own frame (origin on the
    axis at z=0). The parent places them at PINION_AXIS_XY and owns the
    spin joint, so the screw turns with the hub. The screw sits in its
    hole without touching it: the hole is the tap drill, which the
    thread cuts on assembly, and bd_warehouse draws the screw at about
    the thread's minor Ø — so nothing here measures the thread's bite."""
    return (
        Assembly()
        .with_part("pinion", pinion(), material=material_of("pinion"))
        .with_part("set_screw", set_screw(), material=material_of("set_screw"))
        .assert_no_interference("set_screw", "pinion")
    )


def pinion_axis_xy(motor_azimuth_deg: float = MOTOR_AZIMUTH_DEG) -> tuple[float, float]:
    """Pinion / motor axis in m02-local XY for a given motor azimuth."""
    az = math.radians(motor_azimuth_deg)
    return (CENTER_DIST * math.cos(az), CENTER_DIST * math.sin(az))


def build_drive(
    motor_azimuth_deg: float = MOTOR_AZIMUTH_DEG,
    seam_phase_deg: float = BOWL_SEAM_PHASE_DEG,
    arc_screw_holes: bool = True,
) -> Assembly:
    """Drive sub-assembly in drive-local frame (= m02-local).

    ``motor_azimuth_deg`` is where the motor / pinion sit around the
    rotor axis (default ``params.MOTOR_AZIMUTH_DEG``, away from the
    exit arc). It is a per-*instance* choice — the mesh is the same
    at every azimuth — that the m02 chain sets per stage to keep a
    motor clear of that stage's gantry member (S2's stem, S3's cross
    bar; ``cad/m02_chain/params.py`` §Gantry).

    ``seam_phase_deg`` is where the bowl's thirds seams fall — also
    per instance, since they must miss that stage's gantry bar ends
    (the chain's §Seams). Guarded here against the two things the
    unit itself owns: the exit arc (slot and tongue), below, and the
    motor, measured — the motor stands on one sector and clears the
    other two, ears included (they hang below the floor, in the motor's
    own height band).

    ``arc_screw_holes`` drills the skirt for the liner's free-end
    screws — the unit root passes whether its liner is split (default
    True: the default liner is a plain wall).
    """
    ear_deg = math.degrees(2.0 * SEAM_EAR_T / PLATE_R_OUT)
    for seam in seam_azimuths(seam_phase_deg):
        assert angular_gap_deg(seam, 0.0) >= EXIT_ARC_DEG / 2.0 + ear_deg, (
            f"bowl seam at {seam:.1f}° runs into the exit arc's slot"
        )
    axis_xy = pinion_axis_xy(motor_azimuth_deg)
    on_axis = Location((axis_xy[0], axis_xy[1], 0.0)) * Rot(
        0,
        0,
        motor_azimuth_deg,
    )
    radial = (math.cos(math.radians(motor_azimuth_deg)), math.sin(math.radians(motor_azimuth_deg)), 0.0)
    skirt_face = Plane(origin=(SKIRT_R_IN * radial[0], SKIRT_R_IN * radial[1], 0.0), z_dir=radial)
    a = Assembly()
    for k, sector in enumerate(
        bowl_sectors(
            phase_deg=seam_phase_deg,
            motor_azimuth_deg=motor_azimuth_deg,
            arc_screw_holes=arc_screw_holes,
        )
    ):
        a = a.with_part(bowl_part(k), sector, material=material_of(bowl_part(k)))
    motor_sector = bowl_part_at(motor_azimuth_deg, seam_phase_deg)
    a = (
        a.with_part("bearing", bearing(), material=material_of("bearing"))
        .with_part("motor", motor(), location=on_axis, material=material_of("motor"))
        .with_subassembly(
            "pinion",
            build_pinion(),
            location=on_axis,
            joint=RevoluteJoint(
                axis=Axis((0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
                angle_deg=0.0,
                frame="local",
            ),
        )
        # The motor face rests on its sector's floor underside (pilot
        # and shaft inside the hole); the bore clears the shaft.
        .assert_tangent_contact("motor", motor_sector)
        .assert_no_interference("pinion.pinion", "motor")
        .assert_distance(
            "pinion.pinion",
            "motor",
            min_mm=MOTOR_SHAFT_BORE_CLEARANCE,
        )
        # Pinion and motor stay well clear of the bearing.
        .assert_distance("pinion.pinion", "bearing", min_mm=15.0)
        .assert_distance("motor", "bearing", min_mm=PLATE_T)
        # The pinion's tip circle stays inside the skirt with 1 mm to
        # spare, measured radially to the plane tangent to the skirt's
        # inner face on the pinion's own radius (a plain distance to
        # the bowl is the 0.5 mm hub-to-floor gap asserted below). The
        # part is the pitch cylinder, so grow it to the tip circle.
        .assert_distance(
            "pinion.pinion",
            skirt_face,
            along=radial,
            min_mm=1.0,
            grow_a_mm=ADDENDUM,
            name="pinion_tip_inside_skirt",
        )
        .assert_scalar("gear_ratio", GEAR_RATIO, detail="ring / pinion teeth")
        # The set screw's point bears on the shaft.
        .assert_tangent_contact("pinion.set_screw", "motor")
    )
    a = _with_motor_screws(a, on_axis, motor_sector)
    # The motor touches no sector but its own. A seam run into the
    # motor puts a neighbour's ear against its body; 1 mm is a real gap,
    # not a fit (the seam bolts' own corridors are held below).
    for part in BOWL_PARTS:
        if part != motor_sector:
            a = a.assert_distance(
                "motor",
                part,
                min_mm=1.0,
                name=f"motor_clear_of_{part}",
            )
    for k, part in enumerate(BOWL_PARTS):
        a = (
            a
            # The bearing's lower ring spans every seam: it rests on
            # each sector (and, bolted, is what splices the floor).
            .assert_tangent_contact("bearing", part)
            # Pinion hub floats just above the floor.
            .assert_distance(
                "pinion.pinion",
                part,
                min_mm=PINION_HUB_PLATE_CLEAR,
            )
            # Each seam: the two sectors' faces and ears meet on the
            # seam plane.
            .assert_tangent_contact(part, BOWL_PARTS[(k + 1) % SEAM_COUNT])
        )
    return with_seam_bolts(
        a,
        BOWL_PARTS,
        material_of,
        phase_deg=seam_phase_deg,
        r_out=PLATE_R_OUT,
        ear_z=BOWL_EAR_Z,
    )


def _with_motor_screws(a: Assembly, on_axis: Location, motor_sector: str) -> Assembly:
    """The motor's four screws and their claims: each head bears on its
    counterbore's floor (tangent contact with the motor's sector) and its
    thread on the motor's tapped hole (tangent contact with the motor);
    its head's way in overlaps nothing (``motor_screw_entry``: the two
    outboard counterbores stand under the skirt's inner face); and its
    key's corridor keeps MOTOR_SCREW_TOOL_MARGIN clear of every other
    part here.

    Here only, not against the machine: the corridor runs up through
    where the rotor flange and the liner go, and the two inboard screws
    end up under the flange. The motor goes on before the bearing and
    the rotor, and comes off only once the rotor is lifted out."""
    screw = _motor_screw()
    assert screw.head_diameter < NEMA17_BOLT_CBORE_D, "motor screw head does not fit its counterbore"
    assert screw.head_height <= NEMA17_BOLT_CBORE_DEPTH + 1e-6, "motor screw head stands above the floor"
    tip_z = PLATE_TOP_Z - NEMA17_BOLT_CBORE_DEPTH - MOTOR_SCREW_L
    assert MOTOR_FACE_Z - NEMA17_BOLT_TAP_DEPTH <= tip_z < MOTOR_FACE_Z, (
        f"motor screw tip at z {tip_z:.2f} misses the tapped hole (face {MOTOR_FACE_Z}, {NEMA17_BOLT_TAP_DEPTH} deep)"
    )
    placed = tuple(zip(MOTOR_SCREW_NAMES, motor_screw_locations(on_axis), strict=True))
    for name, loc in placed:
        a = a.with_part(name, motor_screw(), location=loc, material=material_of(name))
    everything = tuple(p.name for p in a.placed_parts)
    for name, loc in placed:
        others = [p for p in everything if p != name]
        a = (
            a.assert_tangent_contact(name, motor_sector)  # pyright: ignore[reportUnknownMemberType]  # cad_khana types a keep-out as Shape[Unknown]
            .assert_tangent_contact(name, "motor")
            .with_keepout(f"{name}_entry", loc * motor_screw_entry())
            .assert_clear_of(others, f"{name}_entry", name=f"{name}_entry_own", min_mm=0.0)
            .with_keepout(f"{name}_tools", loc * motor_screw_tools())
            .assert_clear_of(others, f"{name}_tools", name=f"{name}_tools_own", min_mm=MOTOR_SCREW_TOOL_MARGIN)
        )
    return a


# Degenerate memoized master, so ``khana check <file>`` resolves.
assembly = build_drive()
