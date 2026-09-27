"""
M03 scanner — two-unit subtree composition + within-unit checks.

Composes the integrated camera rig + turret in m03-local coordinates
(brick centroid on the IMAGE platform top face is the origin), as two
`with_subassembly` units:

  * `rig` (`arch/`, `camera_mount_esp32cam/`) at m03 identity —
    rig-local frame matches m03, so all rig coordinates in
    `params.py` are m03-local-equivalent.
  * `turret` placed at m03-local `Pos(0, -TURRET_ARM_R, 0)` so the
    turret hub axis lands at m03-local `(0, -ARM_R, *)` and the
    DUMP platform centroid at m03-local `(0, -2·ARM_R)` — which the
    world placement in `cad/assembly.py` maps onto the m05 hub axis
    (option a', 2026-05-05).

Part identity is the dotted tree path (`turret.rotor.arm.spider`,
`rig.cam_front_low_pcb`); cross-subtree checks are declared at the
m03 root against those paths — mostly via subtree group selectors —
while claims owned by a lower level (the turret's pulley Z corridor,
each builder's internal distance/scalar claims) are declared there
and propagate into this root's `check()` qualified. Attributes
(role tags, materials) key on the leaf name — a builder binds them in
its own frame without knowing where it will be placed.

Within-unit checks at this level are the union of arch-internal +
turret-internal + arch×turret pairs. After Phase 2 (2026-05-06), the
old rig T-frame (rails A/B/C/D + the B↔C corner brace + two L-
gussets + one T-gusset) collapses into the new `arch/` (2 posts +
2 inward stubs + rail D) plus the shared back rail (`frame/`'s
`floor_back_x`, bumped +20 mm in Y). Below-* clamps now grip the
shared back rail; back-upper hangs from rail D's underside via the
plain `clamp_mount(...)` body (the wedge variant was demolished
with rail C).

Sub-assemblies (all in `cad/m03_scanner/<sub>/`):
  Rig:    arch, camera_mount_esp32cam (×4 vertices)
  Turret: frame, hub, arm, turret_platform (×4 stations), dump_servo,
          drive, homing

Claims about this assembly's interaction with *probe* geometry — the
below-camera sightline cones — are external, and live in the check
module `check_cones.py`, which nothing here imports. That is what
keeps the cones out of every export.

`turret_platform/` keeps its `turret_` prefix because the bare
`platform` directory name collides with Python's stdlib `platform`
module — `from platform import assembly` would resolve to the
already-imported stdlib copy after `cad_khana`/`build123d` load.
The other turret subs (frame, hub, arm, ...) are safe to drop the
prefix on: `cad/` is a package, so m05's own `frame`/`drive` are
distinct modules under their package paths and never collide.
"""


from functools import cache

from build123d import Axis, Pos

from cad_khana.mechanism.assembly import Assembly, RevoluteJoint
from cad_khana.mechanism.assertions import JointWindow


from .params import (
    RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
    RAIL_PAD_CLAMP_BACK_UPPER_POS,
    RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
    RAIL_PAD_CLAMP_BELOW_LEFT_POS,
    RAIL_PAD_CLAMP_BELOW_RIGHT_CLAMP_ROT,
    RAIL_PAD_CLAMP_BELOW_RIGHT_POS,
    RAIL_PAD_CLAMP_FRONT_LOW_CLAMP_ROT,
    RAIL_PAD_CLAMP_FRONT_LOW_POS,
    TURRET_ARM_R,
    TURRET_DUMP_SERVO_PIVOT_X,
    TURRET_DUMP_SERVO_PIVOT_Y,
    TURRET_DUMP_SERVO_PIVOT_Z,
    TURRET_HINGE_PIN_RADIAL,
    TURRET_HINGE_PIN_Z,
    TURRET_HUB_X,
    TURRET_HUB_Y,
    VERTEX_BACK_UPPER_POS,
    VERTEX_BELOW_LEFT_POS,
    VERTEX_BELOW_RIGHT_POS,
    VERTEX_FRONT_LOW_POS,
    WALL_MIN,
    horizontal_x_roll_deg,
    lens_axis_unit,
    plate_back_centre_world,
    rotation_for_normal,
)
from .arch import assembly as _arch
from .arm import assembly as _arm
from .camera_mount_esp32cam import assembly as _camera_mount
from .drive import assembly as _drive
from .dump_servo import assembly as _dump_servo
from .frame import assembly as _frame
from .homing import assembly as _homing
from .hub import assembly as _hub
from .turret_platform import assembly as _turret_platform

from ..design_systems import DEFAULT_SCHEMA, paint
from .design_systems import design_system
from .materials import material_of


# --- Per-vertex camera specs ------------------------------------------
#
# Each instance contributes one printed body (`{prefix}_clamp_mount`)
# placed at the clamp pose plus one PCB stub (`{prefix}_pcb`) and one
# M12 lens stub (`{prefix}_lens`) placed at the vertex pose. The
# printed body's plate orientation is parameterised by `mount_in_clamp`
# so all four instances derive from one `clamp_mount(...)` function.

_CAMERA_INSTANCES = {
    "cam_front_low": {
        "clamp_pos": RAIL_PAD_CLAMP_FRONT_LOW_POS,
        "clamp_rot": RAIL_PAD_CLAMP_FRONT_LOW_CLAMP_ROT,
        "vertex_pos": VERTEX_FRONT_LOW_POS,
    },
    "cam_back_upper": {
        "clamp_pos": RAIL_PAD_CLAMP_BACK_UPPER_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT,
        "vertex_pos": VERTEX_BACK_UPPER_POS,
    },
    "cam_below_left": {
        "clamp_pos": RAIL_PAD_CLAMP_BELOW_LEFT_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT,
        "vertex_pos": VERTEX_BELOW_LEFT_POS,
    },
    "cam_below_right": {
        "clamp_pos": RAIL_PAD_CLAMP_BELOW_RIGHT_POS,
        "clamp_rot": RAIL_PAD_CLAMP_BELOW_RIGHT_CLAMP_ROT,
        "vertex_pos": VERTEX_BELOW_RIGHT_POS,
    },
}


# --- Naming -----------------------------------------------------------
#
# Identity is the tree path; there are no hand-maintained name tuples.
# The only explicit list is the rig's camera parts (direct children of
# `rig`, needed to split them from the `rig.arch` subtree in the
# rig-internal check).

TURRET_STATIONS = ("image", "load", "idle", "dump")

CAMERA_PART_PATHS = tuple(
    f"rig.{prefix}_{suffix}"
    for prefix in _CAMERA_INSTANCES
    for suffix in ("clamp_mount", "pcb", "lens")
)


# --- Composition builders --------------------------------------------

# Turret unit placement in m03-local: the turret's local origin (hub
# axis at the floor datum) lands at m03-local (0, -ARM_R, 0), so the
# DUMP centroid at turret-local (0, -ARM_R) reaches m03-local
# (0, -2·ARM_R) — which the world placement in `cad/assembly.py` maps
# onto the m05 hub axis (option a', 2026-05-05), while the rig brick
# centroid stays at the m03 origin.
TURRET_BASE = Pos(0, -TURRET_ARM_R, 0)


def build_camera_rig() -> Assembly:
    """Camera-rig sub-assembly. Rig-local frame == m03-local (the rig
    is placed at identity), so all rig coordinates in `params.py`
    apply unchanged."""
    # The front-low and back-upper clamps both hang from rail D's
    # underside in the x-symmetric pose, so their plate-back centres
    # must sit on the world YZ plane. `params.py` derives those from
    # the vertex positions but has no builder of its own to hold the
    # claim; this is the builder that places the clamps.
    for _label, _vertex in (
        ("front-low", VERTEX_FRONT_LOW_POS),
        ("back-upper", VERTEX_BACK_UPPER_POS),
    ):
        _back = plate_back_centre_world(_vertex)
        assert abs(_back[0]) < 1e-6, (
            f"{_label} vertex has drifted off the YZ plane; the "
            f"{_label} rail clamp assumes the x-symmetric (centered "
            f"on rail D) pose"
        )

    a = Assembly().with_subassembly("arch", _arch.build_arch())
    for prefix, spec in _CAMERA_INSTANCES.items():
        clamp_loc = Pos(*spec["clamp_pos"]) * spec["clamp_rot"]
        axis = lens_axis_unit(spec["vertex_pos"])
        camera_loc = Pos(*spec["vertex_pos"]) * rotation_for_normal(
            axis, roll_deg=horizontal_x_roll_deg(axis)
        )
        mount_in_clamp = _camera_mount.mount_in_clamp_for(**spec)
        clamp_body = _camera_mount.clamp_mount(mount_in_clamp=mount_in_clamp)
        a = (
            a
            .with_part(
                f"{prefix}_clamp_mount",
                clamp_body,
                location=clamp_loc,
                material=material_of(f"{prefix}_clamp_mount"),
            )
            .with_part(
                f"{prefix}_pcb", _camera_mount.camera_pcb_stub(),
                location=camera_loc,
            )
            .with_part(
                f"{prefix}_lens", _camera_mount.m12_lens_stub(),
                location=camera_loc,
            )
            # One `clamp_mount` function, four different bodies: the
            # plate's pose is what differs, so whether the plate stands
            # clear of the M4 clamp bolts' drop-in corridors — and
            # whether any plate-sized cut reached into the rail
            # interface block — is a per-camera fact. Measured on the
            # body this instance places (clamp-local), so every built
            # pose is covered, not just the standalone's below-left.
            .assert_scalar(
                f"{prefix}_clamp_bolt_access_clearance",
                _camera_mount.clamp_bolt_access_clearance(clamp_body),
                ge=_camera_mount.CLAMP_BOLT_ACCESS_MARGIN,
                detail="mm, Ø8 M4-head drop-in corridor above each clamp "
                "bolt seat vs the plate and beam",
            )
            .assert_scalar(
                f"{prefix}_clamp_base_shortfall",
                _camera_mount.clamp_base_shortfall_mm3(clamp_body),
                le=1e-3,
                detail="mm³ of the clamp base block missing beyond its "
                "two bolt bores — 0 means the rail interface is intact",
            )
        )
    # The below-* mount plates tilt with their lens axes and reach
    # ~25 mm in +Y past their clamp, toward the arch post on their
    # side. `along` compares Y projections, so this is the whole
    # printed body's reach — plate, beam and clamp — not the plate's
    # bounding-rectangle estimate the arch builder used to carry over
    # a mirrored plate size (bare until 2026-09-22).
    for prefix in ("cam_below_left", "cam_below_right"):
        side = "pos" if _CAMERA_INSTANCES[prefix]["vertex_pos"][0] > 0 else "neg"
        a = a.assert_distance(
            f"{prefix}_clamp_mount", f"arch.arch_post_{side}_x",
            along="Y", min_mm=_arch.CORNER_GAP,
            name=f"{prefix}_plate_post_gap",
        )
    return a


# --- Kinematic hinge pin (cartridge-local) -----------------------------
#
# One axis for all four stations. The cartridge is built with outward
# along +Y and the pin along ±X (see turret_platform), so in
# cartridge-local the hinge pin sits inboard of the centroid at
# y = PIN_RADIAL - ARM_R with direction (-1, 0, 0) — positive joint
# angle tips the outward (+Y) edge DOWN by the right-hand rule. The
# joint is declared ``frame="local"``: each station's ``location=``
# (centroid + Rz) rotates the axis with the cartridge, which is what
# previously required a per-station parent-frame axis table here.

_PLATFORM_PIN_LOCAL = RevoluteJoint(
    axis=Axis(
        (0.0, TURRET_HINGE_PIN_RADIAL - TURRET_ARM_R, TURRET_HINGE_PIN_Z),
        (-1.0, 0.0, 0.0),
    ),
    angle_deg=0.0,
    frame="local",
)


# --- Dump-lift contact phase ------------------------------------------
#
# The servo pad rises against the DUMP-parked platform's drop block —
# design-intended contact, not a defect. Root cause is topological
# rather than dimensional: any lifter that raises a pad against a drop
# block from below overlaps it somewhere in the lift. The retired
# kicker_lever tripped the same pair for the same reason, and until
# 2026-07-25 it was handled by blanket per-frame suppression, which is
# blind at every frame — the contact could have appeared at REST and
# nothing would have said so.
#
# The window below is geometry, derived not guessed: a 37-frame sweep
# of one dwell (`cad_khana.mechanism.sweep.classify`, 2026-07-25)
# reports the pair `transient` over two disjoint `t` intervals —
# 0.0292..0.0535 rising and 0.1215..0.1458 falling — which are ONE span
# in platform-tilt angle, bracketing 5.3582..22.5000°. Two phases, and
# notably not at peak tilt: past 22.5° the pad has passed the block.
# Windowing the joint angle rather than `t` is what collapses the two
# intervals into one claim and keeps it valid if the schedule is
# re-timed.
#
# The bracket is an outer bound because the overlap profile rises and
# falls exactly once across the span — the check cad-khana's SKILL.md
# now names as a required step before feeding `angles_bracketing` into
# a `JointWindow`. Re-derived at 43 frames over one dwell (2026-07-25):
# 7.50° → 0.416, 9.52 → 3.563, 11.66 → 8.368, 13.88 → 12.570,
# 16.12 → 14.680, 18.34 → 13.077, 20.48 → 6.662 mm³, zero at both ends
# and no dip between. A mid-span return to zero would have meant the
# samples straddled two contact events and the bracket edges said
# nothing about the second.
#
# Bound is the sampled peak plus headroom — 16.0 over the 14.680 mm³
# the denser sweep found (the 9-frame set had only seen 14.107, so the
# real headroom is 9%, not 13%). There is deliberately no
# `min_overlap_mm3`: overlap falls to ~0 at both window edges, so an
# engagement floor would fail there. Two phased claims can't be layered
# on one pair to cover that — outside its window a claim collapses to
# no-interference, so a narrow "must engage" claim reddens exactly the
# shallow-overlap frames the wide band allows. The measured volume is
# recorded on pass, so `khana diff` still sees the pad drifting out of
# contact even though no assertion claims it.
_DUMP_LIFT_TILT_MIN_DEG = 5.3582
_DUMP_LIFT_TILT_MAX_DEG = 22.5
_DUMP_LIFT_MAX_OVERLAP_MM3 = 16.0


def _build_turret_rotor() -> Assembly:
    """Rotor sub-assembly in turret-local: the rigid body that revolves
    about the hub Z axis. Includes the spider + hinge pins + every
    shaft-keyed part (hub shaft, driven pulley, homing flag), plus
    four platform sub-sub-assemblies — each carrying the shared
    cartridge-local pin joint (``_PLATFORM_PIN_LOCAL``), so the
    platforms inherit the rotor's azimuth automatically and add only
    their own tilt DOF on top. The dump_servo is bench-mounted (not
    on the rotor), so it doesn't appear here."""
    rotor = (
        Assembly()
        .with_subassembly("arm", _arm.build_turret_arm())
        .with_subassembly("hub", _hub.build_turret_hub_rotor())
        .with_subassembly("drive", _drive.build_turret_drive_rotor())
        .with_subassembly("homing", _homing.build_turret_homing_rotor())
    )
    for station in TURRET_STATIONS:
        rotor = rotor.with_subassembly(
            f"platform_{station}",
            _turret_platform.build_platform_cartridge(),
            location=_turret_platform.STATION_LOCATIONS[station],
            joint=_PLATFORM_PIN_LOCAL,
        )
    return rotor


def _build_servo_arm_subassembly() -> Assembly:
    """Arm-only sub-assembly in turret-local. The servo shaft joint is
    attached at ``with_subassembly`` time inside ``build_turret``; the
    arm rotates around that shaft during a dump dwell while the
    bracket + body stay with the static frame."""
    return _dump_servo.build_turret_dump_servo_arm()


def build_turret() -> Assembly:
    """Turret unit sub-assembly, in turret-local coordinates (hub axis
    at the local origin's X/Y, floor datum at z=0 terms of the frame
    constants). The caller places the whole unit with one transform.

    Structure:
      * static sub-assemblies (frame, hub housing + bearings, motor +
        bracket + motor pulley, homing sensor + bracket, servo
        bracket + body) at unit identity.
      * ``rotor`` is a sub-assembly whose ``RevoluteJoint`` about the
        turret-local Z axis through the hub drives the spider
        rotation. Inside it live four ``platform_<station>``
        sub-sub-assemblies sharing one cartridge-local pin joint
        (``_PLATFORM_PIN_LOCAL``) — only the platform parked at DUMP
        tilts during a dwell, but the same axis covers any station.
      * ``servo_arm`` is a sub-assembly with its servo-shaft joint on
        the same terms; the bracket + body remain static.

    All three joints are ``frame="local"``: each axis is written in
    the jointed sub-assembly's own frame (turret-local for the rotor
    and the arm, cartridge-local for the platforms), so nothing here
    depends on where the unit ends up."""
    t = (
        Assembly()
        .with_subassembly("frame", _frame.build_turret_frame())
        .with_subassembly("hub", _hub.build_turret_hub_static())
        .with_subassembly("drive", _drive.build_turret_drive_static())
        .with_subassembly("homing", _homing.build_turret_homing_static())
        .with_subassembly(
            "dump_servo", _dump_servo.build_turret_dump_servo_static()
        )
    )

    # Rotor — joint about the turret-local Z axis through the hub.
    # (Z is parallel to itself everywhere, so the pivot's Z value is
    # irrelevant; pick 0 for clarity.)
    t = t.with_subassembly(
        "rotor",
        _build_turret_rotor(),
        joint=RevoluteJoint(
            axis=Axis((TURRET_HUB_X, TURRET_HUB_Y, 0.0), (0.0, 0.0, 1.0)),
            angle_deg=0.0,
            frame="local",
        ),
    )

    # Servo arm — bench-mounted; joint about the servo output shaft
    # (turret-local axis along -Y so positive joint angle = lifter
    # pad up; same sign convention as the retired kicker_lever so
    # animated_assembly.py can drive it from the same tilt_schedule
    # without changing the schedule's sign).
    t = t.with_subassembly(
        "servo_arm",
        _build_servo_arm_subassembly(),
        joint=RevoluteJoint(
            axis=Axis(
                (
                    TURRET_DUMP_SERVO_PIVOT_X,
                    TURRET_DUMP_SERVO_PIVOT_Y,
                    TURRET_DUMP_SERVO_PIVOT_Z,
                ),
                (0.0, -1.0, 0.0),
            ),
            angle_deg=0.0,
            frame="local",
        ),
    )

    # Driven-pulley Z corridor — the pulley threads the gap between the
    # hub housing top (below) and the spider bottom (above), with ≥1 mm
    # air each side (grub-screw wrench swing / spin clearance). Declared
    # here, where the drive, hub, and arm subtrees meet — this retires
    # drive/assembly.py's `_SPIDER_BOTTOM_Z` / `HOUSING_TOP_Z` private
    # mirrors and their hand-derived Z arithmetic. The housing gap is
    # exactly 1 mm by construction (BRACKET_PLATE_AIRGAP_BELOW_PULLEY
    # sizes the bracket off the same corridor); bound comparisons
    # carry the library's 1e-6 tolerance, which covers the ~1e-7 of
    # bbox slop the extents carry.
    t = t.assert_distance(
        "rotor.drive.drive_pulley_shaft", "hub.hub_housing",
        along="-Z", min_mm=1.0, name="pulley_housing_gap",
    )
    t = t.assert_distance(
        "rotor.drive.drive_pulley_shaft", "rotor.arm.spider",
        along="Z", min_mm=1.0, name="pulley_spider_gap",
    )
    # Homing parts' lower Z bound — the flag and the sensor must clear
    # the driven pulley's top face by ≥1 mm. Declared here for the same
    # reason as the pulley corridor above: it retires
    # homing/assembly.py's PULLEY_TOP_Z mirror and its hand-derived Z
    # subtractions, measuring the live pulley instead. Both pairs are
    # pose-invariant (flag and pulley are rigid on the rotor; the
    # sensor is bench-mounted and the pulley is a body of revolution),
    # so the 12-frame sweep re-evaluates them safely.
    #
    # The matching UPPER bound — clearing the platform drop blocks — is
    # the rest-scoped per-station set at the end of this builder. The
    # bracket is not bounded below by the pulley: its
    # foot sits far beneath it and its post passes outboard in X.
    t = t.assert_distance(
        "homing.homing_sensor", "rotor.drive.drive_pulley_shaft",
        along="-Z", min_mm=1.0, name="homing_sensor_pulley_gap",
    )
    t = t.assert_distance(
        "rotor.homing.homing_flag", "rotor.drive.drive_pulley_shaft",
        along="-Z", min_mm=1.0, name="homing_flag_pulley_gap",
    )
    # X gaps the static parts keep to their neighbours, measured where
    # both parts first share a level. The driven pulley is a body of
    # revolution about the hub axis, so its X extent is the same at
    # every rotor azimuth. Both were bare asserts over placement
    # constants — the homing one over a `HOUSING_HALF_X` mirror —
    # until 2026-09-22.
    t = t.assert_distance(
        "rotor.drive.drive_pulley_shaft", "drive.drive_motor",
        along="X", min_mm=WALL_MIN, name="pulley_motor_body_gap",
    )
    t = t.assert_distance(
        "homing.homing_bracket", "hub.hub_housing",
        along="X", min_mm=WALL_MIN, name="homing_foot_housing_gap",
    )
    # The hub housing stands on the cross-rail. No-interference alone
    # also passes with the housing floating above it.
    t = t.assert_tangent_contact("hub.hub_housing", "frame.floor_cross_x")
    # Driven pulley pressed onto the real hub shaft — the drive
    # standalone declares the same fit against its own shaft stub;
    # this is the composed-run pair. Bore Ø7.98 on the Ø8 shaft, so
    # the overlap is genuine — and this declaration is what makes the
    # within-turret group check skip the pair.
    t = t.assert_allowed_contact(
        "rotor.drive.drive_pulley_shaft", "rotor.hub.hub_shaft",
        min_overlap_mm3=1.5, max_overlap_mm3=2.6,
        reason=(
            f"press fit, {_drive.PULLEY_SHAFT_PRESS_FIT} mm diametral on Ø8"
        ),
    )
    # Drive-bracket foot resting on the cross-rail's top face — the
    # rail side of the bolted foot joint, asserted here against the
    # real member (the drive standalone asserts it against its rail
    # stub). A foot floating above the rail passes no-interference;
    # this is what says the joint closes.
    t = t.assert_tangent_contact("drive.drive_bracket", "frame.floor_cross_x")
    # …and resting on it *alone*: tangent contact is blind to the foot
    # running off the rail's +X end onto the side rail's top (same z).
    # The foot grew outboard on 2026-09-16 (both bolts past the wall)
    # to within ~3 mm of the cross-rail end; the corner gap is the
    # directed distance the foot must keep from the +X side rail.
    t = t.assert_distance(
        "drive.drive_bracket", "frame.floor_right_y",
        along="X", min_mm=_frame.CORNER_GAP,
        name="drive_foot_on_cross_rail",
    )
    # Detail-tier, same joint: the two M5 foot bolts dip 6 mm past the
    # rail top face into the T-slot opening — hollow on the detailed
    # V-slot that `detail_variations` swaps in for `floor_cross_x`,
    # solid on the cheap Box. So the claim is evaluable only in a
    # detail-applied run; in a cheap `check()` these skip
    # (`passed: null`). Declared *here* rather than at the unit root:
    # this is the level where the drive and frame subtrees meet, so
    # the claim qualifies into every composed root above it. The
    # bolt-vs-bracket and bolt-vs-motor claims live one level down, in
    # `drive/assembly.py`'s static builder.
    for bolt in _drive.DETAIL_FOOT_BOLT_NAMES:
        t = t.assert_no_interference(f"drive.{bolt}", "frame.floor_cross_x")
    # Same joint, servo side — the dump-servo bracket's foot rests on
    # the same cross-rail. Was a `_BRACKET_PLATE_BOTTOM_Z == RAIL_TOP_Z`
    # constant comparison inside dump_servo; the standalone asserts it
    # against its rail stub, this is the real-member pair.
    t = t.assert_tangent_contact(
        "dump_servo.servo_bracket", "frame.floor_cross_x"
    )
    # Servo pad × drop block, once per station: whichever platform is
    # parked at DUMP tilts, so the claim belongs to all four cartridges
    # (they come from one builder, so the window is the same geometry
    # each time — verified at a second station in `animated_assembly`'s
    # frame set). Outside the tilt window the pair is held to plain
    # no-interference, which is the coverage the old suppression list
    # threw away.
    for station in TURRET_STATIONS:
        t = t.assert_allowed_contact(
            f"rotor.platform_{station}.frame", "servo_arm.servo_arm",
            max_overlap_mm3=_DUMP_LIFT_MAX_OVERLAP_MM3,
            during=JointWindow(
                f"rotor.platform_{station}",
                _DUMP_LIFT_TILT_MIN_DEG,
                _DUMP_LIFT_TILT_MAX_DEG,
            ),
            name=f"dump_lift_contact_{station}",
            reason="servo pad lifts the DUMP-parked platform's drop block",
        )
    # Drop-block underside Z band, once per station: every platform
    # indexes over the homing parts (azimuth 180°) and the servo pad
    # (DUMP) in transit, so each cartridge's drop block must clear
    # both from above. `along` compares Z projections, XY-blind — the
    # conservative form, since the rotor puts any platform over either.
    # **Rest-scoped**: a tilting platform's outer edge sweeps down
    # through the same band without touching anything (down to
    # −25.1 mm at the 30° peak, `frame_sweep.py` with the window
    # removed), so the claim holds only at tilt 0 and lapses outside it;
    # the pad bound also needs the servo arm down. These were bare
    # asserts over a `-43` drop-block mirror in homing and dump_servo
    # until 2026-09-21 — neither subtree contains a platform.
    for station in TURRET_STATIONS:
        platform = f"rotor.platform_{station}"
        at_rest = JointWindow(platform, max_deg=0.0)
        for homing_part in _homing.HOMING_PART_NAMES:
            homing_path = (
                f"rotor.homing.{homing_part}"
                if homing_part in _homing.HOMING_ROTOR_PART_NAMES
                else f"homing.{homing_part}"
            )
            t = t.assert_distance(
                homing_path, f"{platform}.frame",
                along="Z", min_mm=1.0, during=at_rest,
                name=f"{homing_part}_drop_block_gap_{station}",
            )
        t = t.assert_distance(
            "servo_arm.servo_arm", f"{platform}.frame",
            along="Z", min_mm=2.0,
            during=(at_rest, JointWindow("servo_arm", max_deg=0.0)),
            name=f"servo_pad_drop_block_gap_{station}",
        )
    return t


def add_m03(a: Assembly) -> Assembly:
    """Compose m03 as two unit subtrees: the camera rig at m03
    identity and the turret unit placed by ``TURRET_BASE``. Identity
    is the tree path (``placed_parts`` qualifies names as it
    flattens): checks, renders, GLB nodes, and diff all address
    ``turret.rotor.arm.spider``-style paths, while attribute maps
    (role tags, materials) key on the leaf name. Joint paths:
    ``turret.rotor``, ``turret.rotor.platform_<station>``,
    ``turret.servo_arm``."""
    a = a.with_subassembly("rig", build_camera_rig())
    a = a.with_subassembly("turret", build_turret(), location=TURRET_BASE)
    # The arch posts stand on the back outriggers, behind the shared
    # back rail with a butt-joint gap. The rail lives in the turret's
    # frame and the posts in the rig, so this is the first level that
    # holds both (a bare assert in the arch builder over the rail's Y
    # until 2026-09-22).
    for side in ("pos", "neg"):
        a = a.assert_distance(
            "turret.frame.floor_back_x", f"rig.arch.arch_post_{side}_x",
            along="Y", min_mm=_arch.CORNER_GAP,
            name=f"arch_post_{side}_x_behind_back_rail",
        )
    # Exported interface anchors (m03-local). ``load_top`` /
    # ``dump_centroid``: station centroids on the platform top-face
    # plane (stations at hub + TURRET_ARM_R·heading; LOAD at turret
    # heading 0°, DUMP at 270°) — where the m02 chain delivers and
    # where bricks drop into the m05 column. ``floor_bottom``
    # re-exports the frame's own declaration (where the member
    # cross-section lives) at unit scope.
    a = a.with_anchor("load_top", TURRET_BASE * Pos(TURRET_ARM_R, 0, 0))
    a = a.with_anchor("dump_centroid", TURRET_BASE * Pos(0, -TURRET_ARM_R, 0))
    a = a.with_anchor("floor_bottom", a.anchor("turret.frame.floor_bottom"))
    return a


# --- Internal-checks helpers -----------------------------------------
#
# Turret-internal known overlaps (kept identical to the prior
# `turret/assembly.py` `_KNOWN_OVERLAPS`). Each entry is
# (part_a, part_b, reason); the integrate loop downgrades these
# pairs to `assert_interference(reason=...)` so they self-fail when
# the underlying design fix lands and the overlap goes away.

_KNOWN_OVERLAPS_STATIC: tuple[tuple[str, str, str], ...] = ()

# There is no `suppressed=` list here. The design-intended overlaps —
# the driven pulley's press fit on the hub shaft, and the four
# per-station dump-lift pairs — are each declared with
# `assert_allowed_contact` in `build_turret`, and the group check
# derives its exclusions from those declarations (cad-khana `db3f6a8`,
# 2026-07-25). Restating them was double bookkeeping to keep in step,
# and it went *wider* than the claim: a suppression is blind at every
# frame, while a phased claim holds its pair to plain no-interference
# outside the tilt window.

# Backwards-compatible alias (kept for module-level callers that took
# the previous symbol name).
_KNOWN_OVERLAPS = _KNOWN_OVERLAPS_STATIC


def add_m03_internal_checks(
    a: Assembly,
    known_overlaps: tuple[tuple[str, str, str], ...] = _KNOWN_OVERLAPS_STATIC,
) -> Assembly:
    """Add every within-m03 cross-boundary check.

    Three blocks:
      1. Rig-internal — every non-arch (camera mounts) vs every arch
         member.
      2. Turret-internal — every pair, with the small allow-list of
         known overlaps downgraded to `assert_interference(reason=…)`.
         Pairs carrying an `assert_allowed_contact` declared at or
         below the turret are skipped by the group form itself.
      3. Cross — every turret part vs every rig part.

    Frame-independent: the same checks apply at every animation frame,
    because the one pair whose overlap state varies with the motion is
    declared as a phased `assert_allowed_contact` in ``build_turret``
    rather than suppressed here. The per-frame ``suppressed_pairs``
    argument this took until 2026-07-25 is gone with it; a caller that
    needs a pair excluded now has to say during which joint phase, and
    that claim belongs at the level owning the pair.
    """
    # 1. Rig-internal — camera mounts vs every arch member (no
    #    suppressions apply within the rig).
    a = a.assert_no_interference_between(CAMERA_PART_PATHS, "rig.arch")

    # 2. Turret-internal — every pair under the turret subtree.
    a = a.assert_no_interference_within(
        "turret",
        known_overlaps=known_overlaps,
    )

    # 3. Cross — turret × rig.
    a = a.assert_no_interference_between("turret", "rig")

    return a



# --- Unit root -------------------------------------------------------

# Cached because the degenerate ``assembly`` memo below and any
# composing parent (``cad/assembly.py``, ``cad/render.py``) each ask
# for the same schema, and the build is seconds of OCCT work. Replaces
# import-time memoization: the memo line is now free, and a parent
# calling the builder gets the same object rather than rebuilding.
# Safe to share — ``Assembly`` is a frozen dataclass and every
# ``with_*`` returns a copy.
#
# Call it with the schema **explicit** everywhere, including the memo
# below: ``@cache`` keys on the call signature, so ``build_x()`` and
# ``build_x(DEFAULT_SCHEMA)`` are two entries that build identical
# geometry twice. That mistake is invisible — same output, double the
# work — so ``cache_info().misses`` is the only thing that shows it.
@cache
def build_m03(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The m03 scanner unit, painted under ``schema``.

    Sub-assembly builders emit no colour; this root paints the finished
    tree once. ``paint`` also carries the design-system coverage check
    that used to be a module-level assert here — every leaf name must
    be tagged, and a palette missing tags names every offender at build
    time instead of crashing the import.
    """
    a = add_m03_internal_checks(add_m03(Assembly()))
    return paint(a, design_system(schema))


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``.
assembly = build_m03(DEFAULT_SCHEMA)
