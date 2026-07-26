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
prefix on; the m05 collisions on `frame`/`drive` are handled by
`cad/assembly.py`'s per-unit module eviction.
"""


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
    horizontal_x_roll_deg,
    lens_axis_unit,
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

from .design_systems import DESIGN_SYSTEM, color_of
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
    a = Assembly().with_subassembly("arch", _arch.build_arch())
    for prefix, spec in _CAMERA_INSTANCES.items():
        clamp_loc = Pos(*spec["clamp_pos"]) * spec["clamp_rot"]
        axis = lens_axis_unit(spec["vertex_pos"])
        camera_loc = Pos(*spec["vertex_pos"]) * rotation_for_normal(
            axis, roll_deg=horizontal_x_roll_deg(axis)
        )
        mount_in_clamp = _camera_mount.mount_in_clamp_for(**spec)
        a = (
            a
            .with_part(
                f"{prefix}_clamp_mount",
                _camera_mount.clamp_mount(mount_in_clamp=mount_in_clamp),
                location=clamp_loc,
                material=material_of(f"{prefix}_clamp_mount"),
                color=color_of(f"{prefix}_clamp_mount"),
            )
            .with_part(
                f"{prefix}_pcb", _camera_mount.camera_pcb_stub(),
                location=camera_loc,
            )
            .with_part(
                f"{prefix}_lens", _camera_mount.m12_lens_stub(),
                location=camera_loc,
            )
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



# --- Standalone assembly ---------------------------------------------

assembly = add_m03_internal_checks(add_m03(Assembly()))

# Detail-tier fastener assertions — rail side of the drive-bracket
# foot joint. The M5 foot bolts enter via
# `detail_variations.apply_default_detail`, whose additions land at
# the root the override is applied to (this assembly), and dip 6 mm
# past the rail top face into the T-slot opening — hollow on the
# detailed V-slot the same override swaps in for `floor_cross_x`,
# solid on the cheap Box. So the claim is evaluable only in a
# detail-applied run; in the cheap `check()` these skip
# (`passed: null`). The bracket side lives in `drive/assembly.py`
# beside the joint.
for _bolt in ("drive_foot_bolt_inner", "drive_foot_bolt_outer"):
    assembly = assembly.assert_no_interference(
        _bolt, "turret.frame.floor_cross_x"
    )


# --- Design-system coverage ------------------------------------------
#
# Every m03 part's leaf name must appear in the active design system's
# tagging dict (`None` allowed for hardware / optical glass / acrylic
# — those keep their material's intrinsic color). Attributes key on
# the leaf (a builder binds them without knowing its placement path);
# identity stays the full path. A part added without a tag fails here
# instead of silently rendering off-palette. Swapping `DESIGN_SYSTEM`
# to a palette with missing tags also fails here, so the swap surface
# is loud.

_untagged = sorted(
    {
        leaf
        for p in assembly.placed_parts
        if (leaf := p.name.rsplit(".", 1)[-1]) not in DESIGN_SYSTEM.tag_of
    }
)
assert not _untagged, (
    f"{len(_untagged)} m03 part leaf name(s) missing a tag for design "
    f"system '{DESIGN_SYSTEM.name}' in design_systems.py: {_untagged}"
)
