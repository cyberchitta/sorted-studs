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
`rig.cam_front_low_pcb`); checks are declared at the m03 root against
those paths — mostly via subtree group selectors (assertions inside
nested assemblies are NOT evaluated by the root `check()`). Attributes
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

`turret_platform/` keeps its `turret_` prefix because the bare
`platform` directory name collides with Python's stdlib `platform`
module — `from platform import assembly` would resolve to the
already-imported stdlib copy after `cad_khana`/`build123d` load.
The other turret subs (frame, hub, arm, ...) are safe to drop the
prefix on; the m05 collisions on `frame`/`drive` are handled by
`cad/assembly.py`'s per-unit module eviction.
"""


from build123d import Axis, Circle, Pos, Rectangle, loft

from cad_khana.mechanism.assembly import Assembly, RevoluteJoint
from cad_khana.mechanism.check import check


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
from .sizing import BRICK_MAX_MM, SCATTER_HALF_MM
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

# Backwards-compatible alias (kept for module-level callers that took
# the previous symbol name).
_KNOWN_OVERLAPS = _KNOWN_OVERLAPS_STATIC


def add_m03_internal_checks(
    a: Assembly,
    known_overlaps: tuple[tuple[str, str, str], ...] = _KNOWN_OVERLAPS_STATIC,
    suppressed_pairs: frozenset[tuple[str, str]] = frozenset(),
) -> Assembly:
    """Add every within-m03 cross-boundary check.

    Three blocks:
      1. Rig-internal — every non-arch (camera mounts) vs every arch
         member.
      2. Turret-internal — every pair, with the small allow-list of
         known overlaps downgraded to `assert_interference(reason=…)`
         and the optional `suppressed_pairs` skipped entirely.
      3. Cross — every turret part vs every rig part.

    ``known_overlaps`` defaults to the static set; per-frame callers in
    ``animated_assembly.py`` swap in a different set when the lever /
    a platform is mid-lift.

    ``suppressed_pairs`` (frozenset of ``(path_a, path_b)`` — order
    independent) skips both the ``assert_no_interference`` and any
    ``assert_interference`` for those pairs. Use for dynamic-conflict
    pairs whose overlap state changes throughout the animation but is
    a single documented kinematic root cause (e.g. the lever hub-end
    block in the IDLE arm corridor — overlap present at some frames,
    absent at others, no clean per-frame predicate).
    """
    # 1. Rig-internal — camera mounts vs every arch member (no
    #    suppressions apply within the rig).
    a = a.assert_no_interference_between(CAMERA_PART_PATHS, "rig.arch")

    # 2. Turret-internal — every pair under the turret subtree.
    a = a.assert_no_interference_within(
        "turret",
        known_overlaps=known_overlaps,
        suppressed=suppressed_pairs,
    )

    # 3. Cross — turret × rig.
    a = a.assert_no_interference_between(
        "turret",
        "rig",
        suppressed=suppressed_pairs,
    )

    return a


# --- Sightline cones (silhouette assertion, was c7-c) -----------------
#
# One check-only stub solid per below-vertex camera: the forbidden
# volume between that camera's lens nose and the imaged region it must
# see through the transparent platform. Any opaque part inside it
# occludes part of the imaged region, which `khana check` is otherwise
# blind to (`cad/CLAUDE.md` §"What a green `khana check` does not
# mean", Visibility). This assertion is the gate for any further
# turret Z compression (state file, Open work 3).
#
# Geometry: an oblique frustum lofted from a small horizontal disc
# just past the lens nose (entrance-pupil stand-in) to the imaged
# square at the platform plane z = 0 — `sizing.py`'s
# P_imaged = BRICK_MAX_MM + 2·SCATTER_HALF_MM (110 at the settled
# 3.6 mm lens), axis-aligned and centred on the brick centroid. The
# square (not its circumscribed circle) is the real imaged region;
# the circle would falsely collide with the platform frame's edge
# grip 5 mm outside it.
#
# Pose claim: asserted at the parked REST pose only. Capture happens
# only during an IMAGE-station dwell with the rotor indexed, so
# transit poses are out of scope; `animated_assembly.py` deliberately
# does not run these checks (mid-transit platforms sweep through the
# cones by design).
#
# Check-only: the cones are appended to a *separate* checked assembly,
# never to `assembly` itself — `render*.py`, `export_glb_animated.py`,
# and `cad/assembly.py` all consume `assembly`, and an opaque cone in
# front of each below camera would wreck renders/GLB (and demand
# design-system tags). Only acrylic parts are exempt from the
# assertions — the below sightlines pass through the transparent
# platform by design.

SIGHTLINE_CONE_SPECS = {
    "sightline_below_left": VERTEX_BELOW_LEFT_POS,
    "sightline_below_right": VERTEX_BELOW_RIGHT_POS,
}

# Imaged-square side at the platform plane (110 at lens 3.6 / brick 80
# / scatter ±15). Live import from sizing.py — no manual mirror; the
# params.py vertex block is reconciled to the same inputs.
_SIGHTLINE_P_IMAGED = BRICK_MAX_MM + 2 * SCATTER_HALF_MM

# Entrance-pupil stand-in disc at the apex (a true M12 pupil at
# f/2.8 is ~Ø1.3; Ø4 adds margin), pushed a further 3 mm past the
# lens nose along the axis so the horizontal disc cannot poke back
# through the tilted lens front face (3 > disc radius ≥ worst-case
# back-reach).
_SIGHTLINE_APEX_D = 4.0
_SIGHTLINE_APEX_SETBACK = 3.0


def sightline_cone(vertex_pos: tuple[float, float, float]):
    """Oblique frustum from the camera's lens nose to the imaged
    square at the platform plane, in m03-local coordinates (place at
    identity). Lens nose = vertex (PCB rear face) + (PCB_T + M12_L)
    along the lens axis, mirroring `m12_lens_stub`'s extent."""
    axis = lens_axis_unit(vertex_pos)
    nose_dist = (
        _camera_mount.PCB_T + _camera_mount.M12_L + _SIGHTLINE_APEX_SETBACK
    )
    apex = tuple(
        vertex_pos[i] + nose_dist * axis[i] for i in range(3)
    )
    return loft(
        [
            Pos(*apex) * Circle(_SIGHTLINE_APEX_D / 2),
            Rectangle(_SIGHTLINE_P_IMAGED, _SIGHTLINE_P_IMAGED),
        ]
    )


# Known cone overlap (2026-07-24, first run of this assertion): the
# acrylic aperture equals the imaged square (ACRYLIC_INSERT_SIZE =
# IMAGED_REGION_SIDE_MM = 110), so each oblique below camera's
# sightline to its own near edge necessarily passes through the IMAGE
# cartridge frame below the platform plane — the perimeter wall band
# (|x| ∈ [55, 60], z ∈ [-4, 0]) plus the two near-side corner lips
# (z ∈ [-7, -4]). ≈958 mm³ per side. Optical cost: bottom-sliver
# vignetting of a brick parked at extreme near-edge scatter, and the
# opposite below camera sees that sliver unobstructed. Kept as a
# regression alarm, not fixed: growing the aperture past the imaged
# square or moving the lips inboard-out is a cartridge redesign the
# vignetting doesn't currently justify — if either lands, this
# assertion self-fails and must be removed.
_SIGHTLINE_KNOWN_OVERLAP_REASON = (
    "aperture == imaged square (110): below-* near-edge sightlines "
    "graze the IMAGE frame wall + corner lips below z=0; bottom-sliver "
    "vignetting covered by the opposite below camera"
)
_SIGHTLINE_KNOWN_OVERLAPS = frozenset({"turret.rotor.platform_image.frame"})


def add_sightline_cone_checks(a: Assembly) -> Assembly:
    """Add both below-vertex sightline cones and assert them clear of
    every m03 part except acrylics (transparent by design) — with the
    documented frame-graze pairs downgraded to `assert_interference`
    regression alarms. Apply to a copy consumed only by `check()` —
    never to the rendered/exported `assembly`."""
    # Opaque set fixed at entry — the cones added below must not land
    # in each other's check groups.
    opaque = [p.name for p in a.placed_parts if "acrylic" not in p.name]
    for cone_name, vertex_pos in SIGHTLINE_CONE_SPECS.items():
        a = a.with_part(cone_name, sightline_cone(vertex_pos))
        a = a.assert_no_interference_between(
            (cone_name,),
            opaque,
            known_overlaps=tuple(
                (cone_name, other, _SIGHTLINE_KNOWN_OVERLAP_REASON)
                for other in _SIGHTLINE_KNOWN_OVERLAPS
            ),
        )
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


if __name__ == "__main__":
    check(add_sightline_cone_checks(assembly), out="outputs")
