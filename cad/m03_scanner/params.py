"""
M03 scanner — unit-local constants and placement helpers.

Single source of truth for the merged camera-rig + turret unit. The
unit-local frame matches world (Z-up; the top-level scanner places
the unit at world identity for the rig anchors and at
``Pos(0, +TURRET_ARM_R, 0)`` for the turret-internal placements):

    origin = brick centroid on the IMAGE platform top face
    +X     = operator's right            (parallel to world +X)
    +Y     = away from operator          (parallel to world +Y)
    +Z     = up                          (parallel to world +Z)

Every sub-assembly under ``cad/m03_scanner/`` (rig + turret + shared
helpers) imports the constants below via the package-relative
``from ..params import …`` (``from .params import …`` at unit level).

The earlier rev-3 split (``camera_rig/params.py`` +
``turret/params.py`` + this file as the inter-unit interface mirror)
was collapsed in the integrated-frame Phase-1 refactor — the rig and
turret share a single bench frame in the upcoming Phase-2 geometry
change, so a per-unit params boundary no longer earns its place.

Tetrahedral verification (2026-05-05): all four camera vertices sit
at exactly r = 160 mm from origin, and every pairwise dot product of
unit-vectors-from-origin is −1/3 — the regular-tetrahedron signature
(109.47° between every pair). Slice c2 shrunk r 250 → 160 (the
3.6 mm M12 lens choice from ``_notes/archive/m03-rig-shrink-handoff.md``); rail anchors
follow from ``sizing.compute_geometry()``'s conservative envelope
(mount_offset = plate_t + half_side = 30, rail_clear =
mount_offset + RAIL/2 + GAP = 40.3). See ``_notes/decisions/m03-cameras.md``
for the rev-5 pivot rationale and ``_notes/archive/m03-rig-shrink-handoff.md`` for the
shrink rationale.
"""

import math

from build123d import Rot


# --- Project-wide policy -----------------------------------------------

# FDM printability floor — minimum wall thickness any printed
# sub-assembly may carry.
WALL_MIN = 1.5

# 90% of LEGO parts fit within an 80 mm sphere. Drives the minimum
# platform aperture and the camera-rig clearance envelope. Optical
# sightline cones at the platform plane (≤ Ø172 per rev-1's design)
# dominate this for acrylic sizing.
BRICK_SPHERE_DIA = 80.0

# Height a *settled* brick reaches on a flat platform (owner
# 2026-09-14): the LOAD platform's loaded envelope for anything that
# passes over the turret's sweep — the m02 chain's S1 stands over it.
# Replaces the any-orientation sphere for that one clearance; the
# sphere still sizes apertures and the camera rig. Asserted, with
# 5 mm clearance, by `cad/check_turret_envelope.py`; it set S1's raise
# (`cad/m02_chain/params.py:S1_VBOT_WORLD_Z`) and nothing else
# (`_notes/decisions/m02-chain.md` §11.6).
BRICK_SETTLED_H = 30.0


# --- Acrylic platform stock spec ---------------------------------------
#
# Sized to the per-camera imaged region from the rev-5 4-cam direct
# topology (sizing.py at lens=3.6 / brick_max=80 / scatter=±15):
# imaged region = 110 × 110 mm + 5 mm PMMA edge grip per side =
# 120 × 120 mm. The prior 180 mm was sized against a Ø172 mirror-era
# sightline cone (rev-1) that no longer exists. Slice c3 (2026-05-05)
# shrunk this 180 → 120 alongside the turret arm shrink.
#
# 4 mm thickness gives enough stiffness to span 120 mm under a brick
# load without flex, and enough edge thickness to host bonded brackets
# without the bond joint dominating geometry.
#
# Used by the rev-3 single-platform stack (now superseded) and the
# rev-4 turret platform cartridges.

PLATFORM_ACRYLIC_SIZE = 120.0
PLATFORM_ACRYLIC_T = 4.0


# --- Workbench reference -----------------------------------------------

# Workbench surface Z. The project-wide reference for the eventual
# physical bench top. Both the camera rig and the turret floor used to
# sit directly on this plane; both have since floated up:
#   - Slice c2 (2026-05-05) raised the camera rig (rail A from z=-220
#     to z=-131) when the rig shrunk to r=160.
#   - Slice c4 (2026-05-05) raised the turret floor (-220 → -180) when
#     the hub housing shrunk 100 → 60 mm.
# A future floor-feet slice will bridge the rig and turret bases down
# to this surface; SHARED_BENCH_Z stays as the workbench reference.
SHARED_BENCH_Z = -220.0


# ─────────────────────────────────────────────────────────────────────
# Camera-rig sizing inputs — vertices and rail anchors below were
# generated from cad/m03_scanner/sizing.py with:
#     LENS_FOCAL_MM    = 3.6  (M12)
#     TILT_DEG         = 65.0
#     BRICK_MAX_MM     = 80.0
#     SCATTER_HALF_MM  = 15.0
#     TETRA_R          = 160.0  (slice c2 shrink, 2026-05-05)
# Last reconciled: 2026-05-05.
#
# To re-derive: edit sizing.py, run it, copy the tetrahedral block
# below verbatim, update LENS_FOCAL_MM/TETRA_R/etc above, update the
# date. Then `khana check` at all three levels (mount / camera_rig /
# m03 top-level).
# ─────────────────────────────────────────────────────────────────────


# --- Tetrahedral camera vertices ----------------------------------------
#
# World-frame positions where each camera's PCB rear face lands. Names
# are pose-descriptive (independent of any prior mirror/camera role) so
# a future role swap doesn't churn the names.
#
# Values from sizing.tetra_vertices_unit(tilt_deg=65) scaled by r=160.

VERTEX_FRONT_LOW_POS = (0.0, -145.0, 67.6)
VERTEX_BACK_UPPER_POS = (0.0, 112.1, 114.2)
VERTEX_BELOW_LEFT_POS = (-130.6, 16.5, -90.9)
VERTEX_BELOW_RIGHT_POS = (130.6, 16.5, -90.9)


def lens_axis_unit(
    vertex_pos: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Unit vector from a camera vertex toward the world origin
    (= the lens optical axis direction for a centroid-pointing camera).
    """
    mag = math.sqrt(sum(c * c for c in vertex_pos))
    return tuple(-c / mag for c in vertex_pos)


# --- Camera mount external interface ------------------------------------
#
# Plate thickness exposed here because the per-vertex clamp pose
# constants below land the clamp directly under the plate-back centre,
# and the back centre depends on the plate offset along the lens axis.
# PCB outline + corner-bolt pattern stay private inside
# `camera_mount_esp32cam/`.

CAMERA_MOUNT_PLATE_T = 5.0


def plate_back_centre_world(
    vertex_pos: tuple[float, float, float],
    plate_t: float = CAMERA_MOUNT_PLATE_T,
) -> tuple[float, float, float]:
    """Mount plate back-face centre in world: vertex_pos shifted away
    from origin along the vertex direction by plate_t.

    Equivalently: vertex_pos − plate_t · lens_axis_unit(vertex_pos),
    since lens_axis is the unit vector from vertex toward origin.
    """
    axis = lens_axis_unit(vertex_pos)
    return tuple(vertex_pos[i] - plate_t * axis[i] for i in range(3))


def rotation_for_normal(
    normal: tuple[float, float, float],
    roll_deg: float = 0.0,
) -> Rot:
    """Rotation that carries a sub-assembly's local +Z onto `normal`.

    Used to orient each camera mount: the mount's local +Z is the lens
    axis (from vertex toward origin), and this returns the `Rot(...)`
    that places that direction correctly in world space.

    Roll about the target normal is unconstrained for the symmetric
    PCB + 4-corner bolt pattern at this rev. `roll_deg` sets the
    `Rz` component of the Euler triple; callers that need mount-local
    +X to fall on a specific world direction (e.g. horizontal for
    cable-routing consistency) supply the value computed by
    `horizontal_x_roll_deg(normal)`.
    """
    nx, ny, nz = normal
    n_mag = math.sqrt(nx * nx + ny * ny + nz * nz)
    nx, ny, nz = nx / n_mag, ny / n_mag, nz / n_mag

    # build123d's `Rot(a, b, c)` composes as Rx(a) · Ry(b) · Rz(c)
    # when applied to a point. With Rz(0) = I:
    #   Rot(a, b, 0) · (0, 0, 1) = Rx(a) · Ry(b) · (0, 0, 1)
    #                            = (sin b, -sin a · cos b, cos a · cos b)
    # Matching component-wise to (nx, ny, nz):
    #   sin b = nx           →  b = asin(nx)
    #   tan a = -ny / nz     →  a = atan2(-ny, nz)   (for cos b > 0)
    beta = math.degrees(math.asin(nx))
    cb = math.cos(math.radians(beta))
    if abs(cb) < 1e-9:
        # Degenerate (local +Z maps onto ±X): any alpha works.
        alpha = 0.0
    else:
        alpha = math.degrees(math.atan2(-ny / cb, nz / cb))
    return Rot(alpha, beta, roll_deg)


def horizontal_x_roll_deg(normal: tuple[float, float, float]) -> float:
    """Roll (`Rz` component) that sends mount-local +X onto a
    world-horizontal direction after `rotation_for_normal(normal)`
    placement. The resulting mount-local +X lies in the plane
    perpendicular to the normal and has zero world-Z component — it
    aligns with the horizontal "tilt axis" of a bracket whose frame
    is `Rot(0, heading, 0)` for this normal's heading.

    Derivation: full rotation matrix is Rx(a)·Ry(b)·Rz(c) per
    `Rot(a,b,c)` semantics. Mount-local +X lands at world
    `(cos b · cos c, cos a · sin c + sin a · sin b · cos c,
      sin a · sin c − cos a · sin b · cos c)`.
    Setting the Z component to zero (under Z-up "horizontal" means
    zero vertical / Z component):
        sin a · sin c − cos a · sin b · cos c = 0
      ⇒ tan c = cos a · sin b / sin a
    Pick the root with `cos c > 0` so `R_11 = cos b · cos c` keeps
    the sign of `cos b`, i.e. mount +X stays on the +X side of the
    horizontal plane for normals with +Z component, matching the
    bracket's +X by convention.
    """
    nx, ny, nz = normal
    n_mag = math.sqrt(nx * nx + ny * ny + nz * nz)
    nx, ny, nz = nx / n_mag, ny / n_mag, nz / n_mag

    beta = math.asin(nx)
    cb = math.cos(beta)
    sb = math.sin(beta)
    if abs(cb) < 1e-9:
        return 0.0
    alpha = math.atan2(-ny / cb, nz / cb)
    sa = math.sin(alpha)
    ca = math.cos(alpha)

    # Root with cos c > 0. tan c = ca · sb / sa.
    num = ca * sb
    den = sa
    if den < 0:
        num, den = -num, -den
    return math.degrees(math.atan2(num, den))


# --- Rig frame (post Phase-2 integrate) --------------------------------
#
# Phase 2 (2026-05-06) collapsed the rev-3 rig T-frame (rails A/B/C/D
# + B↔C corner brace) into the integrated topology described in
# ``integrated_frame_handoff.md``: the turret's back-rail (bumped +20 mm
# in Y to land at m03-local y = +17) plays the role of the old rail A
# (it sits at the same Y and at z = -120 top face vs rail A's -121.2),
# two vertical 2020 posts rise from the back-rail/side-rail corners
# (``arch/``), two short inward stubs at z = +1188 / m03-local +158.1
# bridge each post to rail D's east/west faces, and rail D survives as
# a Y-axis cantilever centred on the arch.
#
# What's left in this section:
#   * the 2020 cross-section constant (still imported by clamp helpers)
#   * the arch post / shared-back-rail anchors (used by ``arch/``,
#     by below-* and back-upper clamp poses, and by the visibility
#     audit)
#   * rail D's centerline / Y-extent (used by ``arch/`` and by the
#     front-low + back-upper clamp poses)
#
# Slice c2 shrink (2026-05-05) sized the rig at r = 160 (lens 3.6 mm,
# tilt 65°). The clamp-Y of +17.0 below ties the new shared back-rail
# Y to the below-* plate-back Y at r = 160 (= vertex_y * (1 + plate_t/r)
# = 16.5 * 1.03125 ≈ 17.0); the clamp's ±10 mm footprint sits centred
# on the back rail.

# Stock 2020 T-slot extrusion cross-section.
SKELETON_RAIL_MEMBER = 20.0

# Shared back rail (= turret back-rail post-Phase-2) — m03-local
# coordinates. The turret's ``frame/`` builder still owns the part;
# these mirrors are the rig-side handle for clamp-Z derivation.
SHARED_BACK_RAIL_Y = 17.0                                # = below-* plate-back Y
SHARED_BACK_RAIL_TOP_Z = -120.0                          # = TURRET_FRAME_FLOOR_Z + MEMBER/2

# Rail D — Y-axis cantilever anchored at the arch crossbar (m03-local
# y = +17). Front cantilever 167 mm (y = -150 → +17, picks up the
# front-low clamp at y ≈ -150); back cantilever 133 mm (y = +17 → +150,
# carries the new back-upper clamp at y ≈ +115.6). The phase-2 back
# end (y = +150) overshoots the handoff's nominal +145 by 5 mm so the
# back-upper clamp's BASE_L/2 = 30 mm footprint at y = 115.6 sits
# fully on rail D with a 4.4 mm margin.
T_FRAME_RAIL_D_Z = 158.1                                 # rail D centerline Z (height)
T_FRAME_RAIL_D_BOTTOM_Z = T_FRAME_RAIL_D_Z - SKELETON_RAIL_MEMBER / 2   # = 148.1
T_FRAME_RAIL_D_TOP_Z = T_FRAME_RAIL_D_Z + SKELETON_RAIL_MEMBER / 2      # = 168.1
T_FRAME_RAIL_D_Y_FRONT = -150.0                          # 5 mm past front-low vertex (y = -145)
T_FRAME_RAIL_D_Y_BACK = +150.0                           # carries back-upper clamp at y ≈ 115.6

# Arch posts — vertical 2020s standing on the back outrigger's +Z
# face, just behind the back rail with a CORNER_GAP butt. Bottom
# face sits CORNER_GAP above the outrigger's top face (= same Z as
# the back rail's top face — both share TURRET_FRAME_FLOOR_Z); top
# face flush with rail D's top face (post grips rail D's east/west
# face indirectly via the inward stub at z = ARCH_CROSSBAR_Z).
#
# X position: the handoff's nominal "back-rail/side-rail corner"
# (x = ±TURRET_FRAME_SIDE_X = ±145). The Phase-2 inboard move to
# ±90 (to dodge the below-* clamp's BASE_L = 60 footprint along the
# back rail at x ∈ [±104.7, ±164.7]) put posts ~7° off the below-*
# lens axes — squarely inside FOV. Restoring ±145 puts posts at
# ~62° off-axis (well outside any reasonable lens FOV) at the cost
# of moving them off the back rail (clamp body still occupies
# x ∈ [±104.7, ±164.7], overlapping the post's x ∈ [±135, ±155]).
#
# Y position: shifted BEHIND the back rail far enough that the post
# body Y ∈ [+45, +65] (post centerline +55) clears (a) the back
# rail body Y ∈ [+7, +27], (b) the platform sweep envelope around
# the hub at m03-local (0, -125), and (c) the below-* mount plate's
# +Y reach. The third constraint is the binding one; details below.
# Three reasons for behind-vs-forward (the slice that flipped this,
# 2026-05-07):
#   1. Platform sweep clearance. The 4-station turret sweeps each
#      platform's outer corners on a circle of radius
#      √((R + P/2)² + (P/2)²) = √(185² + 60²) ≈ 194.5 mm around
#      the hub axis at m03-local (0, -TURRET_ARM_R) = (0, -125). At
#      the prior y = -3.3, the post inner corner sat 175.2 mm from
#      the hub — INSIDE the sweep, so the indexing platform crashed
#      the post. At y = +55, the post inner corner sits ~217 mm
#      from the hub (≈ 22 mm clear).
#   2. Below-* mount plate clearance. The 50 × 50 below-* mount
#      plates sit perpendicular to their lens axes and project onto
#      world +Y by ~25 mm from the plate centre at world Y ≈ +16.8 —
#      plate +Y reach ≈ +41.9. A naïve "just behind the back rail"
#      placement (y = +37.3) put the post body inside that reach;
#      y = +55 keeps post -Y face at +45, ~3 mm clear of plate +Y
#      reach plus CORNER_GAP. (Computed in `arch/assembly.py`'s
#      sweep+plate clearance asserts; see also the 2026-05-07
#      Progress entry in _notes/state/m03-scanner.md for the diagnostic.)
#   3. FOV intrusion. Pushing the post farther in +Y moves it deeper
#      into the back-upper camera's blind hemisphere and farther off
#      every below-* lens axis — both strict improvements over the
#      prior y = -3.3 placement.
# The post no longer rests on the side rail (which ends 0.3 mm
# forward of the back rail). It now stands on the back outrigger's
# +Z face — the outrigger spans m03-local y ∈ [+27.3, +65] (turret-
# local [+152.3, +190]) at x = ±TURRET_FRAME_SIDE_X = ±145, hosting
# the full post body footprint. The outrigger end overhangs the
# m05 deck cylinder edge (Ø660 at world (0, 0)) at world x = ±145
# by ~18 mm; this is acceptable because the deck-foot in a follow-
# up slice bolts at outrigger mid-span (turret-local y ≈ +171.2 =
# world y ≈ +296.2, AT the deck cylinder boundary at x=±145) or
# inboard (e.g. y ≈ +160 = world +285, well inside) — tip-over
# resistance is set by the bolted foot's position, not the rail
# end face. The mass past the deck edge (rail D back tail + arch
# post + back-upper camera) is small and well within the restoring
# moment from the m03 unit's CoM inboard of the foot.
#
# Coordinates m03-local.
ARCH_POST_X = 145.0                                      # = TURRET_FRAME_SIDE_X (outrigger X; below)
ARCH_POST_Y = 55.0                                       # m03-local; clears back rail + sweep + below-* plate +Y reach
ARCH_POST_Z_BOTTOM = SHARED_BACK_RAIL_TOP_Z + 0.3        # = -119.7 (outrigger top + CORNER_GAP)
ARCH_POST_Z_TOP = T_FRAME_RAIL_D_TOP_Z                   # = +168.1

# Arch crossbar plane — the inward stubs sit at this Z (= rail D
# centerline) and at the post Y; they butt against rail D's east /
# west faces with CORNER_GAP. Used by ``arch/assembly.py``.
ARCH_CROSSBAR_Z = T_FRAME_RAIL_D_Z                       # = +158.1
ARCH_CROSSBAR_Y = ARCH_POST_Y                            # = -3.3 (matches post Y)


# --- Per-vertex camera-mount placement ---------------------------------
#
# Four world placements of the single `camera_mount_esp32cam/` body —
# print one body, parameterise it by `mount_in_clamp` for each vertex.
# The clamp is X-Y centred under the plate-back centre so its beam
# runs straight up (or down, for the front-low cantilever-underside
# mount) to the supported plate.
#
# Clamp-local frame (see `camera_mount_esp32cam/assembly.py`):
#     origin : centre of clamp bottom face, on the rail face the clamp
#              grips, directly under the mount plate back-face centre
#              along the rail axis
#     +X     : along rail long axis
#     +Y     : away from rail (clamp body rises in +Y)
#     +Z     : right-handed third axis
#
# Rail assignments (post Phase-2 integrate, 2026-05-06):
#     front-low  → rail D's -Z (underside) face
#     back-upper → rail D's -Z (underside) face  (was rail C's -Y face)
#     below-*    → shared back rail's +Z (top) face  (was rail A's +Z)
#
# Both front-low and back-upper now hang from rail D's underside —
# same `Rot(-90, -90, 0)` clamp pose, different `mount_in_clamp`
# rotations driven by their respective vertex positions. The wedge
# variant of `clamp_mount` (which gripped both rail C's -Y face and
# rail D's -Z face) is gone with rail C.

_FL_BACK = plate_back_centre_world(VERTEX_FRONT_LOW_POS)
_BU_BACK = plate_back_centre_world(VERTEX_BACK_UPPER_POS)
_BL_BACK = plate_back_centre_world(VERTEX_BELOW_LEFT_POS)
_BR_BACK = plate_back_centre_world(VERTEX_BELOW_RIGHT_POS)

# The front-low and back-upper rail clamps both assume an x-symmetric
# pose (centred on rail D), i.e. `_FL_BACK[0]` and `_BU_BACK[0]` are
# 0 — see the `rail_d_centred_vertices` check in `build_camera_rig`.
# A pure-parameter module has no builder to host a claim, so the
# assertion lives with the builder that depends on it rather than
# firing when someone imports this file for an unrelated constant.

# Below-left: clamp on the shared back rail's +Z face, X-centred under
# the mount-back centre. The shared back rail (turret-frame member,
# bumped to m03-local y = +17 in Phase 2) plays the role of the old
# rail A. `Rot(90, 0, 0)` maps clamp-local +X→world +X (along rail),
# +Y→world +Z (clamp body rises), +Z→world -Y.
RAIL_PAD_CLAMP_BELOW_LEFT_POS = (
    _BL_BACK[0],
    _BL_BACK[1],
    SHARED_BACK_RAIL_TOP_Z,
)
RAIL_PAD_CLAMP_BELOW_LEFT_CLAMP_ROT = Rot(90, 0, 0)

# Below-right: X-mirror of below-left. Same rotation; the mount's own
# X-flipped lens axis supplies the chirality, so the printed body is
# the X-mirror of below-left's (different drilled M2 corner positions,
# same overall shape).
RAIL_PAD_CLAMP_BELOW_RIGHT_POS = (
    _BR_BACK[0],
    _BR_BACK[1],
    SHARED_BACK_RAIL_TOP_Z,
)
RAIL_PAD_CLAMP_BELOW_RIGHT_CLAMP_ROT = Rot(90, 0, 0)

# Back-upper: clamp on rail D's -Z (underside) face — Phase-2 swap from
# the rev-3 corner-wedge that gripped rail C's -Y face plus rail D's -Z
# face. With rail C demolished, the back-upper hangs below rail D just
# like the front-low (mirrored about the arch crossbar at y = +17).
# Same `Rot(-90, -90, 0)` clamp pose; the per-instance vertex pose
# supplies the lens axis so one printed `clamp_mount(...)` body covers
# both front-low and back-upper.
RAIL_PAD_CLAMP_BACK_UPPER_POS = (
    _BU_BACK[0],                      # x = 0 (rail D centred on YZ plane)
    _BU_BACK[1],                      # under plate back-face centre
    T_FRAME_RAIL_D_BOTTOM_Z,          # rail D -Z face
)
RAIL_PAD_CLAMP_BACK_UPPER_CLAMP_ROT = Rot(-90, -90, 0)

# Front-low: clamp on rail D's -Z (underside) face. Rail D is the
# Y-cantilever centred on the arch (Phase 2); the camera hangs below
# rail D with its lens axis tilting back-and-up at the brick centroid.
#
# Rot(-90, -90, 0): clamp-local +X→world +Y (along rail D toward back),
# +Y→world -Z (down toward mount), +Z→world -X.
RAIL_PAD_CLAMP_FRONT_LOW_POS = (
    _FL_BACK[0],                      # x = 0 (rail D centred on YZ plane)
    _FL_BACK[1],                      # under plate back-face centre
    T_FRAME_RAIL_D_BOTTOM_Z,          # rail D -Z face
)
RAIL_PAD_CLAMP_FRONT_LOW_CLAMP_ROT = Rot(-90, -90, 0)


# --- Turret geometry ----------------------------------------------------
#
# 4-station rotary indexed turret described in
# `_notes/decisions/m03-turret.md`. Hub axis at the turret-internal origin (the
# turret is placed at world ``Pos(0, +TURRET_ARM_R, 0)`` by
# ``assembly.py``, so turret-internal origin maps to world
# ``(0, +TURRET_ARM_R, *)``). Stations sit at distance TURRET_ARM_R
# from the hub on cardinal axes (option a' renaming, 2026-05-05 — firing
# heading shifted +90° to land DUMP over the diverter hub at world
# (0,0,*)):
#     IMAGE  = (0,    +R, 0)   — heading 90°
#     LOAD   = (+R,   0,  0)   — heading 0°
#     IDLE   = (-R,   0,  0)   — heading 180° (no firing)
#     DUMP   = (0,    -R, 0)   — heading 270° (kicker fires here)
# Brick flow (anti-clockwise spider indexing): LOAD → IMAGE → IDLE →
# DUMP → LOAD. The IDLE dwell is the empty-cycle slot inserted by the
# +90° firing-heading shift; functionally equivalent to the prior
# 3-active layout, slightly longer cycle.
#
# All 4 platforms ride at z = 0. The camera rig's rails sit at
# z = -131 (A) and z = +158 (D), so Z-separation alone keeps the
# turret swept volume clear of the rig.

# Hub axis position in turret-local — origin by definition.
TURRET_HUB_X = 0.0
TURRET_HUB_Y = 0.0

# Arm radius (hub axis to platform centroid). Geometric minimum: at
# radius R the inner corners of two square platforms 90° apart coincide
# when R = PLATFORM_ACRYLIC_SIZE. Slice c3 (2026-05-05) dropped this
# 185 → 125 alongside the platform shrink 180 → 120; 5 mm corner-to-
# corner clearance is the same CORNER_GAP idiom as the prior 185.
# Hub-to-hinge-edge arm reach = TURRET_ARM_R - PLATFORM_ACRYLIC_SIZE/2
# = 65 mm.
TURRET_ARM_R = 125.0

# Station centroid XY positions in turret-local (Z = 0 for all four).
TURRET_STATION_IMAGE_XY = (0.0, +TURRET_ARM_R)
TURRET_STATION_LOAD_XY = (+TURRET_ARM_R, 0.0)
TURRET_STATION_IDLE_XY = (-TURRET_ARM_R, 0.0)
TURRET_STATION_DUMP_XY = (0.0, -TURRET_ARM_R)


# --- Turret frame (slice a) --------------------------------------------
#
# Freestanding 2020 floor frame on the shared bench (z = -220) hosting
# the hub support and providing the M2 → M3 → M5 mechanical interface
# (singulator at LOAD, diverter at DUMP). The back X-rail butts the
# camera rig's rail A front face (CORNER_GAP separator) on the same
# z = -220 bench — that boundary is asserted in ``assembly.py`` after
# the turret is placed in world.
#
# Topology: closed rectangle (4 perimeter rails) plus a center X-rail
# at y = 0 (= TURRET_HUB_Y). The center rail is the mounting line
# shared by the hub bearing (slice b, x = 0), the singulator base
# (LOAD, x = +TURRET_ARM_R) and the diverter base (DUMP, x =
# -TURRET_ARM_R) — putting all three M2/M3/M5 anchors on one rail
# keeps their mounting holes coplanar with the cross-rail's T-slot.

TURRET_FRAME_FLOOR_Z = -130.0                                # turret floor

# Slice c4 (2026-05-05) raised the floor 40 mm (-220 → -180) in
# lockstep with the hub housing shrink 100 → 60 mm (TURRET_HUB_
# HOUSING_HEIGHT in turret_hub/assembly.py). Net effect: housing top
# stayed at z = -110, the cam/pulley/motor stack above unchanged in
# absolute Z, and the turret 40 mm shorter overall.
#
# Slice c5 (2026-05-05) raised the floor a further 30 mm (-180 → -150)
# without shrinking the housing. The housing top therefore rose
# 30 mm to z = -80, closing the housing-top → pulley-bottom gap from
# 20 to 1 mm; the pulley rose 11 mm in lockstep (PULLEY_Z_CENTER
# -86 → -75 in drive) so the housing-pulley airspace stays at 1 mm.
# The retired shaft-mounted dump cam rode the pulley up too.
#
# Slice c7 (2026-05-05) raised the floor a further 20 mm (-150 → -130)
# so the turret floor rails sit at the same Z band as the camera rig's
# rail A — turret rails span Z[-140, -120] vs rail A's [-141.2, -121.2]
# (1.2 mm offset because rail A was bumped a sliver to the
# conservative-envelope value). Housing top rose 20 mm to z = -60
# (housing height unchanged at 60); pulley center -75 → -55, preserving
# the 1 mm housing-pulley airspace; motor body rose 20 mm (flange
# -88 → -68, body bottom -128 → -108); shaft shortened 130 → 110. The
# retired shaft cam closed the cam-top → spider-bottom gap to 3 mm at
# this floor — the marker for the floor-rise stopping point at the
# time. Brick-silhouette occlusion from the motor was rechecked by
# inspection (motor at world x=+59, outboard of below-vertex
# sightlines: line from below_right at world (130.6, 16.5, -90.9) to
# brick at origin passes through x≈95 at the new motor-top Z=-68,
# well outboard of motor body's x=+80 face). The formal silhouette-
# cone assertion (per the spec's c5b retrofit) is deferred to a
# follow-up slice that adds sightline-rod stub parts.
#
# Slice 2026-05-23 retired the shaft-mounted dump cam and bench
# kicker (see `_notes/archive/m03-dump-servo-handoff.md`) — neither is on the shaft
# stack any more, so the compression budget no longer hinges on
# cam-spider clearance. The dump now fires from a bench-mounted
# hobby servo (`dump_servo/`); housing/pulley clearances are still
# load-bearing.
#
# The project workbench reference (SHARED_BENCH_Z = -220 above)
# didn't move; the turret floor now floats 90 mm above it, mirroring
# how the camera rig has floated since c2.

# Center X-rail Y (= hub axis Y = 0 in turret-local). Hosts hub
# bearing + neighbour-module anchors on a single line.
TURRET_FRAME_CROSS_Y = TURRET_HUB_Y                          # = 0

# Back X-rail centerline Y in turret-local. Phase 2 (2026-05-06)
# bumped this +20 mm (122 → 142) so the rail's centerline lands at
# m03-local y = +17 = SHARED_BACK_RAIL_Y, the below-* plate-back Y.
# The integrated frame collapses the rev-3 rig's rail A into this
# turret member: below-left/right clamps now grip the rail's +Z face
# (still the same X-symmetric pose; the 1.2 mm rise from rail A's
# top z = -121.2 to back-rail top z = -120 is absorbed by the
# clamp Z parameter at SHARED_BACK_RAIL_TOP_Z above).
#
# Slices c4–c5 raised the turret floor -220 → -180 → -150 to absorb
# hub-housing shrinks; slice c7 (2026-05-05) raised it further to
# z = -130. Phase 2's +20 mm Y bump and the demolition of rail A
# eliminate the prior butt-joint Y constraint between the rev-3 rig
# rail A (m03-local y = +17) and the rev-4 turret back rail (m03-
# local y = -3 with corresponding face at +7) — the two rails are
# now one member at y = +17.
#
# The IMAGE platform's outer +Y edge sits at turret-local y = ARM_R
# + PLATFORM/2 = 185, cantilevering 43 mm past this rail (= 185 -
# 142). Acceptable because all 4 platforms hang off the spider arm
# at the hub, not off the floor frame; the back rail is a perimeter
# rigidifier and the mounting line for the below-* camera clamps
# (Phase 2) plus the arch posts (`arch/`) at x = ±145.
TURRET_FRAME_BACK_Y = 142.0                                  # = SHARED_BACK_RAIL_Y + ARM_R

# Front X-rail centerline Y — DROPPED in option a' (2026-05-05).
# The new DUMP station lives at turret-local (0, -R, 0); a rail at
# y = -R would sit world-y = 0 over the diverter top column body and
# block the brick drop path. The frame is now a U-shape (back rail +
# 2 side rails + cross rail) closed at the back; side rails cantilever
# ~115 mm forward of the cross-rail. Acceptable because all 4 turret
# platforms hang off the spider arm at the hub (not the floor frame),
# the cross-rail still carries the hub housing, and the back perimeter
# remains rigid. The constant is retained as a -R reference in case
# downstream code reads it; nothing builds a member at this Y any more.
TURRET_FRAME_FRONT_Y = -TURRET_ARM_R                          # = -125 (no member; reference only)

# Side Y-rails X. Sits 20 mm outboard of the LOAD/DUMP centroid lines
# at x = ±TURRET_ARM_R — gives the singulator and diverter mounting
# room on the cross-rail's outboard end without the side rail's outer
# face crashing the swept platform footprint at LOAD/DUMP (LOAD
# platform's outboard X face at +R+60 is at z = 0, well above the
# z = -220 side rail).
TURRET_FRAME_SIDE_X = TURRET_ARM_R + 20.0                     # = 145

# Cross-check: ARCH_POST_X (defined earlier, before TURRET_ARM_R) is
# the literal 145 and must track TURRET_FRAME_SIDE_X, so a future
# change to TURRET_ARM_R can't silently desync the arch post position
# from the side-rail corner it lands on. Asserted in `build_arch`,
# which is what places the post — a pure-parameter module has no
# builder, and an import-time assert here would fire on anyone reading
# an unrelated constant out of this file.


# --- Turret hub (slice b) ----------------------------------------------
#
# Stock 8 mm hardened steel shaft. Selected to match 608ZZ bearing ID.
TURRET_SHAFT_D = 8.0

# Shaft top Z in world. 100 mm above the housing top face — leaves
# room for the slice (c) arm spider to clamp around the shaft above
# the top bearing. The camera sightline at y = -130 covers world
# z > -29 (camera-to-brick line passes through z ≈ +60.7 at this y,
# with a ~90 mm cone half-radius), so any structure above z ≈ -30
# here appears in the camera frame. The slice (c) spider top is
# flush with the shaft top, so the spider sits in the camera frame
# corner; the classifier learns to ignore it.
TURRET_SHAFT_TOP_Z = -10.0


# --- Turret arm (slice c) — hinge interface ----------------------------
#
# Hinge pin axis Z (world). Chosen so the spider's top face sits
# flush with the shaft top (z = -10) and the pin lies on the spider
# centerline: top - SPIDER_HEIGHT/2 = -10 - 25/2 = -22.5. The
# slice (d) platform's bonded hinge brackets reach from acrylic
# underside (z = -4) down to this pin Z, ~18.5 mm tall.
TURRET_HINGE_PIN_Z = -22.5

# Hinge pin radial offset from hub axis. The platform centroid sits
# at TURRET_ARM_R from the hub; its hub-side edge is
# PLATFORM_ACRYLIC_SIZE/2 = 60 closer in. So the hinge pin axis at
# the inner edge sits at radial TURRET_ARM_R - 60 from hub axis.
TURRET_HINGE_PIN_RADIAL = TURRET_ARM_R - PLATFORM_ACRYLIC_SIZE / 2  # = 65

# Hinge pin diameter — M5 stock round (matches the rev-3
# PLATFORM_HINGE_PIN_D so existing M5 round stock applies).
TURRET_HINGE_PIN_D = 5.0


# --- Turret dump servo (bench-mounted hobby servo at DUMP) -------------
#
# MCU-triggered hobby servo (SG90 class) bench-mounted at the DUMP
# heading, driving a short lifter arm that pushes up on the DUMP-pose
# platform extension's drop block from below. Replaces the retired
# shaft cam + bench kicker pair (`dump_cam/`, `dump_kicker/`) — see
# `_notes/archive/m03-dump-servo-handoff.md` for the rationale and `_notes/decisions/m03-turret.md`
# §6.1 for the brief.
#
# Why a servo instead of a cam: the M3 control board already runs an
# MCU for rotor indexing + beam-break sensing, so one extra PWM pin
# is free. The cam approach added a kinematic synchronization puzzle
# (cam lobe headings vs spider arm headings on the same shaft) that
# the face-cam slice tried and failed to resolve; the servo eliminates
# the puzzle entirely. Trade: dump timing is firmware-defined rather
# than mechanically locked to the indexer.
#
# Anchor:
#   * Pivot (= servo output shaft axis) at turret (-80, -25, -100).
#     Output shaft along +Y so the body extends in -Y; the lifter arm
#     sweeps in the X-Z plane around the Y-axis.
#   * X = -80 puts the body clear of the homing bracket
#     (X ∈ [-62.5, -31.5] on the cross-rail top) and the hub housing
#     (X ∈ [-30, +30]).
#   * Y = -25 is the lifter pad target Y (drop block centroid in
#     turret-local at DUMP heading 270°).
#   * Z = -100 puts the body bottom flush with an 8-mm-thick bracket
#     foot sitting on rail top (z = -120). Arm reaches 52 mm vertical
#     and 50 mm horizontal from pivot to pad — a ~72 mm slanted beam.
#
# Lifter pad TOP-FACE Z at rest stays at z = -45, 2 mm below the
# platform extension's bottom at z = -43 (slice (d) bracket bottom
# inherits the LIFTER_TOP_Z value via the assertion in
# `turret_platform/assembly.py`). Pad XY (turret-local) preserved from
# the kicker era so the platform extension geometry doesn't move.
TURRET_DUMP_SERVO_PIVOT_X = -80.0
TURRET_DUMP_SERVO_PIVOT_Y = -25.0
TURRET_DUMP_SERVO_PIVOT_Z = -100.5

# Lifter pad TOP-FACE Z at rest. 2 mm below the platform extension's
# bottom at z = -43 — clears the swept platform footprint so a non-
# DUMP-station platform indexing through doesn't crash the pad.
TURRET_DUMP_SERVO_LIFTER_TOP_Z = -45.0

# Lifter pad XY in turret-local. Same mapping the retired kicker used
# (option a', 2026-05-05): cart-frame extension-tip (X=+30, Y=-100)
# under DUMP rotation Rz(+180) + centroid translate (0, -ARM_R) gives
# turret (-30, -25). Kept for consistency with the platform's
# integrated drop block — `turret_platform/assembly.py` cross-checks
# these against EXTENSION_X_CENTER and LIFT_POINT_Y_LOCAL.
TURRET_DUMP_SERVO_LIFTER_TURRET_X = -30.0
TURRET_DUMP_SERVO_LIFTER_TURRET_Y = -25.0
