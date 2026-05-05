"""
M03 turret — unit-local constants.

The turret is the rev-4 rotary 4-station platform stack hanging off
its own hub axis. Its local frame uses the hub axis as the origin
with `+Z` vertical:

    origin = hub axis at the cross-rail's top face Z plane
    +X     = operator's right            (parallel to world +X)
    +Y     = away from operator          (parallel to world +Y)
    +Z     = up                          (parallel to world +Z)

The unit is placed in world by `Pos(0, +TURRET_ARM_R, 0)` in
`../assembly.py` (option a', 2026-05-05) — pure translation, no
rotation. That places hub at world `(0, +TURRET_ARM_R, *) = (0,
+125, *)`, DUMP platform at world origin (firing heading 270° in
turret-local), and the four station centroids at world `(0, +R±R,
0)` / `(±R, +R, 0)`.

Each turret sub-assembly (`turret_frame/`, `turret_hub/`,
`turret_arm/`, `turret_platform/`) carries its own local frame and
imports the constants below via `from turret.params import …`.
"""

# --- Print floor (matches project-wide value in ../params.py) ----------

WALL_MIN = 1.5


# --- Acrylic platform stock (matches ../params.py) ---------------------
#
# 120 × 120 × 4 mm PMMA (slice c3 shrink, 2026-05-05). Sized to the
# rev-5 4-cam imaged region (110 × 110) + 5 mm PMMA edge grip per side
# per `sizing.py`. Used by all 4 turret platform cartridges (slice d)
# and by the rev-3 single-platform stack. The duplicate of the
# top-level value is intentional — it lets `turret/` stand alone
# without reaching upward; the top-level assembly asserts agreement.

PLATFORM_ACRYLIC_SIZE = 120.0
PLATFORM_ACRYLIC_T = 4.0


# --- Turret geometry ----------------------------------------------------
#
# 4-station rotary indexed turret described in
# `../turret_design_brief.md`. Hub axis at the unit-local origin.
# Stations sit at distance TURRET_ARM_R from the hub on cardinal axes
# (option a' renaming, 2026-05-05 — firing heading shifted +90° to land
# DUMP over the diverter hub at world (0,0,*)):
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
# z = -220 (A/B) and z = +250 (D), so Z-separation alone keeps the
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
# z = -220 bench — that boundary is asserted in the top-level
# `../assembly.py` after the turret is placed in world.
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
# 20 to 1 mm; the pulley and cam disc rose 11 mm in lockstep
# (PULLEY_Z_CENTER -86 → -75 in turret_drive, TURRET_DUMP_CAM_CENTER_Z
# -75 → -64 below) so the cam-bottom → pulley-top gap stays at 1 mm
# and the cam-top → spider-bottom gap closes from 34 to 23 mm.
#
# Slice c7 (2026-05-05) raised the floor a further 20 mm (-150 → -130)
# so the turret floor rails sit at the same Z band as the camera rig's
# rail A — turret rails span Z[-140, -120] vs rail A's [-141.2, -121.2]
# (1.2 mm offset because rail A was bumped a sliver to the
# conservative-envelope value). Housing top rose 20 mm to z = -60
# (housing height unchanged at 60); pulley center -75 → -55 and cam
# center -64 → -44 in lockstep, preserving the 1 mm pulley-top →
# cam-bottom airspace; motor body rose 20 mm (flange -88 → -68, body
# bottom -128 → -108); shaft shortened 130 → 110. Cam-top →
# spider-bottom gap closes from 23 to 3 mm (cam top -38 vs spider
# bottom -35) — most of the available compression along this axis,
# stopping 2 mm shy of the 1 mm floor that further floor-rise budgets
# would imply. The kicker pivot rose 31 mm (TURRET_DUMP_KICKER_PIVOT_Z
# below) — more than the 20 mm floor rise — to lift the lever's hub
# disc clear of the cross-rail and the kicker_mount foot. Brick-
# silhouette occlusion from the motor was rechecked by inspection
# (motor at world x=+59, outboard of below-vertex sightlines: line
# from below_right at world (130.6, 16.5, -90.9) to brick at origin
# passes through x≈95 at the new motor-top Z=-68, well outboard of
# motor body's x=+80 face). The formal silhouette-cone assertion
# (per the spec's c5b retrofit) is deferred to a follow-up slice
# that adds sightline-rod stub parts.
#
# The project workbench reference (SHARED_BENCH_Z = -220 in
# ../params.py) didn't move; the turret floor now floats 90 mm above
# it, mirroring how the camera rig has floated since c2.

# Center X-rail Y (= hub axis Y = 0 in turret-local). Hosts hub
# bearing + neighbour-module anchors on a single line.
TURRET_FRAME_CROSS_Y = TURRET_HUB_Y                          # = 0

# Back X-rail centerline Y in turret-local. Sized so the rail's +Y
# face is flush with the camera rig's rail A -Y face at world Y = +7
# (= T_FRAME_RAIL_A_Y - MEMBER/2 = 17 - 10). With the turret placed
# at world Pos(0, -ARM_R, 0), turret-local +Y face = world face +
# ARM_R = 7 + 125 = 132, and centerline = 132 - MEMBER/2 = 122.
#
# Slices c4–c5 raised the turret floor -220 → -180 → -150, narrowing
# the vertical gap between the turret back rail's top face (z = -140
# after c5) and the camera rig's rail A bottom face (z = -141.2) to
# only 1.2 mm. The rev-3/4-era butt-joint constraint that c2 marked
# "moot" (after the rig shrink lifted rail A to z=-131) is therefore
# back in force as a Y-axis face-flush constraint: any further floor
# rise would crash the two rails unless they no longer overlap in Y.
# Slice c3's interim "+10 mm CORNER_GAP-class margin past the IMAGE
# platform centroid" (= ARM_R + 10 = 135) put the rail directly under
# rail A in plan view; c6 (2026-05-05) restored the boundary.
#
# Slice c7 (2026-05-05) raised the floor a further 20 mm to z = -130.
# The back-rail top face is now at z = -110, *above* rail A's bottom
# face at z = -141.2 — the two rails now overlap in Z fully (turret
# rail Z [-140, -120] ≈ rail A Z [-141.2, -121.2], 1.2 mm offset) in
# addition to their existing plan-view overlap. The Y butt-flush
# constraint (turret back-rail's +Y face at world y=+7 tangent to
# rail A's -Y face at world y=+7) is what keeps them out of
# interference; they share a face but no volume.
#
# The IMAGE platform's outer +Y edge sits at y = ARM_R + PLATFORM/2 =
# 185, cantilevering 63 mm past this rail (was 50 mm). Acceptable
# because the platform hangs off the spider arm at the hub, not off
# the floor frame; the back rail is a perimeter rigidifier and a
# mounting line for nothing back-of-hub.
TURRET_FRAME_BACK_Y = 122.0                                  # rail-A boundary mirror

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


# --- Turret dump cam (slice f1 — shaft cam + bench kicker) -------------
#
# Printed cam disc keyed to the slice (b) shaft. Drives the bench-
# mounted DUMP kicker (slice f2) via a follower lever: 4 radial lobes
# 90° apart aligned with the spider arm headings (LOAD/IMAGE/DUMP/
# RETURN). At every indexed dwell, one lobe sits under the bench-fixed
# follower at heading 180° and pushes the kicker outward to lift the
# DUMP-station platform's tab; in transit the follower drops into the
# valley between lobes and the kicker retracts.
#
# Profile (4-fold rotational symmetry):
#   - base radius (valley)        TURRET_DUMP_CAM_BASE_R = 25
#   - peak radius (lobe top)      TURRET_DUMP_CAM_PEAK_R = 30
#   - 5 mm lobe rise sets the follower's nominal radial stroke; the
#     slice (f2) kicker linkage maps that radial motion to the
#     vertical lifter stroke needed for the platform tilt.
#
# Disc Z span: [-50, -38] (centered on TURRET_DUMP_CAM_CENTER_Z).
# Sits between the housing top (z = -60) and the slice (c) spider
# bottom (z = -35), leaving 1 mm below before the pulley top (z=-51)
# and 3 mm above before the spider. Slice c5 (2026-05-05) raised the
# cam 11 mm (-75 → -64 center) along with the pulley to absorb the
# housing rise; slice c7 (2026-05-05) raised it a further 20 mm
# (-64 → -44 center), closing the cam-top → spider-bottom gap from
# 23 mm to 3 mm. The remaining 2 mm of along-shaft compression budget
# is held back: the lifter beam already clips the cam BASE_R footprint
# at heading ~120° (documented at integrate level) and tightening the
# spider-cam gap further compounds that without addressing the
# underlying redesign. Cam-bottom → pulley-top still 1 mm (pulley
# climbed in lockstep). Lobe-peak / lobe-ramp angular split + the
# polygon discretisation are private to the sub-assembly
# (`turret_dump_cam/assembly.py`).
TURRET_DUMP_CAM_BASE_R = 25.0
TURRET_DUMP_CAM_PEAK_R = 30.0
TURRET_DUMP_CAM_HEIGHT = 12.0
TURRET_DUMP_CAM_CENTER_Z = -44.0
TURRET_DUMP_CAM_TOP_Z = TURRET_DUMP_CAM_CENTER_Z + TURRET_DUMP_CAM_HEIGHT / 2
TURRET_DUMP_CAM_BOTTOM_Z = TURRET_DUMP_CAM_CENTER_Z - TURRET_DUMP_CAM_HEIGHT / 2


# --- Turret dump kicker (slice f2 — bench-mounted lever at DUMP) -------
#
# Bench-mounted 3D L-lever pivoted on a printed bracket bolted to the
# slice (a) cross-rail's top T-slot at turret (x = TURRET_DUMP_KICKER_
# PIVOT_X, y = 0). The lever's hub-end (in the y = 0 plane, in line
# with the cam disc) picks off the slice (f1) shaft cam's OD at heading
# 180° (the -X side, between hub and IDLE); a -Y cantilever (option a',
# 2026-05-05) then carries the lifter pad to turret (x = -30, y = -25)
# so the pad lands under the tip of the slice (d) platform frame's
# inward extension at the new DUMP station (heading 270°, turret-local
# centroid (0, -R, 0)). At rest (cam valley between dwells) the
# follower sits at radial BASE_R = 25 from the cam axis and the lifter
# pad is retracted; at every dwell a cam lobe rotates into heading 180°
# and pushes the follower outward to PEAK_R = 30 (5 mm radial stroke),
# tilting the lever and raising the lifter pad against the extension
# tip at DUMP.
#
# The cam follower heading is independent of the firing-station
# heading: the shaft cam is 4-fold symmetric, so a peak rotates into
# every heading at every dwell. Picking the follower at heading 180°
# (where the existing mount sits) is a free choice; it leaves the
# clevis bracket bolted to the cross-rail at turret (-110, 0)
# regardless of which station fires.
#
# Lifter / extension-tip position. The slice (d) platform frame
# extends one corner inward (in cart -Y direction) past the hinge to
# a lift point at cart-frame (X=+30, Y=-100) — 40 mm hub-side of the
# hinge axis (cart Y=-60), giving a clean single-direction torque
# about the hinge with no competing bonded brick-side push. Cart X=+30
# keeps the extension well inside the rev-5 acrylic and clear of the
# slice (c) DUMP spider arm at cart X ∈ [-7, +7]. Under option a',
# the firing station (DUMP at heading 270°) carries the cart-local
# IMAGE pose to turret-local via Rz(+180) (= the prior RETURN
# station's rotation): cart (X=+30, Y=-100) maps to (-30, +100),
# then translates by the DUMP centroid (0, -125) to turret
# (-30, -25), which is where the lifter lives in turret-local.
#
# This slice ships REST geometry only — static placement of mount,
# lever, pin. Cam-follower kinematics under shaft rotation (lever
# rotation angle vs cam profile, follower roller geometry, peak-stroke
# clearance to extension tip) are deferred until slices f3/f4 add the
# per-arm hinge stop and the platform extension; slice f5 integrate
# ties the kinematics together.
#
# Pivot location: x outboard of the homing bracket region
# (X∈[-62.5,-31.5]) so the kicker foot can sit at the same X∈[-140,-80]
# the homing slice's KICKER_FOOT_X_INNER mirror assumes; z above cross-
# rail top (after c7: -120) plus a clearance for the lever's hub disc
# (R = LEVER_HUB_DISC_OD/2 = 12 in turret_dump_kicker/assembly.py).
# Axis along ±Y so the hub-arm rotates in the X-Z plane; bridge-Y
# cantilevers in -Y (option a', 2026-05-05) to reach the lifter at
# y=-25. PIVOT_X stays at -110.
#
# Slice c7 (2026-05-05) raised this 31 mm (-130 → -99), more than the
# 20 mm floor rise that motivated the slice. Geometry: the lever's
# hub disc is centred on the pivot in Y∈[-6,+6] (in the clevis gap)
# but extends radially R=12 mm in the X-Z plane. With the foot top
# climbing to z=-112 (rail_top -120 + foot_z 8) and the cross-rail
# top at z=-120, the disc bottom (= pivot - R) must land ≥ -111 to
# leave 1 mm clearance to the foot top: pivot ≥ -112 + R + 1 = -99.
# A latent c5 bug (kicker_lever overlapping cross_rail by 216 mm³ and
# kicker_mount foot by 1904 mm³, both because the c5 floor rise
# climbed toward the static pivot) is folded into this c7 fix —
# c5/c6's "khana check passes" claim was true only at the standalones
# that use stale local mirrors, not at turret/assembly.py-level. The
# 11 mm gain over the spec's nominal "rises 20 mm" is the explicit
# fix for that bug.
TURRET_DUMP_KICKER_PIVOT_X = -110.0
TURRET_DUMP_KICKER_PIVOT_Y = 0.0
TURRET_DUMP_KICKER_PIVOT_Z = -99.0

# Lifter pad TOP-FACE Z at rest. 17.7 mm below the slice (d) platform
# bracket bottom (z = -27.3) — clear of the swept platform footprint at
# any station so the lever-at-rest doesn't crash an indexing platform
# that's not yet at DUMP. Slice (d)'s integrated extension drops down
# to z = -43 at the lift point; the 2 mm REST clearance lets the cam
# stroke close the gap and then push the platform.
TURRET_DUMP_KICKER_LIFTER_TOP_Z = -45.0

# Lifter pad XY in turret-local. Resolves the cart-frame extension-tip
# position (X=+30, Y=-100) under the DUMP rotation Rz(+180) (option a',
# 2026-05-05 — same rotation the prior RETURN station used):
#     turret_x = -cart_x         =         -30
#     turret_y = -ARM_R - cart_y = -125 - (-100) = -25
# Lever arm = |cart_y - hinge_cart_y| = |-100 - (-60)| = 40 mm hub-side
# of the hinge — single-direction torque (no competing bonded brick-side
# push as in the prior straddle-the-hinge tab design). Slice c3
# (2026-05-05) shrunk the cart-frame coords with the platform: cart X
# from +80 → +30 (well inside the new 120 acrylic; spider arm at ±7
# unchanged), cart Y from -130 → -100 (preserving the 40 mm lever
# arm against the new hinge at cart Y=-60). Option a' (2026-05-05)
# kept the cart-frame coords and switched the firing station from
# heading 180° (Rz(+90), turret (-25, +30)) to heading 270° (Rz(+180),
# turret (-30, -25)).
TURRET_DUMP_KICKER_LIFTER_TURRET_X = -30.0
TURRET_DUMP_KICKER_LIFTER_TURRET_Y = -25.0
