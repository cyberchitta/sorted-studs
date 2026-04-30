"""
M03 turret — unit-local constants.

The turret is the rev-4 rotary 4-station platform stack hanging off
its own hub axis. Its local frame uses the hub axis as the origin
with `+Y` vertical:

    origin = hub axis at the cross-rail's top face Y plane
    +X     = operator's right            (parallel to world +X)
    +Y     = up                          (parallel to world +Y)
    +Z     = away from operator          (parallel to world +Z)

The unit is placed in world by `Pos(0, 0, -TURRET_ARM_R)` in
`../assembly.py` — pure translation, no rotation. That places hub at
world `(0, *, -TURRET_ARM_R) = (0, *, -185)`, IMAGE platform at world
origin, and the four station centroids at world
`(±R, 0, -R)` / `(0, 0, -R±R)`.

Each turret sub-assembly (`turret_frame/`, `turret_hub/`,
`turret_arm/`, `turret_platform/`) carries its own local frame and
imports the constants below via `from turret.params import …`.
"""

# --- Print floor (matches project-wide value in ../params.py) ----------

WALL_MIN = 1.5


# --- Acrylic platform stock (matches ../params.py) ---------------------
#
# 180 × 180 × 4 mm PMMA. Used by all 4 turret platform cartridges
# (slice d) and by the rev-3 single-platform stack. The duplicate of
# the top-level value is intentional — it lets `turret/` stand alone
# without reaching upward; the top-level assembly asserts agreement.

PLATFORM_ACRYLIC_SIZE = 180.0
PLATFORM_ACRYLIC_T = 4.0


# --- Turret geometry ----------------------------------------------------
#
# 4-station rotary indexed turret described in
# `../turret_design_brief.md`. Hub axis at the unit-local origin.
# Stations sit at distance TURRET_ARM_R from the hub on cardinal axes:
#     IMAGE  = (0,    0, +R)   — toward world +Z (back) when placed
#     LOAD   = (+R,   0,  0)   — toward world +X (operator-right)
#     DUMP   = (-R,   0,  0)   — toward world -X (operator-left)
#     RETURN = (0,    0, -R)   — toward world -Z (operator-forward)
#
# All 4 platforms ride at y = 0. The camera rig's rails sit at
# y = -220 (A/B) and y = +250 (D), so Y-separation alone keeps the
# turret swept volume clear of the rig.

# Hub axis position in turret-local — origin by definition.
TURRET_HUB_X = 0.0
TURRET_HUB_Z = 0.0

# Arm radius (hub axis to platform centroid). The design brief's
# original 100–150 mm range was wrong: at radius R the inner corners
# of two 180 × 180 mm platforms 90° apart only clear when R ≥ 180 mm
# (geometric minimum: corner at (S/2, S/2) coincides with adjacent
# platform's corner at (S/2, S/2) when R = S = 180). 185 mm gives
# 5 mm corner-to-corner clearance — same idiom as CORNER_GAP elsewhere.
# Hub-to-hinge-edge arm reach = TURRET_ARM_R - PLATFORM_ACRYLIC_SIZE/2
# = 95 mm.
TURRET_ARM_R = 185.0

# Station centroid XZ positions in turret-local (Y = 0 for all four).
TURRET_STATION_IMAGE_XZ = (0.0, +TURRET_ARM_R)
TURRET_STATION_LOAD_XZ = (+TURRET_ARM_R, 0.0)
TURRET_STATION_DUMP_XZ = (-TURRET_ARM_R, 0.0)
TURRET_STATION_RETURN_XZ = (0.0, -TURRET_ARM_R)


# --- Turret frame (slice a) --------------------------------------------
#
# Freestanding 2020 floor frame on the shared bench (y = -220) hosting
# the hub support and providing the M2 → M3 → M5 mechanical interface
# (singulator at LOAD, diverter at DUMP). The back X-rail butts the
# camera rig's rail A front face (CORNER_GAP separator) on the same
# y = -220 bench — that boundary is asserted in the top-level
# `../assembly.py` after the turret is placed in world.
#
# Topology: closed rectangle (4 perimeter rails) plus a center X-rail
# at z = 0 (= TURRET_HUB_Z). The center rail is the mounting line
# shared by the hub bearing (slice b, x = 0), the singulator base
# (LOAD, x = +TURRET_ARM_R) and the diverter base (DUMP, x =
# -TURRET_ARM_R) — putting all three M2/M3/M5 anchors on one rail
# keeps their mounting holes coplanar with the cross-rail's T-slot.

TURRET_FRAME_FLOOR_Y = -220.0                                # shared bench

# Center X-rail Z (= hub axis Z = 0 in turret-local). Hosts hub
# bearing + neighbour-module anchors on a single line.
TURRET_FRAME_CROSS_Z = TURRET_HUB_Z                          # = 0

# Back X-rail centerline Z in turret-local. The +Z face must butt the
# camera rig's rail A front face when placed at world Z = -ARM_R.
# Numerical: world rail A front Z = +18.44; world placement adds
# -ARM_R to turret-local Z, so turret-local rail A front Z = ARM_R +
# 18.44; back rail back face = ARM_R + 18.44 - CORNER_GAP = ARM_R +
# 18.14; back rail centerline = back face - MEMBER/2 = ARM_R + 8.14.
# The top-level `../assembly.py` asserts this matches the rig's rail A.
TURRET_FRAME_BACK_Z = TURRET_ARM_R + 8.14

# Front X-rail centerline Z — under the RETURN station centroid
# (turret-local (0, 0, -R)). The RETURN platform's outer edge
# cantilevers PLATFORM_ACRYLIC_SIZE/2 forward of this rail (to
# z = -R - 90 = -275 at R = 185); accepted because the platform is
# light, hangs off the spider arm rather than the floor, and stopping
# the floor here keeps the bench footprint bounded.
TURRET_FRAME_FRONT_Z = -TURRET_ARM_R                          # = -185

# Side Z-rails X. Sits 20 mm outboard of the LOAD/DUMP centroid lines
# at x = ±TURRET_ARM_R — gives the singulator and diverter mounting
# room on the cross-rail's outboard end without the side rail's outer
# face crashing the swept platform footprint at LOAD/DUMP (LOAD
# platform's outboard X face at +R+90 is at y = 0, well above the
# y = -220 side rail).
TURRET_FRAME_SIDE_X = TURRET_ARM_R + 20.0                     # = 205


# --- Turret hub (slice b) ----------------------------------------------
#
# Stock 8 mm hardened steel shaft. Selected to match 608ZZ bearing ID.
TURRET_SHAFT_D = 8.0

# Shaft top Y in world. 100 mm above the housing top face — leaves
# room for the slice (c) arm spider to clamp around the shaft above
# the top bearing. The camera sightline at z = -130 covers world
# y > -29 (camera-to-brick line passes through y ≈ +60.7 at this z,
# with a ~90 mm cone half-radius), so any structure above y ≈ -30
# here appears in the camera frame. The slice (c) spider top is
# flush with the shaft top, so the spider sits in the camera frame
# corner; the classifier learns to ignore it.
TURRET_SHAFT_TOP_Y = -10.0


# --- Turret arm (slice c) — hinge interface ----------------------------
#
# Hinge pin axis Y (world). Chosen so the spider's top face sits
# flush with the shaft top (y = -10) and the pin lies on the spider
# centerline: top - SPIDER_HEIGHT/2 = -10 - 25/2 = -22.5. The
# slice (d) platform's bonded hinge brackets reach from acrylic
# underside (y = -4) down to this pin Y, ~18.5 mm tall.
TURRET_HINGE_PIN_Y = -22.5

# Hinge pin radial offset from hub axis. The platform centroid sits
# at TURRET_ARM_R from the hub; its hub-side edge is
# PLATFORM_ACRYLIC_SIZE/2 = 90 closer in. So the hinge pin axis at
# the inner edge sits at radial TURRET_ARM_R - 90 from hub axis.
TURRET_HINGE_PIN_RADIAL = TURRET_ARM_R - PLATFORM_ACRYLIC_SIZE / 2  # = 95

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
# Disc Y span: [-81, -69] (centered on TURRET_DUMP_CAM_CENTER_Y).
# Sits between the housing top (y = -110) and the slice (c) spider
# bottom (y = -35), leaving 29 mm below for a future drive coupling
# and 34 mm above before the spider. Lobe-peak / lobe-ramp angular
# split + the polygon discretisation are private to the sub-assembly
# (`turret_dump_cam/assembly.py`).
TURRET_DUMP_CAM_BASE_R = 25.0
TURRET_DUMP_CAM_PEAK_R = 30.0
TURRET_DUMP_CAM_HEIGHT = 12.0
TURRET_DUMP_CAM_CENTER_Y = -75.0
TURRET_DUMP_CAM_TOP_Y = TURRET_DUMP_CAM_CENTER_Y + TURRET_DUMP_CAM_HEIGHT / 2
TURRET_DUMP_CAM_BOTTOM_Y = TURRET_DUMP_CAM_CENTER_Y - TURRET_DUMP_CAM_HEIGHT / 2
