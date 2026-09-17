"""Shared interface constants for the M02 C-channel singulator.

Top-level geometry that multiple sub-assemblies (rotor, stator, drive,
exit_sensor) must agree on. Sub-assembly-internal details stay in that
sub-assembly's file.

Cross-module inputs:
- #channel_v_angle (_notes/specs/parameters.md) → V_HALF_ANGLE_DEG
- #channel_wall_height                  → STATOR_WALL_H
- #print_clearance                      → MOTOR_SHAFT_BORE_CLEARANCE

Vertical stack below the V-bottom plane (z = 0), top to bottom — the
rim drive of `_notes/decisions/m02-rotor-drive.md`:

    z = 0                      rotor base rim / stator bottom / V-bottom
    z ∈ [-10,   0]             rotor flange, ring-gear teeth on its OD
    z ∈ [-9,   -1]             pinion face (meshes with the flange OD)
    z ∈ [-18.5, -10]           turntable bearing (bought)
    z ∈ [-24.5, -18.5]         bowl floor = stator foot ring (§5)
    z ∈ [-64.5, -24.5]         NEMA 17 body, hanging under the floor

The bowl (2026-09-16, `_notes/decisions/m02-chain.md` §5): the floor
and a cylindrical skirt at the kink radius rising from it to
SKIRT_TOP_Z are one printed part in `drive/` (it carries the bearing
and the motor). The stator wall is a separate liner that sits on the
skirt's top edge by a flange at the same height — see SKIRT_TOP_Z.
"""

import math


# --- Rotor + stator V geometry -----------------------------------------

# Rotor outer diameter at its base (the widest section, closest to the
# V-bottom). Sized for the chained-singulator architecture: this m02
# is the **S1 (LOAD-feeding) stage**, the smallest of a tapered
# 3-stage chain (S1 = 240, S2 = 300, S3 = 360 mm rotor base). 240 mm
# gives an exit-arc length at the rim ≈ 75 mm — enough margin under
# the BRICK_SPHERE_DIA = 80 mm worst case that singulation events
# don't routinely wedge mid-pass. Bumped from 180 (which gave only
# 56 mm of exit-arc at the rim; bricks longer than ~55 mm spanned
# the full arc with no behind-it gap for the next-in-line to
# declump). See `_notes/decisions/m02-chain.md` for the chain
# rationale.
ROTOR_BASE_DIAMETER = 240.0

# Rotor top diameter — the flat top platform bricks drop onto. Not
# zero (would be a bare apex); a truncated top gives the hopper feed
# somewhere to land. 60 mm is comfortably smaller than
# #brick_sphere_dia so most bricks shed outward onto the flank rather
# than stacking on top. Held constant across the chain (S1/S2/S3) so
# the rotor flank gets steeper (and the rotor gets taller) as
# diameter grows; this is intentional — bigger upstream stages want
# more flank-roll distance for clump-breaking.
ROTOR_TOP_DIAMETER = 60.0

# V half-angle, measured from vertical. Shared by the rotor flank and
# the stator outer wall so the two surfaces form a symmetric V.
# Matches #channel_v_angle (45°).
V_HALF_ANGLE_DEG = 45.0

# Derived: rotor height from base/top diameters and the flank angle.
# (r_base − r_top) = h · tan(V_HALF_ANGLE_DEG) for a cone flank at
# V_HALF_ANGLE from vertical.
ROTOR_HEIGHT = (
    (ROTOR_BASE_DIAMETER - ROTOR_TOP_DIAMETER) / 2.0
    / math.tan(math.radians(V_HALF_ANGLE_DEG))
)

# Facet count around the rotor circumference. 10 panels meet at the
# top; the rim stays a true circle so no gap opens against the stator
# regardless of rotor angle.
ROTOR_FACET_COUNT = 10

# Rotor facet "crown kink" — a 20-vertex star cross-section placed
# partway up the flank. 10 "peak" star points line up with top-polygon
# vertices (the ridges visible in the render); 10 "valley" points sit
# at the intermediate azimuths, pulled radially inward to create the
# triangular sub-facets between ridges. The rim (z=0) stays a circle;
# the kink fades out as it approaches the base.
ROTOR_FACET_KINK_Z_FRAC = 0.35         # fraction of rotor height
ROTOR_FACET_PEAK_BULGE = 0.0           # outward deflection of peaks (mm)
ROTOR_FACET_VALLEY_DENT = 4.0          # inward deflection of valleys (mm)

# Radial gap between rotor base edge and stator V-bottom. Must stay
# smaller than the thinnest common LEGO plate (~3.2 mm) so no piece
# slips past the V-bottom. 2 mm is a tight starting point; candidate
# for #rotor_stator_gap once tuned.
ROTOR_STATOR_GAP = 2.0

# Stator container wall height above the V-bottom, standard variant
# (no extended-wall hopper). Matches #channel_wall_height.
STATOR_WALL_H = 40.0

# Height above the V-bottom where the stator inner wall transitions
# from sloped (V at V_HALF_ANGLE_DEG) to vertical. Sized to clear the
# tallest common LEGO element that rides in the V — 30 mm covers a
# 3× brick stack, 2×2 round cylinders, and minifig torso+head with
# headroom. Above this, the cone is unnecessary: the pieces are
# already seated in the V, and a cylindrical upper wall just
# contains splash. Must be ≤ STATOR_WALL_H.
STATOR_V_H = 30.0

# Exit arc angular width — the cut-away on the outer stator wall
# where bricks fall off. Starts at one facet-pitch so a single facet
# face spans the opening when aligned; candidate for tuning per the
# spec's Open Questions.
EXIT_ARC_DEG = 360.0 / ROTOR_FACET_COUNT   # 36° at N=10


# --- Rotor shell (hollow) ----------------------------------------------

# Rotor is an FDM shell, not a solid. The cavity is open at the base so
# the support under the cavity ceiling can come out after printing; it
# no longer hides anything (the drive left the cavity 2026-09-11).

# Radial wall thickness of the faceted shell, measured between matched-
# taper inner and outer polygons at their circumscribing-circle
# radii. Actual perpendicular-to-facet wall is this × cos(π/N) ≈ 2.31
# at N=8 — still 5 perimeters at 0.4 mm nozzle.
ROTOR_SHELL_WALL_T = 2.5

# Thickness of the solid top cap (the brick-drop platform). 10 mm is
# thick enough to be rigid across the cavity opening during printing.
ROTOR_TOP_CAP_T = 10.0


# --- Stator wall + skirt radii -----------------------------------------

# Radial wall thickness. Shared, not stator-private, because the base
# plate's OD is the skirt's OD (`m02-chain.md` §5: the foot ring sits
# at the kink radius, and the plate *is* the foot ring).
STATOR_WALL_T = 3.0

# Inner face at the V-bottom sits ROTOR_STATOR_GAP radially outside
# the rotor's base circumscribing-circle edge; the V-cone kinks to
# vertical at STATOR_V_H.
R_STATOR_INNER_BASE = ROTOR_BASE_DIAMETER / 2.0 + ROTOR_STATOR_GAP
R_STATOR_INNER_KINK = (
    R_STATOR_INNER_BASE + STATOR_V_H * math.tan(math.radians(V_HALF_ANGLE_DEG))
)
R_STATOR_OUTER_BASE = R_STATOR_INNER_BASE + STATOR_WALL_T
R_STATOR_OUTER_KINK = R_STATOR_INNER_KINK + STATOR_WALL_T     # 155


# --- Drive: rim gear on a rim bearing ----------------------------------
# Decided 2026-09-11 (`_notes/decisions/m02-rotor-drive.md`). The rotor
# grows a flange below its base rim carrying ring-gear teeth on its OD,
# rides on a bought turntable bearing, and is driven from below by an
# off-axis NEMA 17 + pinion hanging under the base plate. Replaces the
# hidden direct drive (motor inside the cavity on a fixed stem).

# Spur pair. Module 1.5 at 20° PA: the ring pitch diameter is the rotor
# base diameter itself (so the flange's pitch cylinder continues the
# rim straight down), which makes the tooth count an integer for every
# chain stage (240 → 160, 300 → 200, 360 → 240). Module 2 was the
# first instinct but a 16-tooth m2 pinion reaches r ≈ 154 and hits the
# Ø310 skirt; at m1.5 a 17-tooth pinion (above the ~17 T undercut floor
# at 20° PA) reaches r ≈ 147 with 5 mm to spare. The m05 base floor
# runs m1.5 / 12 T — same module, so one pinion could serve both if m05
# moves its rails out (17 T there pushes the tip 3.75 mm past the rail
# ring; see `state/m05-diverter.md`).
MODULE = 1.5
PRESSURE_ANGLE_DEG = 20.0             # documentation — cheap model has no teeth
PINION_TEETH = 17
RING_TEETH = round(ROTOR_BASE_DIAMETER / MODULE)   # 160
RING_PITCH_D = RING_TEETH * MODULE                 # 240.0
PINION_PITCH_D = PINION_TEETH * MODULE             # 25.5
GEAR_RATIO = RING_TEETH / PINION_TEETH             # ≈ 9.41 : 1
ADDENDUM = MODULE                                  # tip = pitch + module
MESH_BACKLASH = 0.2                   # extra centre distance for FDM mesh play

# Pinion axis radius from the rotor axis, and its azimuth. Azimuth 180°
# is the -X side: away from the exit arc (+X, the downstream stage) —
# the chain rule "point the motor away from downstream", which for S1
# is also the off-hub side.
CENTER_DIST = (RING_PITCH_D + PINION_PITCH_D) / 2.0 + MESH_BACKLASH   # 132.95
MOTOR_AZIMUTH_DEG = 180.0

# Rotor flange: annulus below the base rim. OD is the ring pitch
# cylinder; ID leaves a 7.5 mm land inboard of the bearing ID.
ROTOR_FLANGE_T = 10.0                 # = ring-gear face width
ROTOR_FLANGE_R_IN = 75.0

# Pinion: pitch cylinder + a hub below it that reaches down to just
# above the plate, so the set screw has shaft to bite on.
PINION_FACE_W = 8.0
PINION_TOP_CLEAR = 1.0                # pinion top below the stator bottom
PINION_HUB_OD = 14.0
PINION_HUB_PLATE_CLEAR = 0.5

# Bought aluminium turntable ("lazy susan") bearing, nominal 8-inch
# class. Stub dims — confirm against the part actually ordered; the
# seat is parametric on these three numbers.
BEARING_OD = 200.0
BEARING_ID = 165.0
BEARING_H = 8.5

# Bowl floor ("plate" in every derived name below): the §5 foot ring,
# extended inward to carry the bearing. OD = skirt OD (Ø310 — exceeds
# the X1C's 256 mm bed exactly as the stator wall already does; both
# print in thirds, seams open in `state/m02-singulator.md`).
PLATE_T = 6.0
PLATE_R_IN = ROTOR_FLANGE_R_IN
PLATE_R_OUT = R_STATOR_OUTER_KINK

# Skirt: the bowl's wall, STATOR_WALL_T thick, on the floor's rim,
# rising to SKIRT_TOP_Z. Its inner face is the kink radius, so the
# stator's V-cone (outer face r = R_STATOR_OUTER_BASE + z·tan V) meets
# that face exactly at SKIRT_TOP_Z — the stator carries a flat flange
# there, out to R_STATOR_OUTER_KINK, that rests on the skirt's top
# edge, and its cone seats on that edge's inner corner (self-centring).
# Why two parts and not one bowl: each prints without support in its
# own orientation (the bowl floor-down, the liner inverted as before),
# and the Ø310 thirds seams of the two rings can be staggered.
SKIRT_R_IN = PLATE_R_OUT - STATOR_WALL_T                    # 152 = R_STATOR_INNER_KINK
SKIRT_TOP_Z = (SKIRT_R_IN - R_STATOR_OUTER_BASE) / math.tan(math.radians(V_HALF_ANGLE_DEG))   # 27

# Exit floor (the tongue, a stator part) — shared because the skirt is
# slotted for it: the wedge cut through the skirt stops
# EXIT_FLOOR_SIDE_CLEAR under the tongue's underside, leaving a stub
# of skirt below it in the exit arc.
EXIT_FLOOR_T = 4.0
EXIT_FLOOR_SIDE_CLEAR = 1.5
SKIRT_CUT_BOTTOM_Z = -EXIT_FLOOR_T - EXIT_FLOOR_SIDE_CLEAR    # -5.5

# Thirds seams (`seams.py`; 2026-09-17). Both Ø310 rings print as
# three sectors. Phases are m02-local degrees from the exit arc's
# centre, and per-instance like MOTOR_AZIMUTH_DEG:
# - the liner's seams (75 / −165 / −45) need only miss the exit arc —
#   24° clear of the ±21° the wedge and paddle take — so one phase
#   serves every stage and the liner stays the same part everywhere;
# - the bowl's must also miss the motor pattern and every gantry bar
#   end under the floor, and stay staggered from the liner's, so the
#   chain passes one per stage (`cad/m02_chain/params.py` §Seams). The
#   default is S1's.
LINER_SEAM_PHASE_DEG = 75.0
BOWL_SEAM_PHASE_DEG = 41.0

# NEMA 17 compact short body (shortest commodity variant).
NEMA17_SIDE = 42.3
NEMA17_BODY_LEN = 40.0
NEMA17_SHAFT_D = 5.0
NEMA17_SHAFT_LEN = 22.0
NEMA17_BOLT_PATTERN = 31.0   # M3 × 4 square spacing, shaft-exit face
NEMA17_PILOT_D = 22.0        # centring boss on the shaft-exit face
NEMA17_PILOT_H = 2.0
NEMA17_BOLT_HOLE_D = 3.4     # M3 clearance through the floor
# The M3 heads are counterbored flush into the floor's top face: at
# CENTER_DIST the pattern's two outboard corners reach r ≈ 149.3, and
# an M3 socket head (Ø5.5) standing on the floor would touch the
# skirt's inner face at r 152. Recessed, the heads also clear the
# rotor flange above and need no room; the counterbore's rim nicks
# the skirt's foot by ~0.3 mm over its 3 mm depth at those two
# corners, which the bowl builder guards and accepts.
NEMA17_BOLT_CBORE_D = 6.0    # M3 SHCS head Ø5.5 + 0.5
NEMA17_BOLT_CBORE_DEPTH = 3.0   # = head height, flush
PLATE_MOTOR_HOLE_D = NEMA17_PILOT_D + 1.0   # pilot boss clears the floor

# Slip fit between the NEMA-17 shaft and the pinion bore. Snug so a
# single transverse M3 set screw through the hub locks the pinion.
MOTOR_SHAFT_BORE_CLEARANCE = 0.1

# Derived vertical stack (see the module docstring).
FLANGE_BOTTOM_Z = -ROTOR_FLANGE_T                          # -10
BEARING_TOP_Z = FLANGE_BOTTOM_Z                            # -10
BEARING_BOTTOM_Z = BEARING_TOP_Z - BEARING_H               # -18.5
PLATE_TOP_Z = BEARING_BOTTOM_Z                             # -18.5
PLATE_BOTTOM_Z = PLATE_TOP_Z - PLATE_T                     # -24.5
MOTOR_FACE_Z = PLATE_BOTTOM_Z                              # -24.5
MOTOR_BOTTOM_Z = MOTOR_FACE_Z - NEMA17_BODY_LEN            # -64.5
SHAFT_TIP_Z = MOTOR_FACE_Z + NEMA17_SHAFT_LEN              # -2.5
PINION_TOP_Z = -PINION_TOP_CLEAR                           # -1
PINION_BOTTOM_Z = PINION_TOP_Z - PINION_FACE_W             # -9
PINION_HUB_BOTTOM_Z = PLATE_TOP_Z + PINION_HUB_PLATE_CLEAR # -18
