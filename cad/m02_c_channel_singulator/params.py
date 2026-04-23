"""Shared interface constants for the M02 C-channel singulator.

Top-level geometry that multiple sub-assemblies (rotor, stator, drive,
exit_sensor) must agree on. Sub-assembly-internal details stay in that
sub-assembly's file.

Cross-module inputs:
- #channel_v_angle (docs/parameters.md) → V_HALF_ANGLE_DEG
- #channel_wall_height                  → STATOR_WALL_H
- #print_clearance                      → MOTOR_SHAFT_BORE_CLEARANCE
"""

import math


# --- Rotor + stator V geometry -----------------------------------------

# Rotor outer diameter at its base (the widest section, closest to the
# V-bottom). Middle of the reference project's 150–200 mm range
# (see docs/m02-c-channel-singulator.md Open Questions).
ROTOR_BASE_DIAMETER = 180.0

# Rotor top diameter — the flat top platform bricks drop onto. Not
# zero (would be a bare apex); a truncated top gives the hopper feed
# somewhere to land. 60 mm is comfortably larger than #brick_sphere_dia
# would need, but small enough that most bricks shed outward onto the
# flank rather than stacking on top.
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

# Facet count around the rotor circumference. Spec's default starting
# point; 6 gives wider panels (more grip), 12 gives smoother conveyance
# (less pulse per revolution).
ROTOR_FACET_COUNT = 8

# Radial gap between rotor base edge and stator V-bottom. Must stay
# smaller than the thinnest common LEGO plate (~3.2 mm) so no piece
# slips past the V-bottom. 2 mm is a tight starting point; candidate
# for #rotor_stator_gap once tuned.
ROTOR_STATOR_GAP = 2.0

# Stator container wall height above the V-bottom, standard variant
# (no extended-wall hopper). Matches #channel_wall_height.
STATOR_WALL_H = 40.0

# Exit arc angular width — the cut-away on the outer stator wall
# where bricks fall off. Starts at one facet-pitch so a single facet
# face spans the opening when aligned; candidate for tuning per the
# spec's Open Questions.
EXIT_ARC_DEG = 360.0 / ROTOR_FACET_COUNT   # 45° at N=8


# --- Rotor shell (hollow) ----------------------------------------------

# Rotor is an FDM shell, not a solid. Cavity hides the drive: motor
# body and stem sit inside the spinning rotor's interior.

# Radial wall thickness of the faceted shell, measured between matched-
# taper inner and outer polygons at their circumscribing-circle
# radii. Actual perpendicular-to-facet wall is this × cos(π/N) ≈ 2.31
# at N=8 — still 5 perimeters at 0.4 mm nozzle.
ROTOR_SHELL_WALL_T = 2.5

# Thickness of the solid top cap (the brick-drop platform). 10 mm is
# thick enough to be rigid across the cavity opening during printing
# and gives a 5 mm-deep blind bore above the motor-shaft tip.
ROTOR_TOP_CAP_T = 10.0

# Internal coupling boss — a cylinder descending from the underside
# of the top cap into the cavity. Shaft bore runs up through the
# boss and into the top cap (blind). OD gives ~7.5 mm of wall meat
# around the 5 mm NEMA-17 shaft — room for a transverse M3 set screw
# without cracking.
ROTOR_BOSS_OD = 20.0


# --- Drive: NEMA 17 stepper + fixed stem -------------------------------

# NEMA 17 compact short body (shortest commodity variant). Any
# NEMA 17 up to ~48 mm body length still fits the cavity.
NEMA17_SIDE = 42.3
NEMA17_BODY_LEN = 40.0
NEMA17_SHAFT_D = 5.0
NEMA17_SHAFT_LEN = 22.0
NEMA17_BOLT_PATTERN = 31.0   # M3 × 4 square spacing, shaft-exit face

# Slip fit between NEMA-17 shaft and rotor's internal bore. Snug so a
# single set screw (M3 transverse through the boss, captured against
# the shaft's flat) locks the rotor to the shaft.
MOTOR_SHAFT_BORE_CLEARANCE = 0.1

# Stem — a fixed vertical rod rising from the frame, through the
# rotor's open bottom, to a mount plate inside the cavity. Motor
# hangs from the plate (front face up, body below, shaft further
# up). The rod is OFFSET from the rotor axis — it can't be coaxial
# because the motor body occupies the axial column between
# MOTOR_BASE_Z and MOUNT_Z. A short cantilever arm joins the rod
# top to the centered mount plate above the motor.
#
# STEM_OFFSET_X picked so the rod sits clear of the NEMA-17 corner
# (r ≈ 30) and well inside the rotor cavity apothem at z=MOUNT_Z
# (apothem ≈ 48). Rod OD scaled down to 10 mm (steel shaft; stiff
# enough for the 200 g cantilever motor + <3 kg rotor + bricks).
STEM_OD = 10.0
STEM_OFFSET_X = 38.0             # radial offset of rod from rotor axis
STEM_BOTTOM_Z = -50.0            # deep in the frame; stub value for V1
STEM_FLANGE_DIA = 50.0           # mount-plate disc OD (centered on axis)
STEM_FLANGE_T = 5.0
STEM_ARM_W = 15.0                # cantilever arm width (y-extent)

# Derived vertical layout — anchored so the motor-shaft tip sits
# 5 mm below the rotor's outer top face (blind bore depth).
SHAFT_TIP_Z = ROTOR_HEIGHT - ROTOR_TOP_CAP_T / 2.0       # 55 (mid top cap)
MOUNT_Z = SHAFT_TIP_Z - NEMA17_SHAFT_LEN                 # 33
MOTOR_BASE_Z = MOUNT_Z - NEMA17_BODY_LEN                 # -7

# Boss + blind-bore span. Boss starts AT the motor front face
# (MOUNT_Z): any lower and the boss body would collide with the
# motor body; any higher and the shaft bottom would hang exposed
# below the boss. The bore spans the full boss height plus another
# ROTOR_TOP_CAP_T/2 into the top-cap solid, giving ~3 mm of axial
# headroom between the shaft tip and the blind end.
BOSS_BOTTOM_Z = MOUNT_Z                                  # 33
BOSS_TOP_Z = ROTOR_HEIGHT - ROTOR_TOP_CAP_T              # 50
SHAFT_BORE_BOTTOM_Z = BOSS_BOTTOM_Z                      # 33 (open)
SHAFT_BORE_TOP_Z = ROTOR_HEIGHT - ROTOR_TOP_CAP_T / 2.0  # 55 — blind
