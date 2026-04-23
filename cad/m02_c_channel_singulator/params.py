"""Shared interface constants for the M02 C-channel singulator.

Top-level geometry that multiple sub-assemblies (rotor, stator, drive,
exit_sensor) must agree on. Sub-assembly-internal details stay in that
sub-assembly's file.

Cross-module inputs:
- #channel_v_angle (docs/parameters.md) → V_HALF_ANGLE_DEG
- #channel_wall_height               → STATOR_WALL_H
- #print_clearance_loose             → SHAFT_BORE_CLEARANCE
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


# --- Drive shaft + bearings --------------------------------------------

# 8 mm steel shaft + 608 skateboard bearings (ID 8, OD 22, W 7).
# Standard in the 3D-printer ecosystem; cheap, widely available.
SHAFT_DIAMETER = 8.0
BEARING_ID = 8.0
BEARING_OD = 22.0
BEARING_W = 7.0

# Rotor shaft-bore clearance. Set-screw retention wants a snug slip
# fit, not a press fit — #print_clearance_loose applied symmetrically.
SHAFT_BORE_CLEARANCE = 0.25
