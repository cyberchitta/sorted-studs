"""Shared interface constants for the m02 singulator chain.

The chain is a relocatable unit that hangs the m02 singulator(s) off
column extensions rising from the m05 diverter's top deck and
delivers bricks onto the m03 turret's LOAD platform. This file owns
the chain-local frame and the cross-boundary anchors mirrored from
m05 / m03 / m02 with explicit assertions catching drift.

Coordinate frame (chain-local):
    origin = LOAD platform top-face centroid (= world (+125, +125, 1030)
             after the top-level pose applies)
    +X     = away from the diverter hub, into the singulator chain
             (= world +X)
    +Y     = parallel to the turret cross-rail (= world +Y)
    +Z     = up
    z = 0  = LOAD platform top face

The top-level `cad/assembly.py` places the chain with one transform
`Pos(+125, +125, 1030)`, so every chain-local position resolves to
world by adding that offset.

Anchors at-a-glance:

    m05 deck top face       chain-local z = -140  (world 890)
    m05 outer_vert_0 top    chain-local (+145, +31, -140)
    col1 extension          chain-local (+145, +31), z ∈ [-140, +38]
    col2 outboard stub      chain-local (+325, +50), z ∈ [-140, +38]
    S1 yoke crossbar top    chain-local z = +38
    S1 m02 V-bottom         chain-local (+235, 0, +38)  (world (+360, +125, 1068))
    S1 chute exit (handoff) chain-local (+80, 0, +48)   (world (+205, +125, 1078))
    LOAD ramp inlet         chain-local (+80, 0, +48)
    LOAD ramp outlet        chain-local (0, 0, +2)      (2 mm above LOAD top)

col2's Y is shifted +50 mm off the handoff sketch (which placed it
at chain-local (+325, 0)) so the yoke axis from col1 (+145, +31) to
col2 clears the m02 NEMA-17 motor body in +Y. At col2 = (+325, 0)
the yoke axis ran through the motor's +Y face at the 7-mm
yoke/motor z-overlap (yoke z [+18, +38] vs motor z [+31, +71]).
With col2 = (+325, +50), yoke-to-motor perpendicular clearance is
~8 mm.

This slice DEFERS the m02 stator-mounting interface (handoff
sub-task 2): the m02 stator sits at its geometric S1 position but
NO plate / clamp / cradle bridges yoke top to stator V-bottom rim
yet. The stator's outer wall at z = 0 has only a 3-mm-wide rim
ring; gripping it (Option B: chain-side clamps) needs a curved
cradle, and Option A (m02-internal `frame/` flange) needs an
m02 sub-assembly edit. Both interact with the rotor/motor/stem
clearances inside the stator's inner footprint and earn a
separate slice. The chain's columns + yoke + LOAD ramp are
independent of the mounting choice.
"""

from __future__ import annotations

import math


# --- Mirror from cad/m05_diverter/params.py ----------------------------

# Outer-hex circumradius (center → vertex) for the m05 frame. The
# closest m05 through-column to the S1 m02 axis is `outer_vert_0` at
# azimuth 30° (= world (+R_VERT_OUTER · cos 30°, +R_VERT_OUTER · sin
# 30°) ≈ (+270, +156)) — the chain's primary support column extension
# rises from this vertex.
M05_R_VERT_OUTER = 311.769145362398

# m05's `_APOTHEM_TO_VERTEX = 2 / sqrt(3)` × R_HEX_OUTER (= 270). The
# literal is reproduced here so the chain stays standalone; the
# top-level `cad/assembly.py` cross-checks against the live m05 value.
assert abs(M05_R_VERT_OUTER - (270.0 * 2.0 / math.sqrt(3.0))) < 1e-6, (
    "M05_R_VERT_OUTER mismatched — update from "
    "cad/m05_diverter/params.py: R_VERT_OUTER"
)


# --- Mirror from cad/m05_diverter/assembly.py --------------------------

# Top of m05 through-columns (= `TOP_VERT_Z = 4 * FLOOR_H`). Top deck
# bottom face sits flush with the column tops; deck top face is
# TOP_DECK_T higher.
M05_TOP_VERT_Z = 880.0
M05_TOP_DECK_T = 10.0
M05_DECK_TOP_Z = M05_TOP_VERT_Z + M05_TOP_DECK_T   # 890

assert M05_TOP_VERT_Z == 4 * 220.0, (
    "M05_TOP_VERT_Z mismatched — m05 FLOOR_H × 4 expected; update from "
    "cad/m05_diverter/assembly.py: TOP_VERT_Z"
)


# --- Mirror from cad/m03_scanner/params.py + cad/assembly.py -----------

# Turret arm radius (hub axis → station centroid). The LOAD station
# sits at turret-local (+TURRET_ARM_R, 0); the chain origin = LOAD
# platform centroid in world.
M03_TURRET_ARM_R = 125.0

# World z of the LOAD platform top face (= top-level `_TURRET_WORLD_Z`,
# derived in `cad/assembly.py` from m05 deck top + turret floor offset).
M03_LOAD_TOP_Z = 1030.0


# --- Chain world pose --------------------------------------------------

# World location of chain-local origin. `cad/assembly.py` splices the
# chain at `Pos(*CHAIN_WORLD_POSE_XYZ)` (no rotation — chain-local axes
# match world axes).
CHAIN_WORLD_POSE_XYZ = (
    M03_TURRET_ARM_R,                      # +125
    M03_TURRET_ARM_R,                      # +125
    M03_LOAD_TOP_Z,                        # 1030
)


# --- m05 anchors expressed in chain-local coordinates ------------------

# m05 deck top face plane (world z = 890) → chain-local z.
DECK_TOP_Z_LOCAL = M05_DECK_TOP_Z - M03_LOAD_TOP_Z                    # -140

# m05 `outer_vert_0` axis is at world (+R_VERT_OUTER·cos 30°,
# +R_VERT_OUTER·sin 30°) ≈ (+270.0, +155.88). In chain-local:
_OUTER_VERT_0_WORLD_X = M05_R_VERT_OUTER * math.cos(math.radians(30.0))
_OUTER_VERT_0_WORLD_Y = M05_R_VERT_OUTER * math.sin(math.radians(30.0))
OUTER_VERT_0_LOCAL_XY = (
    _OUTER_VERT_0_WORLD_X - M03_TURRET_ARM_R,   # ≈ +145.0
    _OUTER_VERT_0_WORLD_Y - M03_TURRET_ARM_R,   # ≈ +30.88
)


# --- S1 stage placement ------------------------------------------------

# S1 m02 axis world location (see `_notes/decisions/m02-chain.md`).
# The +360 in X comes from the chute-exit X (+205) + stator outer
# radius at the V-cone top (R_STATOR_OUTER_KINK ≈ 155); +125 in Y
# matches the LOAD centroid.
S1_AXIS_WORLD_X = 360.0
S1_AXIS_WORLD_Y = 125.0
S1_AXIS_LOCAL_XY = (
    S1_AXIS_WORLD_X - M03_TURRET_ARM_R,    # +235
    S1_AXIS_WORLD_Y - M03_TURRET_ARM_R,    # 0
)

# Yoke top face = S1 m02 V-bottom plane. The stator V-bottom sits at
# m02-local z = 0; chain places m02 at chain-local z = YOKE_TOP_Z_LOCAL
# so the V-bottom plane and yoke top face plane coincide. The stator's
# mounting bridge (deferred this slice) would sit on this same plane.
YOKE_TOP_Z_LOCAL = 38.0
S1_VBOT_Z_LOCAL = YOKE_TOP_Z_LOCAL                            # +38
S1_VBOT_WORLD_Z = M03_LOAD_TOP_Z + S1_VBOT_Z_LOCAL            # 1068

# S1 m02 splice pose (chain-local). The m02 internal frame fires its
# chute in m02-local +X; we rotate by 180° about Z so the chute fires
# in chain-local -X, toward the LOAD platform.
S1_M02_HEADING_DEG = 180.0


# --- Column extensions (chain primary structure) -----------------------

# 2020 aluminum extrusion — matches m05's EXT_SIZE.
EXT_SIZE = 20.0

# col1: extension above m05 outer_vert_0. Spans the deck-top plane
# (z = DECK_TOP_Z_LOCAL = -140, butting against outer_vert_0's top
# face and resting on the deck) up to the yoke top face plane
# (z = YOKE_TOP_Z_LOCAL = +38). Length ≈ 178 mm. Rotated by 30° about
# Z to match m05's outer_vert_0 azimuth orientation (one face aligned
# with the bracket plate convention).
COL1_LOCAL_XY = OUTER_VERT_0_LOCAL_XY                        # ≈ (+145, +31)
COL1_ROT_DEG = 30.0
COL_Z_BOT_LOCAL = DECK_TOP_Z_LOCAL                           # -140
COL_Z_TOP_LOCAL = YOKE_TOP_Z_LOCAL                           # +38
COL_LEN = COL_Z_TOP_LOCAL - COL_Z_BOT_LOCAL                  # 178

# col2: new outboard 2020 stub. Picked so the yoke from col1 to col2
# clears the m02 motor body in +Y (see module docstring). Sits in
# +Y +X direction from S1. Outside the m05 top-deck OD/2 = 330; needs
# a small outboard outrigger off col1 at z = DECK_TOP_Z_LOCAL to anchor
# its base — the outrigger is DEFERRED this slice (modeled as col2
# starting at the deck plane with no separate base part).
COL2_LOCAL_XY = (
    S1_AXIS_LOCAL_XY[0] + 90.0,                              # +325
    S1_AXIS_LOCAL_XY[1] + 50.0,                              # +50
)

assert (COL2_LOCAL_XY[0] + M03_TURRET_ARM_R) > 330.0, (
    f"col2 world X ({COL2_LOCAL_XY[0] + M03_TURRET_ARM_R}) should sit "
    "outboard of the m05 top-deck OD/2 = 330. Move col2 outward or "
    "rework the deck mounting strategy."
)


# --- Yoke crossbar -----------------------------------------------------

# 2020 horizontal extrusion spanning col1 ↔ col2, with top face at
# YOKE_TOP_Z_LOCAL. The axis runs from col1's center to col2's center;
# length reserves enough at each end to butt cleanly against both
# columns' rotated cross-sections. col1 is rotated 30° (relative to
# the yoke axis the relative angle is ≈ 24°), so the col1 corner
# extends 10·(cos 24° + sin 24°) ≈ 13.2 mm along the yoke axis from
# col1 center. col2 rotated by the yoke angle has corner extent
# 10 mm. Reserve 20 mm per end (= EXT_SIZE) for both columns' worst-
# case rotated corners — single value, slightly conservative on col2.
_YOKE_DX = COL2_LOCAL_XY[0] - COL1_LOCAL_XY[0]               # +180
_YOKE_DY = COL2_LOCAL_XY[1] - COL1_LOCAL_XY[1]               # ≈ +19.12
YOKE_AXIAL_DISTANCE = math.sqrt(_YOKE_DX**2 + _YOKE_DY**2)   # ≈ 181.0
YOKE_LEN = YOKE_AXIAL_DISTANCE - 2.0 * EXT_SIZE              # ≈ 141.0
YOKE_ANGLE_DEG = math.degrees(math.atan2(_YOKE_DY, _YOKE_DX))  # ≈ +6.06
YOKE_MID_XY = (
    (COL1_LOCAL_XY[0] + COL2_LOCAL_XY[0]) / 2.0,             # +235
    (COL1_LOCAL_XY[1] + COL2_LOCAL_XY[1]) / 2.0,             # ≈ +40.44
)


# --- LOAD ramp ---------------------------------------------------------

# 30° was the conservative LEGO-on-PLA friction-angle target. The
# achieved pitch is atan(41/80) ≈ 27.1° — under the target but still
# well above the empirical LEGO-on-PLA static friction angle (~20–25°).
# Outlet z is offset RAMP_FLOOR_T + 2 mm above the LOAD platform top
# face so the ramp's BOTTOM face clears the platform acrylic by 2 mm
# (the acrylic insert occupies platform-local z ∈ [-4, 0], with the
# top face at the platform's reference plane). The brick contact
# surface at the outlet sits 7 mm above the acrylic top — a small
# drop onto the LOAD station.
CHUTE_EXIT_LOCAL = (80.0, 0.0, 48.0)        # (+205, +125, 1078) in world
RAMP_INLET_LOCAL = CHUTE_EXIT_LOCAL
RAMP_FLOOR_T = 5.0
RAMP_OUTLET_LOCAL = (0.0, 0.0, RAMP_FLOOR_T + 2.0)  # +7  (world z = 1037)

RAMP_DX = RAMP_INLET_LOCAL[0] - RAMP_OUTLET_LOCAL[0]         # 80
RAMP_DZ = RAMP_INLET_LOCAL[2] - RAMP_OUTLET_LOCAL[2]         # 41
RAMP_PITCH_DEG = math.degrees(math.atan2(RAMP_DZ, RAMP_DX))  # ≈ 27.1
RAMP_SLOPE_LEN = math.sqrt(RAMP_DX**2 + RAMP_DZ**2)          # ≈ 89.9

# 100 mm wide — gives the brick a generous lateral catch envelope
# (BRICK_SPHERE_DIA = 80 mm; the rotor-tangential drift from the
# downstream paddle adds ~38 mm worst-case before friction). Side
# walls catch the drift so the brick stabilises onto the ramp
# centerline.
RAMP_WIDTH = 100.0
RAMP_WALL_H = 20.0
RAMP_WALL_T = 4.0
