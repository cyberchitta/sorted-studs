"""Shared interface constants for the m02 singulator chain.

The chain is a relocatable unit: three m02 singulator stages — S1 at
the LOAD handoff, S2 and S3 upstream of it, each dropping its bricks
onto the stage below — the gantry that carries them on two of the m05
diverter's outer through-columns, and the LOAD ramp that delivers S1's
output onto the m03 turret's LOAD platform. This file owns the
chain-local frame, the placement family, the derived stage table, the
gantry sizes, and the cross-boundary constants mirrored from m05 / m03
(the chain must build standalone, so it cannot import their live
values). m02 is different: the chain composes it, so m02's own
`params.py` is imported, not mirrored. `assembly.py` exports the
mirrors' implied datums as belief anchors (`load_top`, `deck_top`,
`post_top_4`, `post_top_5`); the top-level `cad/assembly.py` asserts
them coincident with the live units' own anchors, so a drifted mirror
fails loudly there.

Coordinate frame (chain-local):
    origin = LOAD platform top-face centroid (= world (+125, +125, 1030)
             after the top-level pose applies)
    +X     = world +X
    +Y     = world +Y
    +Z     = up
    z = 0  = LOAD platform top face

The top-level `cad/assembly.py` places the chain with one translation
`Pos(+125, +125, 1030)` (no rotation: the m03 turret rotation ψ is 0),
so every chain-local position resolves to world by adding that offset.

Placement — family P (owner 2026-09-15; `_notes/decisions/m02-chain.md`
§4, found in `_notes/explorers/m02-family-choice.html`):

    LOAD ramp   runs RAMP_RUN from LOAD at azimuth RAMP_AZIMUTH_DEG (φ)
                to S1's chute exit
    S1 axis     one stator kink radius (R_STATOR_OUTER_KINK = 155) from
                the chute exit, at PIVOT_THETA_DEG (θ) clockwise from
                the ramp line; S1 heads (fires its chute) back at LOAD
    S2, S3      one CHAIN_CHORD upstream of the stage below, the chain
                bending CHAIN_BEND_DEG per hop toward the m05 axis; each
                heads at the stage below, so headings derive
    z           S1's V-bottom at S1_VBOT_WORLD_Z (the turret-envelope
                raise, decided); each higher stage STAGE_PITCH above the
                one below, so its plate clears the lower rotor's apex by
                STAGE_CLEAR

Anchors at-a-glance (chain-local; world = + (125, 125, 1030)):

    m05 deck top face         z = -140                 (world 890)
    LOAD ramp outlet          (0, 0, +7)               (world z 1037)
    S1 chute exit = inlet     (130, 0, +75)            (world (255, 125, 1105))
    S1 axis                   (103, -153), V-bottom +65    (world (228, -28, 1095))
    S2 axis                   (0, -275),   V-bottom +209.5 (world (125, -150, 1239.5))
    S3 axis                   (-150, -330), V-bottom +354  (world (-25, -205, 1384))
    V_5, S1's post            (145, -281); top +20.5   (world 1050.5 = 160.5 over the deck)
    V_4, S3's post + S2's arm (-125, -437); top +309.5 (world 1339.5 = 449.5 over the deck)

Gantry (owner 2026-09-14/15, `m02-chain.md` §6; sizes from the
structure pass in `_notes/handoff/m02-gantry-spine.md`, hand statics):
S1 is propped on V_5 and S3 on V_4 — the m05 column continues through
the deck (`m05_diverter/params.py:THROUGH_DECK_POSTS`) and a 2020 prop
bar lies across its end, under the stage's base plate, from the post
side over the axis to a far end that adds nothing to the raise (S1)
or the ring's far edge (S3, which also takes a cross for the hopper
load). S2 hangs on one T arm: a 2020 stem square off V_4's +Y face to
a 2020 T bar under S2's plate. V_4 is braced at BRACE_H_ABOVE_DECK by
two 2020 diagonals down to the deck at V_3 and V_5. Post tops are the
bars' undersides — one bar depth below each propped plate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..m02_c_channel_singulator.params import (
    EXIT_ARC_DEG,
    MOTOR_AZIMUTH_DEG,
    PLATE_BOTTOM_Z,
    PLATE_R_IN,
    R_STATOR_OUTER_KINK,
    ROTOR_BASE_DIAMETER,
    ROTOR_HEIGHT,
)


# --- Mirror from cad/m05_diverter/params.py ----------------------------

# Outer-hex circumradius (center → vertex) for the m05 frame; vertex
# k sits at azimuth (k + 0.5)·60° about the m05 axis. The chain's
# posts are the outer columns at V_4 (270°) and V_5 (330°), continued
# through the deck.
M05_R_VERT_OUTER = 311.769145362398

# m05's `_APOTHEM_TO_VERTEX = 2 / sqrt(3)` × R_HEX_OUTER (= 270). The
# literal is reproduced here so the chain stays standalone. Drift is
# caught by the `post_top_4` / `post_top_5` belief anchors, which are
# derived from this value and checked against m05's live
# `outer_vert_{k}_top` anchors in the top-level `cad/assembly.py`. A
# local `assert` against a recomputed `270 * 2/sqrt(3)` would only
# catch a transcription typo here — both sides being local literals,
# it cannot see m05 move.


# --- Mirror from cad/m05_diverter/assembly.py --------------------------

# Top of m05's ten short through-columns (= `TOP_VERT_Z = 4 * FLOOR_H`).
# Top deck bottom face sits flush with those column tops; deck top face
# is TOP_DECK_T higher. (The two post columns continue above it.)
M05_TOP_VERT_Z = 880.0
M05_TOP_DECK_T = 10.0
M05_DECK_TOP_Z = M05_TOP_VERT_Z + M05_TOP_DECK_T   # 890
#
# As with M05_R_VERT_OUTER: drift is caught by the `deck_top` belief
# anchor derived from this value, against m05's live `deck_top` at the
# top level — not by a local assert over local literals.


# --- Mirror from cad/m03_scanner/params.py + cad/assembly.py -----------

# Turret arm radius (hub axis → station centroid). The LOAD station
# sits at turret-local (+TURRET_ARM_R, 0); the chain origin = LOAD
# platform centroid in world. The turret hub is at world (0, +125),
# the m05 axis at world (0, 0).
M03_TURRET_ARM_R = 125.0

# World z of the LOAD platform top face (derived in `cad/assembly.py`
# from m05 deck top + turret floor offset).
M03_LOAD_TOP_Z = 1030.0


# --- Chain world pose --------------------------------------------------

# World location of chain-local origin. `cad/assembly.py` derives the
# chain's pose from m03's live `load_top`; this is what the mirrors
# say it is (ψ = 0 — a turret rotation would move it, and the belief
# anchors would then stop coinciding).
CHAIN_WORLD_POSE_XYZ = (
    M03_TURRET_ARM_R,                      # +125
    M03_TURRET_ARM_R,                      # +125
    M03_LOAD_TOP_Z,                        # 1030
)


# --- m05 datums expressed in chain-local coordinates -------------------

# m05 deck top face plane (world z = 890) → chain-local z.
DECK_TOP_Z_LOCAL = M05_DECK_TOP_Z - M03_LOAD_TOP_Z                    # -140

# m05 axis (world (0, 0)) in chain-local XY.
M05_AXIS_LOCAL_XY = (-M03_TURRET_ARM_R, -M03_TURRET_ARM_R)           # (-125, -125)


def m05_outer_vertex_local_xy(k: int) -> tuple[float, float]:
    """Chain-local XY of m05 outer vertex k (azimuth (k + 0.5)·60°
    about the m05 axis at circumradius M05_R_VERT_OUTER)."""
    az = math.radians((k + 0.5) * 60.0)
    return (
        M05_AXIS_LOCAL_XY[0] + M05_R_VERT_OUTER * math.cos(az),
        M05_AXIS_LOCAL_XY[1] + M05_R_VERT_OUTER * math.sin(az),
    )


# --- Placement family P (owner 2026-09-15) -----------------------------

PIVOT_THETA_DEG = 100.0     # θ: S1 axis clockwise from the ramp line about the chute exit
RAMP_AZIMUTH_DEG = 0.0      # φ: the LOAD ramp comes in from this azimuth about LOAD
CHAIN_BEND_DEG = 30.0       # per hop, toward the m05 axis
CHAIN_CHORD = 160.0         # axis-to-axis, both hops
RAMP_RUN = 130.0            # LOAD centroid → S1 chute exit, in plan
N_STAGES = 3


# --- Vertical stack ----------------------------------------------------

# S1's V-bottom. Decided (owner 2026-09-15): the turret-envelope raise
# at family P with the short prop bar. Derivation, explorer arithmetic:
# S1's base plate stands over the LOAD platform's own brick, so its
# underside must clear the turret hardware top (world 1035) plus the
# settled-brick envelope (`m03_scanner/params.py:BRICK_SETTLED_H`, 30)
# by 5 mm: 1035 + 30 + 5 - PLATE_BOTTOM_Z (-24.5) = 1094.5 → 1095.
# `cad/check_turret_envelope.py` measures that clearance against the
# real geometry; this number is the chosen input, not the check.
S1_VBOT_WORLD_Z = 1095.0
S1_VBOT_Z_LOCAL = S1_VBOT_WORLD_Z - M03_LOAD_TOP_Z                   # +65

# The chute exit sits this far above the V-bottom (the stator's exit
# arc opening; unchanged from the 2026-05 chain).
CHUTE_EXIT_ABOVE_VBOT = 10.0

# Stage pitch: the upper stage's plate underside clears the lower
# rotor's apex (ROTOR_HEIGHT above the lower V-bottom) by STAGE_CLEAR.
# Holds while the upper plate disc reaches over the lower axis, which
# a CHAIN_CHORD of one kink radius plus a few mm guarantees — the
# chute then lands its brick CHUTE_LAND_R from the lower axis.
STAGE_CLEAR = 30.0
STAGE_PITCH = ROTOR_HEIGHT - PLATE_BOTTOM_Z + STAGE_CLEAR            # 144.5
CHUTE_LAND_R = CHAIN_CHORD - R_STATOR_OUTER_KINK                     # 5


# --- Stage table -------------------------------------------------------

def _rot2(v: tuple[float, float], deg: float) -> tuple[float, float]:
    a = math.radians(deg)
    return (
        v[0] * math.cos(a) - v[1] * math.sin(a),
        v[0] * math.sin(a) + v[1] * math.cos(a),
    )


@dataclass(frozen=True)
class Stage:
    """One singulator stage in chain-local coordinates."""

    name: str
    axis_xy: tuple[float, float]
    heading_deg: float          # azimuth the chute fires (m02-local +X)
    vbot_z: float               # stator V-bottom plane (m02-local z = 0)
    motor_azimuth_deg: float    # m02-local; passed to build_m02

    @property
    def chute_exit_xy(self) -> tuple[float, float]:
        h = math.radians(self.heading_deg)
        return (
            self.axis_xy[0] + R_STATOR_OUTER_KINK * math.cos(h),
            self.axis_xy[1] + R_STATOR_OUTER_KINK * math.sin(h),
        )

    @property
    def chute_exit_z(self) -> float:
        return self.vbot_z + CHUTE_EXIT_ABOVE_VBOT

    @property
    def plate_bottom_z(self) -> float:
        return self.vbot_z + PLATE_BOTTOM_Z


# Per-stage motor swing, m02-local degrees added to MOTOR_AZIMUTH_DEG
# (180 = away from the exit). S2's stem leaves V_4's +Y face and would
# run into S2's motor at 180; S3's cross bar, perpendicular to its prop
# bar, would pass under S3's motor at 180. Both swing +30 (explorer
# `propMotor` was 0 at P because it assumed a 5° post rotation the m05
# column cannot have; `assembly.py` asserts the clearances).
MOTOR_SWING_DEG = {"s1": 0.0, "s2": 30.0, "s3": 30.0}


def _stage_table() -> tuple[Stage, ...]:
    az = RAMP_AZIMUTH_DEG
    chute = (
        RAMP_RUN * math.cos(math.radians(az)),
        RAMP_RUN * math.sin(math.radians(az)),
    )
    t = math.radians(PIVOT_THETA_DEG)
    u = _rot2((math.cos(t), -math.sin(t)), az)      # chute exit → S1 axis
    s1 = (chute[0] + R_STATOR_OUTER_KINK * u[0], chute[1] + R_STATOR_OUTER_KINK * u[1])
    # Bend toward the m05 axis: the sign of the perpendicular that
    # points from S1 at the m05 axis.
    perp = (-u[1], u[0])
    to_hub = (M05_AXIS_LOCAL_XY[0] - s1[0], M05_AXIS_LOCAL_XY[1] - s1[1])
    hub_sign = 1.0 if perp[0] * to_hub[0] + perp[1] * to_hub[1] >= 0 else -1.0
    axes = [s1]
    for i in range(1, N_STAGES):
        d = _rot2(u, i * hub_sign * CHAIN_BEND_DEG)
        prev = axes[-1]
        axes.append((prev[0] + CHAIN_CHORD * d[0], prev[1] + CHAIN_CHORD * d[1]))
    stages = []
    for i, axis in enumerate(axes):
        name = f"s{i + 1}"
        if i == 0:
            hd = math.degrees(math.atan2(-u[1], -u[0]))
        else:
            dn = axes[i - 1]
            hd = math.degrees(math.atan2(dn[1] - axis[1], dn[0] - axis[0]))
        stages.append(Stage(
            name=name,
            axis_xy=axis,
            heading_deg=hd,
            vbot_z=S1_VBOT_Z_LOCAL + i * STAGE_PITCH,
            motor_azimuth_deg=MOTOR_AZIMUTH_DEG + MOTOR_SWING_DEG[name],
        ))
    return tuple(stages)


STAGES: tuple[Stage, ...] = _stage_table()
S1, S2, S3 = STAGES


# --- LOAD ramp ---------------------------------------------------------

# Outlet z is offset RAMP_FLOOR_T + 2 mm above the LOAD platform top
# face so the ramp's BOTTOM face clears the platform acrylic by 2 mm
# (the acrylic insert occupies platform-local z ∈ [-4, 0], with the
# top face at the platform's reference plane). The brick contact
# surface at the outlet sits 7 mm above the acrylic top — a small
# drop onto the LOAD station.
RAMP_FLOOR_T = 5.0
RAMP_OUTLET_LOCAL = (0.0, 0.0, RAMP_FLOOR_T + 2.0)  # +7  (world z = 1037)
RAMP_INLET_LOCAL = (*S1.chute_exit_xy, S1.chute_exit_z)               # (130, 0, 75)

RAMP_DZ = RAMP_INLET_LOCAL[2] - RAMP_OUTLET_LOCAL[2]                  # 68
RAMP_PITCH_DEG = math.degrees(math.atan2(RAMP_DZ, RAMP_RUN))          # ≈ 27.6
RAMP_SLOPE_LEN = math.sqrt(RAMP_RUN**2 + RAMP_DZ**2)                  # ≈ 146.7

# Pitch window — placeholders, not measured friction (LEGO on PLA
# static friction angle is ~20–25°; 30° was the earlier target).
RAMP_PITCH_MIN_DEG = 20.0
RAMP_PITCH_MAX_DEG = 40.0

# At family P the chute faces sideways to the ramp (S1 heads at 80°,
# the ramp runs at 180°): the brick leaves the rotor wall *tangentially*
# down the ramp axis (10° off it), through an exit arc that lies along
# the ramp rather than across it. So the ramp is a tangential catch:
# its inner side is S1's stator wall — the ramp is cut back to
# RAMP_STATOR_CLEAR outside the kink radius about S1's axis — and a
# flat landing at the exit height continues past the exit centre to
# the far end of the exit arc, so the whole opening has floor under it.
RAMP_STATOR_CLEAR = 1.0
RAMP_LANDING_LEN = R_STATOR_OUTER_KINK * math.sin(math.radians(EXIT_ARC_DEG / 2.0))   # ≈ 47.9

# 100 mm wide — gives the brick a generous lateral catch envelope
# (BRICK_SPHERE_DIA = 80 mm; the rotor-tangential drift from the
# downstream paddle adds ~38 mm worst-case before friction). Side
# walls catch the drift so the brick stabilises onto the ramp
# centerline.
RAMP_WIDTH = 100.0
RAMP_WALL_H = 20.0
RAMP_WALL_T = 4.0


# --- Gantry ------------------------------------------------------------

# 2020 aluminum extrusion — matches m05's EXT_SIZE.
EXT_SIZE = 20.0
BAR_DEPTH = EXT_SIZE            # bars lie flat under the plates

# Which m05 outer column props which stage (owner 2026-09-14).
PROP_POST = {"s1": 5, "s3": 4}
ARM_POST = 4                    # S2's T arm leaves V_4

# Prop bar: along the post → axis line, from the ring's outer edge on
# the post side (so it covers the post's whole end face) over the axis
# to `far` past it. S1's far end is the longest reach that stays clear
# of the turret's loaded sweep — the explorer's r 104 (`plat=short`,
# 2026-09-15) put the bar's far corner 0.3 mm outside the LOAD
# platform's corner circle, an in/out test with no margin; r 96 gives
# the 5 mm `check_turret_envelope.py` asks for, and still leaves 21 mm
# of ring past the plate's hole (r 75) for the far bolt. S3's is the
# ring's far edge less a margin (explorer: r 142).
PROP_BAR_NEAR = R_STATOR_OUTER_KINK                                  # 155
PROP_BAR_FAR = {"s1": 96.0, "s3": 142.0}
# S3's cross: two half-bars square off the prop bar's faces, out to
# the ring's edge, for the 3 kg hopper (owner 2026-09-15). Each runs
# from the prop bar's face (EXT_SIZE/2 off the axis line) to the edge.
CROSS_HALF_LEN = R_STATOR_OUTER_KINK - EXT_SIZE / 2.0                 # 145

# S2's T arm: stem square off V_4's +Y face (the m05 column's faces are
# at 0 / 90 / 180 / 270 — it is the column, not a rotatable post), to
# a T bar under S2's plate whose centreline is ARM_STEM_REACH from the
# post axis; the stem butts the bar's near face. Explorer at P: reach
# 204 post → axis, the axis 125 off the stem line, bar 310.
ARM_STEM_FACE_DEG = 90.0
ARM_STEM_REACH = 170.0
ARM_STEM_LEN = ARM_STEM_REACH - EXT_SIZE                              # 150: face to bar face
ARM_TEE_LEN = 310.0

# Post tops: the propped plate's underside less the bar lying across
# the post's end. m05 mirrors these as `THROUGH_DECK_POSTS` (height
# above its deck top); the anchors below catch drift either way.
POST_TOP_Z_LOCAL = {
    PROP_POST["s1"]: S1.plate_bottom_z - BAR_DEPTH,                   # +20.5  (world 1050.5)
    PROP_POST["s3"]: S3.plate_bottom_z - BAR_DEPTH,                   # +309.5 (world 1339.5)
}
POST_H_ABOVE_DECK = {k: z - DECK_TOP_Z_LOCAL for k, z in POST_TOP_Z_LOCAL.items()}  # 160.5 / 449.5

# S2's stem sits with its top at S2's plate underside — "S2's bracket
# at 325 above the deck" in the structure pass.
ARM_TOP_Z_LOCAL = S2.plate_bottom_z                                  # +185 (world 1215)

# Brace: one 2020 diagonal from V_4 at BRACE_H_ABOVE_DECK down to the
# deck at each of V_3 and V_5 (a V pair resists the moment in every
# direction — structure pass finding 2). 280 rather than the pass's
# 325: at the stem's height the brace ends beside the stem run into
# it, 45 mm lower they clear it (explorer `brace=5&braceH=280` clean).
# Each end is trimmed BRACE_END_TRIM along the member so it stands
# clear of the post and the deck — the end joints (a gusset or corner
# bracket) are open work; the trims are the gaps they will bridge.
BRACE_H_ABOVE_DECK = 280.0
BRACE_COLS = (3, 5)
BRACE_END_TRIM = 40.0
