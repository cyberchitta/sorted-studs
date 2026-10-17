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

    LOAD ramp   S1's chute exit sits RAMP_RUN from LOAD at azimuth
                RAMP_AZIMUTH_DEG (φ); the ramp runs from there down to
                an outlet RAMP_OUTLET_X out along the same line — just
                outside the turret's loaded-platform sweep, so a
                platform leaving LOAD with a brick never passes under
                the lip
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
    LOAD ramp outlet          (74.5, 0, +7)            (world (199.5, 125, 1037))
    S1 chute exit = inlet     (130, 0, +65)            (world (255, 125, 1095))
    S1 axis                   (103, -153), V-bottom +65    (world (228, -28, 1095))
    S2 axis                   (0, -275),   V-bottom +209.5 (world (125, -150, 1239.5))
    S3 axis                   (-150, -330), V-bottom +354  (world (-25, -205, 1384))
    V_5, S1's post            (145, -281); top +20.5   (world 1050.5 = 160.5 over the deck)
    V_4, S3's post + S2's arm (-125, -437); top +309.5 (world 1339.5 = 449.5 over the deck)

Gantry (owner 2026-09-14/15, `m02-chain.md` §6; sizes from the
structure pass in `_notes/decisions/m02-chain.md` §6.1, hand statics):
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
    HOPPER_WALL_H,
    STATOR_WALL_H,
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

# Deck disc (`m05_diverter/params.py:TOP_DECK_OD`,
# `m05_diverter/assembly.py:TOP_DECK_APERTURE_D`), used only by the
# standalone fixture's deck stand-in — a drift here changes the
# picture, not a claim (no anchor derives from it).
M05_TOP_DECK_OD = 720.0
M05_TOP_DECK_APERTURE_D = 150.0
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

# The platform's acrylic square (`m03_scanner/params.py:
# PLATFORM_ACRYLIC_SIZE`). With the arm radius it gives the corner
# circle the loaded platforms sweep about the hub — the outer radius of
# the probe in `cad/check_turret_envelope.py`, which also takes its
# clearance from TURRET_SWEEP_CLEAR here so the ramp outlet and the
# check agree by construction.
M03_PLATFORM_SIZE = 120.0
M03_PLATFORM_CORNER_R = math.hypot(
    M03_TURRET_ARM_R + M03_PLATFORM_SIZE / 2.0, M03_PLATFORM_SIZE / 2.0,
)                                                                     # ≈ 194.5
TURRET_SWEEP_CLEAR = 5.0


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


def m05_column_rot_deg(k: int) -> float:
    """Yaw of m05 outer column k about its own axis: the frame turns
    each column to its vertex azimuth (`m05_diverter/frame/assembly.py`
    places `outer_vert_k` with `Rot(0, 0, v_angle)`), so its four
    faces sit at (k + 0.5)·60° + {0, 90, 180, 270}. V_4's come out
    axis-aligned (270 ≡ 0 mod 90); V_5's sit at 60 / 150 / 240 / 330.
    Mirrored so the printed post seats' sockets and the standalone
    stubs take the column's real orientation; drift shows at the top
    level as a seat overlapping its column."""
    return (k + 0.5) * 60.0


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

# The chute exit's height above the V-bottom. 0 since 2026-09-15: the
# stator's exit floor (`m02_c_channel_singulator/stator/assembly.py:
# exit_floor`) is a tongue at the V-bottom plane out to the kink
# radius, and S1's ramp starts flush with it 1 mm further out, so a
# brick slides from the V onto the ramp with no step. (Was 10, a
# 2026-05 leftover from before the rim drive put a plate under the
# exit — with it the ramp floor began 10 mm above anything a brick at
# the V-bottom could reach.)
CHUTE_EXIT_ABOVE_VBOT = 0.0

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
    bowl_seam_phase_deg: float  # m02-local; passed to build_m02
    wall_h: float               # liner height; passed to build_m02

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


# --- Seams ---------------------------------------------------------------

# Per-stage phase of the bowl's thirds seams, m02-local degrees from
# the exit arc's centre (`m02_c_channel_singulator/seams.py`,
# 2026-09-17). A seam must not land where a gantry member ends under
# the floor — a bar end is a point load on one sector's floor edge,
# where the ring beam is already cut — nor anywhere along a member's
# run under the floor when the member stands beside the axis and so
# sweeps a few degrees of it (S2's T), nor through the exit slot or
# the motor (the unit guards those two), and it stays staggered from
# the liner's fixed seams at 75 / −165 / −45 (the unit asserts ≥ 25°).
# Stage-local end azimuths, found by search 2026-09-17: S1 its prop
# bar's post end −152 and far end +28; S2 the T bar's ends −47 / +127
# and the stem's end at the T; S3 its prop bar −97 / +83 and the cross
# halves' outer ends −7 / +173. The chosen phases put every seam ≥ 9°
# (≈ 24 mm at the rim) past the nearest member's side; one phase for
# all three would have left 5°. `assembly.py` records each stage's
# worst case as `sN_bowl_seams_clear_of_bar_ends_deg`.
BOWL_SEAM_PHASE_DEG = {"s1": 41.0, "s2": 31.0, "s3": 38.0}
BOWL_SEAM_BAR_CLEAR_MIN_DEG = 5.0


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
            bowl_seam_phase_deg=BOWL_SEAM_PHASE_DEG[name],
            # The top stage takes the bulk pile (`m02-chain.md` §1).
            wall_h=HOPPER_WALL_H if i == N_STAGES - 1 else STATOR_WALL_H,
        ))
    return tuple(stages)


STAGES: tuple[Stage, ...] = _stage_table()
S1, S2, S3 = STAGES


# --- LOAD ramp ---------------------------------------------------------

# Outlet: on the ramp line, RAMP_OUTLET_X out from the LOAD centroid —
# the loaded platform's corner circle plus the sweep clearance, so the
# lip stands just outside everything a platform leaving LOAD (brick on
# board) sweeps. Decided 2026-09-15 (owner; the alternative was to
# raise the outlet above the brick envelope): until then the outlet
# hung over the platform's centre, 7 mm up, and a 30 mm brick under
# the lip was dragged when the platform indexed away. The corner
# circle, not the platform's edge midpoint (60), sets it: the trailing
# corner sweeps through the ramp line at r 194.5 as the platform
# leaves. The lip is therefore 14.5 mm outside the platform's edge —
# the brick has to carry itself onto the platform (the 7 mm fall at
# the ramp's exit speed) and slide in toward the cameras' centre.
# That landing is unmodelled and unverifiable before a bench exists.
#
# Outlet z is offset RAMP_FLOOR_T + 2 mm above the LOAD platform top
# face so the ramp's BOTTOM face clears the platform acrylic by 2 mm
# (the acrylic insert occupies platform-local z ∈ [-4, 0], with the
# top face at the platform's reference plane). The brick contact
# surface at the outlet sits 7 mm above the acrylic top.
RAMP_FLOOR_T = 5.0
RAMP_OUTLET_X = M03_PLATFORM_CORNER_R + TURRET_SWEEP_CLEAR - M03_TURRET_ARM_R   # ≈ 74.5
RAMP_OUTLET_LOCAL = (
    RAMP_OUTLET_X * math.cos(math.radians(RAMP_AZIMUTH_DEG)),
    RAMP_OUTLET_X * math.sin(math.radians(RAMP_AZIMUTH_DEG)),
    RAMP_FLOOR_T + 2.0,                                               # +7  (world z = 1037)
)
RAMP_INLET_LOCAL = (*S1.chute_exit_xy, S1.chute_exit_z)               # (130, 0, 65)

RAMP_PLAN_RUN = RAMP_RUN - RAMP_OUTLET_X                              # ≈ 55.5
RAMP_DZ = RAMP_INLET_LOCAL[2] - RAMP_OUTLET_LOCAL[2]                  # 58
RAMP_PITCH_DEG = math.degrees(math.atan2(RAMP_DZ, RAMP_PLAN_RUN))     # ≈ 46.3
RAMP_SLOPE_LEN = math.sqrt(RAMP_PLAN_RUN**2 + RAMP_DZ**2)             # ≈ 80.3

# Pitch window — placeholders, not measured friction (LEGO on PLA
# static friction angle is ~20–25°; 30° was the earlier target). The
# ceiling was 40 until the outlet moved out (2026-09-15), which put the
# pitch at ≈ 51; widened to 55 as the same kind of placeholder, not as
# a measurement — what a brick does leaving a ramp this steep is a
# bench question.
RAMP_PITCH_MIN_DEG = 20.0
RAMP_PITCH_MAX_DEG = 55.0

# At family P the chute faces sideways to the ramp (S1 heads at 80°,
# the ramp runs at 180°): the brick leaves the rotor wall *tangentially*
# down the ramp axis (10° off it), through an exit arc that lies along
# the ramp rather than across it. So the ramp is a tangential catch:
# its inner side is S1's stator wall — the ramp is cut back to
# RAMP_STATOR_CLEAR outside the kink radius about S1's axis — and a
# flat landing at the exit height continues past the exit centre to
# the far end of the exit arc, so the whole opening has floor under it.
# (A straight 100-wide chute from the exit cut into S1's own plate and
# rotor on the first check, 4256 / 5114 mm³ — hence the catch.)
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
# How much of the outlet end, in plan from the exit edge, the
# `load_ramp@exit_lip` region names — the feature the top level holds
# against m03's LOAD platform. Long enough to hold the floor's end face
# (it leans back RAMP_FLOOR_T·sin(pitch) ≈ 3.6 under the exit edge).
RAMP_LIP_LEN = 5.0


# --- LOAD beam-break (option A, owner 2026-10-17) ----------------------

# The brick-released sensor: a through-beam across the chute near the
# outlet, square to the run and parallel to the floor. Emitter outboard
# of the +Y side wall, looking through a hole in it; receiver lying on a
# tab grown off the floor's −Y edge (the half-chute has no −Y wall there
# — the stator cut took it), looking back across the chute. The tab is
# a shelf: the floor carried on past its edge, floor-thick and flush with
# the floor top, so it adds no face that hangs down when the ramp prints
# on its side (chain −Y up, `printability.py`). A wall behind the
# receiver with a hole in it was tried first (2026-10-17): its
# chute-side face must lean out ≥ 45° from vertical in print, and the
# hole breaking that slant leaves a 40° wedge at its rim (min wall 1.39,
# alignment 0.77 = cos 40°) — red without a waiver.
#
# Where the beam sits, chute-local (from the exit edge): BEAM_S up the
# slope (≈ 6.9 in plan, 7.2 in z) and, because the receiver lies on the
# tab, one sensor radius above the floor top.
BEAM_S = 10.0
# The thinnest brick that has to break the beam: a plate or tile lying
# flat (3.2 mm), on a 1×1's 7.8 side. The beam's core must lie wholly
# under it; the claim is `check_load_beam.py`'s.
THINNEST_BRICK_H = 3.2
THINNEST_BRICK_W = 7.8
# The emitter's aperture through the +Y wall (the beam's full width at
# the chute), and the part of it a brick has to cover for the receiver
# to count a break — a placeholder for the receiver's threshold, not a
# measurement.
BEAM_HOLE_D = 3.0
BEAM_CORE_D = 1.5

# Bought sensor envelope — PLACEHOLDER dims for a generic 3 mm IR
# LED / phototransistor pair (T-1 package): flange Ø3.8, body + dome
# 5.3 long. Behind the flange, SENSOR_LEAD_L is how far the leads stand
# out along the beam before they turn: bent 90° about 1 mm behind the
# flange and soldered to the wire, heat-shrunk, the bend radius and the
# sleeve's depth make ~3 (owner 2026-10-17: shortened from 8, a straight
# lead into a 2-pin connector, which put the emitter's end 1.7 from the
# +X service reach). The wire's run beyond the bend is not modelled.
# Each half is one cylinder of SENSOR_ENV_D × SENSOR_ENV_L on the beam
# axis.
# Derived from these: the envelopes, and so every clearance claim that
# reads them; the beam's height (BEAM_H, the receiver lying on the
# tab), and so whether the thinnest brick still covers its core — a
# 5 mm package (Ø5.8) puts the core's top at 3.65, over a plate, and
# `check_load_beam.py` goes red; and the tab's and the guard's width.
# The receiver is captive in the guard's pocket (below); how the emitter
# is held on the +Y wall is not modelled.
SENSOR_FLANGE_D = 3.8
SENSOR_BODY_L = 5.3
SENSOR_LEAD_L = 3.0
SENSOR_ENV_D = SENSOR_FLANGE_D
SENSOR_ENV_L = SENSOR_BODY_L + SENSOR_LEAD_L                          # 8.3
BEAM_H = SENSOR_ENV_D / 2.0                                           # 1.9

# The receiver guard (owner 2026-10-17): a separate printed block on the
# tab that shields the receiver from a brick overhanging the floor's −Y
# edge, holds it captive in a pocket against the tab, and — because the
# same block runs on up the slope until it meets S1's bowl overhead — is
# also the ramp's −Y side wall over the run the stator cut left open.
# Chute-local: its chute face is the plane through the receiver's front
# (the floor's −Y edge at the exit edge, its innermost point, so the
# guard lies wholly inside the stator cut and a strip of tab, flush with
# the floor, runs between the two), its back BEAM_TAB_LIP past the
# receiver's. It stands GUARD_H above the floor top from GUARD_S0 up the
# slope (a square end there; the outlet's vertical plane trims it above),
# capped RAMP_STATOR_CLEAR under S1's bowl floor, and ends square where
# that cap leaves GUARD_END_H of it. It prints on its chute face (the
# guard is a prism across the chute, so every other face stands
# vertical); the pocket runs along the up axis.
GUARD_H = 20.0                   # = the +Y wall's RAMP_WALL_H
GUARD_S0 = 3.0                   # its square end stands GUARD_S0 / tan(pitch) ≈ 2.9 before the trim
GUARD_END_H = 3.0
GUARD_POCKET_CLEAR = 0.2         # radial, round the receiver: a slip fit
GUARD_TOP_Z_LOCAL = S1.vbot_z + PLATE_BOTTOM_Z - RAMP_STATOR_CLEAR    # 39.5: S1's bowl floor less the cut clearance
GUARD_S_END = (
    GUARD_TOP_Z_LOCAL - RAMP_OUTLET_LOCAL[2] - GUARD_END_H * math.cos(math.radians(RAMP_PITCH_DEG))
) / math.sin(math.radians(RAMP_PITCH_DEG))                            # ≈ 42.1

# The tab under it: the floor carried on past its −Y edge, from the exit
# edge to the guard's end, out to the guard's back.
BEAM_TAB_LEN = GUARD_S_END
BEAM_TAB_LIP = 2.0

# The guard is held by two screws up through the tab into heat-set
# inserts in its underside (the same M3 insert as S1's liner free ends,
# `m02_c_channel_singulator/params.py` ARC_INSERT_*): the screw is the
# only fixing that both locates (two of them, along the slope, fix it in
# plan) and holds against a brick's knock, and the insert puts the
# thread in the thick guard rather than the 5 mm tab. Bought: ISO 4762
# M3×10 and an M3 × 5.7 brass insert. The screw heads sit on the tab's
# underside. In the ramp's print the clearance holes run across the up
# axis, so they are teardrops roofed GUARD_HOLE_ROOF_DEG toward it; in
# the guard's, so are the insert holes.
GUARD_SCREW_D = 3.0
GUARD_SCREW_HEAD_D = 5.5         # ISO 4762 M3 dk
GUARD_SCREW_HEAD_H = 3.0         # ISO 4762 M3 k
GUARD_SCREW_L = 10.0
GUARD_SCREW_HOLE_D = 3.4         # ISO 273 medium clearance, through the tab
GUARD_INSERT_L = 5.7
GUARD_INSERT_HOLE_D = 4.0
GUARD_INSERT_HOLE_DEPTH = 6.0    # ≥ GUARD_INSERT_L, and past the screw's tip (GUARD_SCREW_L − RAMP_FLOOR_T = 5)
GUARD_SCREW_S = (16.0, 24.0)     # up the slope from the exit edge, upstream of the receiver
GUARD_HOLE_ROOF_DEG = 40.0


# --- Gantry ------------------------------------------------------------

# 2020 aluminum extrusion — matches m05's EXT_SIZE.
EXT_SIZE = 20.0
BAR_DEPTH = EXT_SIZE            # bars lie flat under the plates

# Which m05 outer column props which stage (owner 2026-09-14).
PROP_POST = {"s1": 5, "s3": 4}
ARM_POST = 4                    # S2's T arm leaves V_4

# Prop bar: along the post → axis line, from the ring's outer edge on
# the post side (so it covers the post's whole end face) over the axis
# to `far` past it. Pointing at the axis is deliberate (owner
# 2026-09-18, `_notes/decisions/m02-chain.md` §6): the bar then only
# bends. It leaves the bar yawed to its column's face normals (42° on
# V_5, 13° on V_4), so no 90° corner bracket seats under it and a
# printed seat stands in — but a bar squared to a face stands beside
# the axis and carries the load as twist, and a 2020 is ~10× softer in
# twist than in bending (It 0.07 vs Ix 0.72 cm⁴). Squaring S3 was
# built and checked green, then reverted on that arithmetic.
# S1's far end is the longest reach that stays clear
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
# Accepted by the owner 2026-09-15. Re-figured 2026-09-18 by hand
# (`_notes/state/m02-singulator.md` §Open): 0.45 mm / ~25 Hz at the
# top with rigid brace ends, but the printed sockets are in series
# with the brace and it is their stiffness, not the aluminium's, that
# sets what the brace buys — an unmeasured number.
BRACE_H_ABOVE_DECK = 280.0
BRACE_COLS = (3, 5)

# Brace ends (2026-09-17). Each brace meets its columns at a compound
# angle — the V_4 → V_k run is 30° off every column face in plan and
# the member descends at ~42° — so no standard 90° corner bracket or
# single-axis hinge joint lands on it. The ends are printed sockets
# (`joints.py`: a collar on V_4 with a U-channel per brace,
# a foot block on the deck at each V_k) and the brace's two end faces
# are cut VERTICAL, each at a fixed horizontal distance from the
# column axis it is nearest, so the end bears full-face on a vertical
# stop in the socket. The head stop sits past V_4's collar along the
# 30° ray (16 / cos 30° = 18.5); the foot stop keeps the brace's low
# corner above the deck (its underside meets the deck plane 14.9 out
# from the foot's axis) and past V_5's through-deck vertex bracket
# (`m05.top_cap.outer_vertex_bracket_5`), whose vertical arm rises 50
# above the deck on the column's OUTWARD face — clear from 15 out
# along the brace's ray (probed 2026-09-17), so the V_5 foot block
# starts at BRACE_FOOT_START_H and takes no collar. V_3 has no post
# above the deck: its foot block runs over the column axis (an
# end-tap bolt through the deck is the detail-layer fastening) with
# a flange behind it.
BRACE_HEAD_STOP_H = 25.0
BRACE_FOOT_STOP_H = 30.0
BRACE_FOOT_START_H = {3: 0.0, 5: 15.0}

# V_3's foot is bolted down (owner 2026-10-17): one M5 socket-head cap
# screw (ISO 4762) on an ISO 7093 wide washer, down the column axis
# through the foot's flange and m05's deck into V_3's tapped centre bore
# — it clamps the foot and the deck to the column together. Bought;
# cheap envelopes (head and washer cylinders, shank at the thread's
# major Ø). The same M5 family as m05's corner bolts
# (`m05_diverter/assembly.py` DECK_BOLT_*), kept here so the chain
# builds standalone; the top level's claims that this shank fills the
# column's tapped bore and centres in the deck's hole are what catch a
# mismatch. The foot's hole, its washer seat, the deck stand-in's hole
# and the length check derive from these.
FOOT_BOLT_COL = 3
FOOT_BOLT_D = 5.0             # M5 thread major Ø
FOOT_BOLT_HEAD_D = 8.5        # ISO 4762 M5 dk
FOOT_BOLT_HEAD_H = 5.0        # ISO 4762 M5 k
FOOT_BOLT_L = 25.0            # a stock size
FOOT_BOLT_WASHER_OD = 15.0    # ISO 7093 M5 d2
FOOT_BOLT_WASHER_ID = 5.3     # ISO 7093 M5 d1
FOOT_BOLT_WASHER_T = 1.2      # ISO 7093 M5 h
FOOT_BOLT_HOLE_D = 5.5        # ISO 273 medium clearance, in the foot and in m05's deck
# Radial room round the washer in the foot's seat, a counterbore down
# to the flange's top face through the block that rises over the axis.
FOOT_BOLT_SEAT_CLEAR = 1.0
# Thread engaged in the column's tapped bore, at least (steel into
# aluminium, 1.5 Ø).
FOOT_BOLT_ENGAGE_MIN = 1.5 * FOOT_BOLT_D
# Floor on the head's gap to the brace it sits beside.
FOOT_BOLT_BRACE_CLEAR = 5.0


def brace_run(col: int) -> tuple[float, float, float]:
    """(azimuth from V_4 toward V_col in degrees, descent angle in
    degrees, horizontal run) of the brace to outer vertex `col`."""
    post = m05_outer_vertex_local_xy(ARM_POST)
    foot = m05_outer_vertex_local_xy(col)
    dx, dy = foot[0] - post[0], foot[1] - post[1]
    run = math.hypot(dx, dy)
    return (
        math.degrees(math.atan2(dy, dx)),
        math.degrees(math.atan2(BRACE_H_ABOVE_DECK, run)),
        run,
    )


def prop_bar_azimuth_deg(stage: Stage) -> float:
    """Azimuth of the stage's prop bar, post → axis, chain-local."""
    post = m05_outer_vertex_local_xy(PROP_POST[stage.name])
    return math.degrees(math.atan2(
        stage.axis_xy[1] - post[1], stage.axis_xy[0] - post[0],
    ))
