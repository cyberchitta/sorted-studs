"""M05 diverter — time-parameterized exhibit assembly factory.

``build_exhibit_at(t)`` returns the static exhibit ``Assembly`` at
animation parameter ``t ∈ [0, 1]`` with the ``rotating_stack``
sub-assembly's revolute joint set to one shared azimuth, and each
floor's ``trap`` joint set from the trap schedule. The column-side
stack (every floor's column / ramp / servo) is one rigid body (base
motor + bas-relief joints), so a single joint DOF carries the
azimuth; the traps are the only other DOFs, one per floor.

Per-floor basket selection happens through the *trap doors* in the
real design (each floor decides whether to drop or pass the brick),
not through differential ramp aim. The animation walks the shared
ramp azimuth past every basket in turn: phase A picks basket A on
F2 → F1 → F0 top-to-bottom, then phase B picks basket B on
F2 → F1 → F0. Each basket lives at a unique azimuth, so the
single shared rotation tells the story unambiguously. While the ramp
dwells on a floor's basket, that floor's trap swings up from hanging
to its catch pose and back; every other trap hangs (pass-through).

The t→Assembly model is the project's animation primitive: motion is
expressed mathematically here, and ``cad_khana.export.export_animated_glb``
samples the factory at N timestamps and emits one rotation channel
on the ``rotating_stack`` parent node — the structural-emission path,
free of trajectory inference and chord-vs-arc drift.
"""

from __future__ import annotations

import math
from dataclasses import replace


from cad_khana.mechanism.assembly import Assembly

from ..design_systems import DEFAULT_SCHEMA
from .assembly import build_tower
from .ramp_mechanism.assembly import TRAP_CATCH_DEG, TRAP_REST_DEG
from .exhibit_assembly import (
    EXHIBIT_BASKETS_BY_FLOOR,
    EXHIBIT_FLOOR_PLAN,
)


# Per-floor basket azimuths. Mirrors ``EXHIBIT_BASKETS_BY_FLOOR``
# (first entry is phase A's target on each floor, second is phase B's).
_PHASE_A = {idx: layout[0][0] for idx, layout in EXHIBIT_BASKETS_BY_FLOOR.items()}
_PHASE_B = {idx: layout[1][0] for idx, layout in EXHIBIT_BASKETS_BY_FLOOR.items()}

# Six keyframe column azimuths walking the ramp through every basket.
# The whole column stack rotates as one rigid body (single base motor +
# bas-relief joints couple all three columns), so each pose is a
# single shared angle — not a per-floor tuple. Each basket lives at a
# unique azimuth, so one shared rotation still walks the ramp past
# every basket in turn; the reader sees "selecting one basket per
# floor" through which floor that azimuth happens to host.
#
# Phase A (top → bottom): basket A on F2, F1, F0.
# Phase B (top → bottom): basket B on F2, F1, F0.
# POSES[6] implicitly == POSES[0], closing the loop.
POSES: list[float] = [
    _PHASE_A[2],   # F2 basket A
    _PHASE_A[1],   # F1 basket A
    _PHASE_A[0],   # F0 basket A
    _PHASE_B[2],   # F2 basket B
    _PHASE_B[1],   # F1 basket B
    _PHASE_B[0],   # F0 basket B
]
N_STEPS = len(POSES)
# The floor whose basket each step selects — the floor whose trap catches.
CATCH_FLOOR: list[int] = [2, 1, 0, 2, 1, 0]

# Fraction of each step spent parked at the target (the "ramp is
# selecting this basket" beat). The remainder is the smooth sweep to
# the next step's target.
DWELL_FRAC = 0.55

# Trap beat inside the dwell, as fractions of the step: swing up to the
# catch pose, hold, swing back down to hanging before the column moves.
TRAP_UP = (0.05, 0.15)
TRAP_DOWN = (0.40, 0.50)
assert TRAP_DOWN[1] <= DWELL_FRAC   # the trap is down before the turn


def _lerp_angle(a: float, b: float, s: float) -> float:
    """Shortest-arc interpolation between two angles in degrees."""
    delta = ((b - a + 180.0) % 360.0) - 180.0
    return a + delta * s


def _step(t: float) -> tuple[int, float]:
    """``t ∈ [0, 1]`` → ``(step_idx ∈ [0, N_STEPS-1], local_t ∈ [0, 1))``.
    Maps ``t == 1.0`` back to ``(0, 0.0)`` so the animation loops."""
    if t >= 1.0:
        return 0, 0.0
    step_idx = int(t * N_STEPS)
    return step_idx, t * N_STEPS - step_idx


def pose_at(t: float) -> float:
    """Shared column rotation angle (degrees) at animation time t."""
    step_idx, local = _step(t)
    current = POSES[step_idx]
    if local < DWELL_FRAC:
        return current
    s = (local - DWELL_FRAC) / (1.0 - DWELL_FRAC)
    nxt = POSES[(step_idx + 1) % N_STEPS]
    return _lerp_angle(current, nxt, s)


def _ease(s: float) -> float:
    """Cosine ease-in-out on ``s ∈ [0, 1]``."""
    return 0.5 - 0.5 * math.cos(math.pi * min(max(s, 0.0), 1.0))


def trap_angle_at(t: float, floor: int) -> float:
    """``floor``'s trap joint angle (degrees) at animation time t:
    TRAP_REST_DEG (hanging) except during its own step's catch beat."""
    step_idx, local = _step(t)
    if CATCH_FLOOR[step_idx] != floor:
        return TRAP_REST_DEG
    up = _ease((local - TRAP_UP[0]) / (TRAP_UP[1] - TRAP_UP[0]))
    down = _ease((local - TRAP_DOWN[0]) / (TRAP_DOWN[1] - TRAP_DOWN[0]))
    s = up - down
    return TRAP_REST_DEG + (TRAP_CATCH_DEG - TRAP_REST_DEG) * s


def build_exhibit_at(t: float, schema: str = DEFAULT_SCHEMA) -> Assembly:
    """Animated exhibit assembly at time ``t ∈ [0, 1]``.

    Rebuilds the exhibit assembly via ``build_tower(...)`` and sets the
    ``rotating_stack`` sub-assembly's joint angle to ``pose_at(t)``
    and each floor's trap to ``trap_angle_at(t, floor)``.
    Assertions are stripped — the rotating ramp intentionally swings
    into basket interiors, which is the mechanism's whole point and
    would fail static interference checks.
    """
    a = build_tower(
        EXHIBIT_FLOOR_PLAN,
        schema=schema,
        basket_layout=EXHIBIT_BASKETS_BY_FLOOR,
        with_top_deck=True,
        with_drive_train=False,
    )
    a = a.with_joint_angle("rotating_stack", pose_at(t))
    for floor in EXHIBIT_BASKETS_BY_FLOOR:
        a = a.with_joint_angle(
            f"rotating_stack.f{floor}.trap", trap_angle_at(t, floor),
        )
    return replace(a, assertions=())


# Degenerate memoized master at t=0. Assertion-free by design — see
# the docstring; a green ``khana check`` here proves nothing.
assembly = build_exhibit_at(0.0)
