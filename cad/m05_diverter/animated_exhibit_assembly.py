"""M05 diverter — time-parameterized exhibit assembly factory.

``build_exhibit_at(t)`` returns the static exhibit ``Assembly`` at
animation parameter ``t ∈ [0, 1]`` with the ``rotating_stack``
sub-assembly's revolute joint set to one shared azimuth. The whole
column-side stack (every floor's column / hinge / ramp / trap) is one
rigid body (base motor + bas-relief joints), so a single joint DOF
captures the motion.

Per-floor basket selection happens through the *trap doors* in the
real design (each floor decides whether to drop or pass the brick),
not through differential ramp aim. The animation walks the shared
ramp azimuth past every basket in turn: phase A picks basket A on
F2 → F1 → F0 top-to-bottom, then phase B picks basket B on
F2 → F1 → F0. Each basket lives at a unique azimuth, so the
single shared rotation tells the story unambiguously.

The t→Assembly model is the project's animation primitive: motion is
expressed mathematically here, and ``cad_khana.export.export_animated_glb``
samples the factory at N timestamps and emits one rotation channel
on the ``rotating_stack`` parent node — the structural-emission path,
free of trajectory inference and chord-vs-arc drift.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from cad_khana.mechanism.assembly import Assembly  # noqa: E402

from assembly import build_tower  # noqa: E402
from exhibit_assembly import (  # noqa: E402
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

# Fraction of each step spent parked at the target (the "ramp is
# selecting this basket" beat). The remainder is the smooth sweep to
# the next step's target.
DWELL_FRAC = 0.55


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


def build_exhibit_at(t: float) -> Assembly:
    """Animated exhibit assembly at time ``t ∈ [0, 1]``.

    Rebuilds the exhibit assembly via ``build_tower(...)`` and sets the
    ``rotating_stack`` sub-assembly's joint angle to ``pose_at(t)``.
    Assertions are stripped — the rotating ramp intentionally swings
    into basket interiors, which is the mechanism's whole point and
    would fail static interference checks.
    """
    a, _ = build_tower(
        EXHIBIT_FLOOR_PLAN,
        basket_layout=EXHIBIT_BASKETS_BY_FLOOR,
        with_top_deck=True,
        with_drive_train=False,
    )
    a = a.with_joint_angle("rotating_stack", pose_at(t))
    return replace(a, assertions=())


assembly = build_exhibit_at(0.0)
