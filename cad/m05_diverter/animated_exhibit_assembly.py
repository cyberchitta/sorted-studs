"""M05 diverter — time-parameterized exhibit assembly factory.

``build_exhibit_at(t)`` returns the static exhibit ``Assembly`` at
animation parameter ``t ∈ [0, 1]`` with the entire column stack
(every floor's column / hinge / ramp / trap) rotated around the
shared world-Z axis by one shared angle — the stack is mechanically
a single rigid body (base motor + bas-relief joints), so all three
columns turn together. Frame, baskets, bearings, and the top
acrylic deck stay still.

Per-floor basket selection happens through the *trap doors* in the
real design (each floor decides whether to drop or pass the brick),
not through differential ramp aim. The animation walks the shared
ramp azimuth past every basket in turn: phase A picks basket A on
F2 → F1 → F0 top-to-bottom, then phase B picks basket B on
F2 → F1 → F0. Each basket lives at a unique azimuth, so the
single shared rotation tells the story unambiguously.

The t→Assembly model is the project's animation primitive: motion is
expressed mathematically here, and ``cad_khana.export.export_animated_glb``
samples the factory at N timestamps to emit glTF TRS samplers.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from build123d import Rot  # noqa: E402

from cad_khana.mechanism.assembly import Assembly  # noqa: E402

from assembly import build_tower  # noqa: E402
from exhibit_assembly import (  # noqa: E402
    EXHIBIT_BASKETS_BY_FLOOR,
    EXHIBIT_FLOOR_PLAN,
)


# Per-column parts that rotate with the column. Identified by basename
# (the part name after stripping the `f{idx}_{role}__` floor prefix).
# bearing_spider + ring_bearing stay with the frame; baskets, frame
# horizontals, brackets, through-columns, and the top deck are all
# static.
ROTATING_BASENAMES = frozenset(
    (
        "column",
        "hinge_pin",
        "ramp",
        "trap_closed",
        "trap_open",
    )
)


def _floor_prefix(idx: int, role: str) -> str:
    return f"f{idx}_{role}__"


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

    Builds the static exhibit assembly via ``build_tower(...)``, then
    pre-multiplies the location of every rotating part on every floor
    with the shared ``Rot(0, 0, theta)``. The column stack is one
    rigid body — base motor drives the bottom column and bas-relief
    joints couple the upper columns to it — so all rotating parts on
    all three floors share the same angle.

    Assertions are stripped — the rotating ramp intentionally swings
    into basket interiors, which is the mechanism's whole point and
    would fail static interference checks.
    """
    theta = pose_at(t)
    rot = Rot(0, 0, theta)
    prefixes = {_floor_prefix(idx, role) for idx, role in EXHIBIT_FLOOR_PLAN}

    a, _ = build_tower(
        EXHIBIT_FLOOR_PLAN,
        basket_layout=EXHIBIT_BASKETS_BY_FLOOR,
        with_top_deck=True,
        with_drive_train=False,
    )

    def _new_location(p):
        for prefix in prefixes:
            if p.name.startswith(prefix):
                basename = p.name[len(prefix):]
                if basename in ROTATING_BASENAMES:
                    return rot * p.location
                break
        return p.location

    return replace(
        a,
        parts=tuple(replace(p, location=_new_location(p)) for p in a.parts),
        assertions=(),
    )


assembly = build_exhibit_at(0.0)
