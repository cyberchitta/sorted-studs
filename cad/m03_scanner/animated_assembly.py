"""M03 scanner — time-parameterized assembly factory.

`build_m03_at(t)` returns the static m03 ``Assembly`` at animation
parameter ``t ∈ [0, 1]``, with the rotor (spider + platforms +
kicker + shaft + flag + shaft pulley) revolved one full turn around
the hub Z axis. The turret floor frame, hub housing, bearings, dump
cam, drive motor + bracket, and homing sensor + bracket all stay
fixed — they're bolted to the deck, not the spider.

Used by the GLB animation exporter (see ``export_glb_animated.py``)
and as the unit of regression insurance for `khana check` across
mid-motion frames.

The t→Assembly model is the project's animation primitive: motion is
expressed mathematically here (``theta = 360 * t``), the cad-khana
sweep is a thin wrapper that samples ``factory(t)`` at N values.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from build123d import Rot  # noqa: E402

from cad_khana.mechanism.assembly import Assembly  # noqa: E402
from cad_khana.mechanism.check import check  # noqa: E402

from assembly import (  # noqa: E402
    TURRET_ARM_NAMES,
    TURRET_BASE,
    TURRET_DUMP_KICKER_NAMES,
    TURRET_PLATFORM_NAMES,
    add_m03,
    add_m03_internal_checks,
)

# Parts that revolve with the spider. Hub bearings stay with the
# housing — physically the outer race is static and the inner spins
# with the shaft, but the bearing-as-a-whole is rendered as one body,
# and the visual cost of pinning it to the housing side is nil.
ROTATING_TURRET_PARTS = frozenset(
    (
        "hub_shaft",
        "drive_pulley_shaft",
        "homing_flag",
        *TURRET_ARM_NAMES,
        *TURRET_PLATFORM_NAMES,
        *TURRET_DUMP_KICKER_NAMES,
    )
)


def build_m03_at(t: float) -> Assembly:
    """``t ∈ [0, 1]`` ↔ one full rotor revolution around the hub Z axis.

    Builds the static m03 assembly at θ=0, then conjugates each rotating
    part's m03-local location through ``TURRET_BASE * Rot(0,0,θ) *
    TURRET_BASE⁻¹`` — a rotation around the hub axis expressed in
    m03-local. Internal checks are added afterward; they're name-based,
    so the location shuffle leaves them well-formed."""
    theta_deg = 360.0 * t
    a = Assembly()
    a = add_m03(a)  # turret + rig at θ=0
    # Strip back to just the placements; we'll re-add assertions last.
    a = replace(a, assertions=())
    if theta_deg != 0.0:
        rotor = TURRET_BASE * Rot(0, 0, theta_deg) * TURRET_BASE.inverse()
        a = replace(
            a,
            parts=tuple(
                replace(p, location=rotor * p.location)
                if p.name in ROTATING_TURRET_PARTS
                else p
                for p in a.parts
            ),
        )
    a = add_m03_internal_checks(a)
    return a


assembly = build_m03_at(0.0)


if __name__ == "__main__":
    # Per-frame static check at sampled t — regression insurance for
    # mid-motion interferences. The static m03 is already clean; this
    # catches any rotor pose where the spider or a platform pin clips a
    # rig member.
    out_root = HERE / "outputs" / "animated"
    for t in (0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875):
        check(build_m03_at(t), out=str(out_root / f"t_{t:.3f}"))
