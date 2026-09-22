"""The singulator's claims held over one facet period of the rotor's
spin — a check target, not the unit.

    .venv/bin/khana check cad/m02_c_channel_singulator/check_rotor_turn.py

`build_m02` declares the ``rotor`` joint and **no** motion, on purpose:
`m02_chain` and `cad/assembly.py` compose it three times, and a unit's
motion is held at every root that composes it. So `khana check
assembly.py` evaluates every rotor claim at angle 0 only — a peak
facing +X, which puts a star *valley* (18°) at the exit paddle, the
most generous pose for the claim `EXIT_WALL_ROTOR_GAP` sizes against a
worst case. This target turns the rotor through the rest.

One period is every distinct pose: the rotor is exactly
`ROTOR_FACET_COUNT`-fold (`rotor()` — star, top polygon and cavity all
on 36° spacing, the flange a plain annulus), so a turn of 360/N maps it
onto itself. Nothing else in the unit is jointed, so only the claims
that reference ``rotor.rotor`` move.

What it found (2026-09-22): the rest pose was not generous. All 15
moved claims read the same at every sampled angle, because each one's
closest point sits on the rotor's circular rim (z = 0), which the spin
does not change — the facets only open the gap above it. That includes
the paddle, which no claim here can isolate (it is unioned into its
liner sector): probed alone, 2.121 mm (= EXIT_WALL_ROTOR_GAP/√2) at
every degree. So this target is a regression net — it goes live the
day a facet reaches the rim (``peak_bulge > 0``, a kink at z = 0).
"""

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.motion import Motion

from .assembly import build_m02
from .design_systems import DEFAULT_SCHEMA
from .params import ROTOR_FACET_COUNT

# 1° is ≈ 2.1 mm of travel at the rim (r 120) — half the valley dent,
# the smallest feature whose position the claims depend on.
ROTOR_TURN_STEP_DEG = 1.0


def build_rotor_turn(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The production unit — exactly as the chain composes it — with
    one facet period of rotor spin declared on top."""
    return build_m02(schema).with_motion(
        Motion.over_joint(
            "rotor_turn", "rotor", 0.0, 360.0 / ROTOR_FACET_COUNT,
            step=ROTOR_TURN_STEP_DEG,
        )
    )


assembly = build_rotor_turn(DEFAULT_SCHEMA)
