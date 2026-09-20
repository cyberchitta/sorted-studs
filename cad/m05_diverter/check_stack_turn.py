"""The tower's claims held over the stack turn — a check target, not
the tower.

    .venv/bin/khana check cad/m05_diverter/check_stack_turn.py

`build_m05` / `build_tower` deliberately declare **no** motion. A unit's motion is
held at every root that composes it, and `cad/assembly.py` composes
m05, so a `with_motion` inside the factory would put a 181-pose sweep
on every whole-machine check — minutes, on every run, for a DOF the
top level has nothing new to say about. cad-khana ships no switch to
opt out per root, so the only place to put it is a target the top
level does not import. This is that target.

What it adds over `khana check assembly.py`: the ~20 tower claims that
span the `rotating_stack` joint and are not rigid — column vs each
floor's bearing and ring bearing, trap servo vs the same, the drive
train's motor / mount / pinion / ring gear against the base column,
and `top_acrylic` against the top floor's column and trap servo. Every
one of those was a claim about azimuth 0 only.

What it does **not** cover: the intra-floor pairs. Those are
z-translations of one another across the four floors, so
`single_floor.py` holds them over the same turn once
(`floor_layout_sweep.py` for the other three basket layouts). Don't
mirror them here — a claim declared at two levels just evaluates twice
under two names.
"""

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.motion import Motion

from ..design_systems import DEFAULT_SCHEMA
from .assembly import build_m05


def build_stack_turn(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The production tower — exactly as `cad/assembly.py` composes it —
    with the stack turn declared on top."""
    return build_m05(schema).with_motion(
        Motion.over_joint("stack_turn", "rotating_stack", 0, 358, step=2)
    )


assembly = build_stack_turn(DEFAULT_SCHEMA)
