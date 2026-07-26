"""
M02 C-channel singulator — top-level assembly.

Composes the sub-assemblies as subtrees (all share the m02-local
frame, so every placement is identity) and runs the interference /
clearance checks that span sub-assembly boundaries. Part identity is
the dotted tree path (``rotor.rotor``, ``drive.stem``); attributes
(role tags, materials) key on the leaf name inside each sub-module's
builder. The rotor's spin DOF is declared at composition
(``RevoluteJoint`` about the rotor-local Z axis, angle 0 in the
static rest pose). Per-sub-system geometry and single-part
printability stay in each sub-assembly's standalone mode.

Hidden-drive architecture: rotor is hollow, motor + support stem
occupy the cavity. Clearance between the rotating rotor and the
stationary drive is the critical invariant — shrinking the rotor
cavity (or growing the motor body) must fail here.

Cross-boundary checks:

- rotor ↔ stator — the V-gap. ≥ (ROTOR_STATOR_GAP − 0.5) mm.
- rotor ↔ stem   — stem threads through the rotor's open bottom
                   and up into the cavity; must not touch the
                   spinning shell anywhere.
- rotor ↔ motor  — motor body lives inside the cavity; shaft lives
                   inside the boss bore. Tight envelope — asserted
                   as no-interference only (clearance is <0.1 mm at
                   the shaft-bore fit, below noise).
- stator ↔ stem  — stem passes below the stator through the frame.
- stator ↔ motor — motor lives inside the rotor's cavity, entirely
                   inside the stator's footprint.

Deferred:
- exit_sensor ↔ stator exit arc (beam alignment)
- frame mount ↔ stator / stem
"""


from functools import cache

from build123d import Axis

from cad_khana.mechanism.assembly import Assembly, RevoluteJoint

from ..design_systems import DEFAULT_SCHEMA, paint
from .design_systems import design_system
from .drive import assembly as _drive
from .params import ROTOR_STATOR_GAP
from .rotor import assembly as _rotor
from .stator import assembly as _stator


GAP_CLEARANCE_TOLERANCE = 0.5   # mm — catches a shrunk V-gap early


# Cached because the degenerate ``assembly`` memo below and any
# composing parent (``cad/assembly.py``, ``cad/render.py``) each ask
# for the same schema, and the build is seconds of OCCT work. Replaces
# import-time memoization: the memo line is now free, and a parent
# calling the builder gets the same object rather than rebuilding.
# Safe to share — ``Assembly`` is a frozen dataclass and every
# ``with_*`` returns a copy.
#
# Call it with the schema **explicit** everywhere, including the memo
# below: ``@cache`` keys on the call signature, so ``build_x()`` and
# ``build_x(DEFAULT_SCHEMA)`` are two entries that build identical
# geometry twice. That mistake is invisible — same output, double the
# work — so ``cache_info().misses`` is the only thing that shows it.
@cache
def build_m02(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The singulator unit, painted under ``schema``.

    Sub-assembly builders emit no colour; this root paints the finished
    tree once. ``paint`` also carries the design-system coverage check
    that used to be a module-level assert here: every leaf name must be
    tagged, and a missing one names all offenders at build time instead
    of crashing the import.
    """
    return paint(
        Assembly()
        # Spin DOF declared from the start (angle 0 = static rest pose).
        # ``frame="local"``: the axis is the rotor's own Z — identical to
        # m02-local here because the subtree is placed at identity.
        .with_subassembly(
            "rotor",
            _rotor.build_rotor(),
            joint=RevoluteJoint(
                axis=Axis((0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
                angle_deg=0.0,
                frame="local",
            ),
        )
        .with_subassembly("stator", _stator.build_stator())
        .with_subassembly("drive", _drive.build_drive())
        # V-gap
        .assert_no_interference("rotor.rotor", "stator.stator")
        .assert_clearance(
            "rotor.rotor", "stator.stator",
            min_mm=ROTOR_STATOR_GAP - GAP_CLEARANCE_TOLERANCE,
        )
        # Drive vs spinning rotor
        .assert_no_interference("rotor.rotor", "drive.stem")
        .assert_no_interference("rotor.rotor", "drive.motor")
        # Drive vs fixed stator
        .assert_no_interference("stator.stator", "drive.stem")
        .assert_no_interference("stator.stator", "drive.motor")
        # Motor body sits on stem flange (tangent contact). Re-declared
        # here — drive's standalone assert rides along nested and is not
        # evaluated by this root's ``check()``.
        .assert_no_interference("drive.stem", "drive.motor"),
        design_system(schema),
    )


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``. Imported as an object by ``m02_chain`` and
# the top level.
assembly = build_m02(DEFAULT_SCHEMA)
