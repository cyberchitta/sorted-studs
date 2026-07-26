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


from build123d import Axis

from cad_khana.mechanism.assembly import Assembly, RevoluteJoint

from .design_systems import DESIGN_SYSTEM
from .drive import assembly as _drive
from .params import ROTOR_STATOR_GAP
from .rotor import assembly as _rotor
from .stator import assembly as _stator


GAP_CLEARANCE_TOLERANCE = 0.5   # mm — catches a shrunk V-gap early


assembly = (
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
    .assert_no_interference("drive.stem", "drive.motor")
)


# --- Design-system coverage --------------------------------------------
#
# Every m02 part's leaf name must appear in the active design system's
# tagging dict (``None`` allowed for hardware — kept at material's
# intrinsic color). Attributes key on the leaf (a builder binds them
# without knowing its placement path); identity stays the full path.
# A part added without a tag fails here instead of silently rendering
# off-palette; so does swapping ``DESIGN_SYSTEM_NAME`` (in
# ``cad/design_systems.py``) to a palette with missing tags.

_untagged = sorted(
    {
        leaf
        for p in assembly.placed_parts
        if (leaf := p.name.rsplit(".", 1)[-1]) not in DESIGN_SYSTEM.tag_of
    }
)
assert not _untagged, (
    f"{len(_untagged)} m02 part leaf name(s) missing a tag for design "
    f"system '{DESIGN_SYSTEM.name}' in design_systems.py: {_untagged}"
)
