"""
M02 C-channel singulator — top-level assembly.

Composes the sub-assemblies at their installed positions and runs
the interference / clearance checks that span sub-assembly
boundaries. Per-sub-system geometry and single-part printability
stay in each sub-assembly's standalone mode.

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

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

from design_systems import DESIGN_SYSTEM, color_of
from materials import material_of
from drive import assembly as _drive
from params import ROTOR_STATOR_GAP
from rotor import assembly as _rotor
from stator import assembly as _stator


GAP_CLEARANCE_TOLERANCE = 0.5   # mm — catches a shrunk V-gap early


# --- Design-system coverage --------------------------------------------
#
# Every named m02 part must appear in the active design system's
# tagging dict (``None`` allowed for hardware — kept at material's
# intrinsic color). A part added without a tag fails here instead of
# silently rendering off-palette. Swapping ``DESIGN_SYSTEM_NAME``
# (in ``cad/design_systems.py``) to a palette with missing tags also
# fails here, so the swap surface is loud.

M02_PART_NAMES = (
    list(_rotor.ROTOR_PART_NAMES)
    + list(_stator.STATOR_PART_NAMES)
    + list(_drive.DRIVE_PART_NAMES)
)
_untagged = [n for n in M02_PART_NAMES if n not in DESIGN_SYSTEM.tag_of]
assert not _untagged, (
    f"{len(_untagged)} m02 part(s) missing a tag for design system "
    f"'{DESIGN_SYSTEM.name}' in design_systems.py: {_untagged}"
)


def _kw(name: str) -> dict:
    """Color + material kwargs for the active design system, so every
    ``.with_part(...)`` site below picks them up without boilerplate."""
    return {"color": color_of(name), "material": material_of(name)}


assembly = (
    Assembly()
    .with_part("rotor", _rotor.rotor(), **_kw("rotor"))
    .with_part("stator", _stator.stator(), **_kw("stator"))
    .with_part("stem", _drive.stem(), **_kw("stem"))
    .with_part("motor", _drive.motor(), **_kw("motor"))
    # V-gap
    .assert_no_interference("rotor", "stator")
    .assert_clearance(
        "rotor", "stator",
        min_mm=ROTOR_STATOR_GAP - GAP_CLEARANCE_TOLERANCE,
    )
    # Drive vs spinning rotor
    .assert_no_interference("rotor", "stem")
    .assert_no_interference("rotor", "motor")
    # Drive vs fixed stator
    .assert_no_interference("stator", "stem")
    .assert_no_interference("stator", "motor")
    # Motor body sits on stem flange (tangent contact)
    .assert_no_interference("stem", "motor")
)


if __name__ == "__main__":
    check(assembly, out="outputs")
