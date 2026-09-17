"""
M02 C-channel singulator — top-level assembly.

Composes the sub-assemblies as subtrees (all share the m02-local
frame, so every placement is identity) and runs the interference /
clearance checks that span sub-assembly boundaries. Part identity is
the dotted tree path (``rotor.rotor``, ``drive.bowl_0``,
``drive.pinion.pinion``); attributes (role tags, materials) key on
the leaf name inside each sub-module's builder. The rotor's spin DOF
is declared at composition (``RevoluteJoint`` about the rotor-local Z
axis, angle 0 in the static rest pose); the pinion's is declared in
``drive/``.

Rim-drive architecture (2026-09-11): the rotor carries a ring-gear
flange below its base rim, rests on a turntable bearing on the base
plate, and is driven by an off-axis pinion under the stator wall.
The stack is documented in ``params.py``.

Cross-boundary checks:

- rotor ↔ stator   — the V-gap. ≥ (ROTOR_STATOR_GAP − 0.5) mm.
- rotor ↔ pinion   — the gear mesh: pitch cylinders MESH_BACKLASH
                     apart, close but not touching.
- rotor ↔ bearing  — the flange underside rests on the bearing
                     (tangent contact).
- rotor ↔ bowl     — separated by the bearing height.
- rotor ↔ motor    — the motor hangs under the floor, well clear.
- stator ↔ pinion  — the pinion runs PINION_TOP_CLEAR under the
                     stator's bottom face.
- stator ↔ bowl    — each liner sector's flange rests on the skirt's
                     top edge of every bowl sector it overlaps (tangent
                     contact); nothing else in the drive reaches the
                     wall. The two rings' thirds seams are staggered
                     (asserted, SEAM_STAGGER_MIN_DEG).
- exit floor ↔ bowl — the tongue's rib rests tangent on the floor;
                     the tongue passes through the skirt's slot.
- exit floor ↔ rotor — the tongue's inner edge stays
                     EXIT_FLOOR_GEAR_CLEAR outside the ring gear's tip
                     circle (the pitch cylinder grown by ADDENDUM).

Deferred:
- exit_sensor ↔ stator exit arc (beam alignment)
"""


from functools import cache

from build123d import Axis

from cad_khana.mechanism.assembly import Assembly, RevoluteJoint

from ..design_systems import DEFAULT_SCHEMA, paint
from .design_systems import design_system
from .drive import assembly as _drive
from .params import (
    BOWL_SEAM_PHASE_DEG,
    LINER_SEAM_PHASE_DEG,
    ADDENDUM,
    MOTOR_AZIMUTH_DEG,
    BEARING_H,
    MESH_BACKLASH,
    PINION_TOP_CLEAR,
    ROTOR_STATOR_GAP,
)
from .rotor import assembly as _rotor
from .seams import SEAM_COUNT, angular_gap_deg, sector_index, seam_azimuths
from .stator import assembly as _stator


GAP_CLEARANCE_TOLERANCE = 0.5   # mm — catches a shrunk V-gap early

# Measurement tolerance for bounds constructed from the same constant
# as the geometry (cad/CLAUDE.md principle 6, corollary 2).
_MEAS_TOL = 0.01

# The bowl's seams stay at least this far round from the liner's, so
# no seam line runs through both rings (the stagger is what makes two
# parts worth having — `m02-chain.md` §5).
SEAM_STAGGER_MIN_DEG = 25.0


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
def build_m02(
    schema: str = DEFAULT_SCHEMA,
    motor_azimuth_deg: float = MOTOR_AZIMUTH_DEG,
    bowl_seam_phase_deg: float = BOWL_SEAM_PHASE_DEG,
) -> Assembly:
    """The singulator unit, painted under ``schema``.

    ``motor_azimuth_deg`` places the drive's motor / pinion around the
    rotor axis (default: away from the exit arc). The chain passes a
    per-stage value where a stage's gantry member would otherwise run
    into its motor; the unit's own claims (mesh, rests, clearances)
    hold at any azimuth. ``bowl_seam_phase_deg`` is the same kind of
    per-stage choice for the bowl's thirds seams (``seams.py``).

    Sub-assembly builders emit no colour; this root paints the finished
    tree once. ``paint`` also carries the design-system coverage check
    that used to be a module-level assert here: every leaf name must be
    tagged, and a missing one names all offenders at build time instead
    of crashing the import.
    """
    bowl_seams = seam_azimuths(bowl_seam_phase_deg)
    liner_seams = seam_azimuths(LINER_SEAM_PHASE_DEG)
    stagger = min(angular_gap_deg(b, l) for b in bowl_seams for l in liner_seams)
    a = (
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
        .with_subassembly("drive", _drive.build_drive(motor_azimuth_deg, bowl_seam_phase_deg))
        .assert_scalar(
            "seam_stagger_deg", stagger, ge=SEAM_STAGGER_MIN_DEG,
            detail="closest bowl seam to a liner seam, degrees round the axis",
        )
        # Gear mesh: pitch cylinders sit MESH_BACKLASH apart. Measured
        # against the whole rotor part — the flange OD is the closest
        # thing to the pinion.
        .assert_distance(
            "rotor.rotor", "drive.pinion.pinion",
            min_mm=MESH_BACKLASH - _MEAS_TOL, max_mm=MESH_BACKLASH + 0.1,
            name="gear_mesh_backlash",
        )
        # Rotor rests on the bearing, which is BEARING_H above the plate.
        .assert_tangent_contact("rotor.rotor", "drive.bearing")
        .assert_no_interference("rotor.rotor", "drive.motor")
        .assert_clearance("rotor.rotor", "drive.motor", min_mm=10.0)
        # Exit floor: inner edge outside the ring gear's tips (the cheap
        # rotor is the pitch cylinder — grow it by the addendum), rib
        # standing on the plate.
        .assert_no_interference("rotor.rotor", "stator.exit_floor")
        .assert_distance(
            "stator.exit_floor", "rotor.rotor",
            min_mm=_stator.EXIT_FLOOR_GEAR_CLEAR - _MEAS_TOL, grow_b_mm=ADDENDUM,
            name="exit_floor_clears_ring_gear_tips",
        )
        .assert_tangent_contact(
            "stator.exit_floor", f"drive.{_drive.bowl_part_at(0.0, bowl_seam_phase_deg)}",
        )
    )
    for part in _stator.STATOR_PARTS:
        a = (
            a
            # V-gap
            .assert_no_interference("rotor.rotor", f"stator.{part}")
            .assert_clearance(
                "rotor.rotor", f"stator.{part}",
                min_mm=ROTOR_STATOR_GAP - GAP_CLEARANCE_TOLERANCE,
            )
            # Pinion runs just under the stator's bottom face.
            .assert_clearance(
                f"stator.{part}", "drive.pinion.pinion", min_mm=PINION_TOP_CLEAR - _MEAS_TOL,
            )
        )
    for part in _drive.BOWL_PARTS:
        a = a.assert_clearance("rotor.rotor", f"drive.{part}", min_mm=BEARING_H - _MEAS_TOL)
    # Each liner sector's flange rests on the skirt's top edge of every
    # bowl sector it overlaps round the axis (its cone seats on the
    # edge's inner corner at the same z — a line contact, zero overlap).
    span = 360.0 / SEAM_COUNT
    for i, liner in enumerate(_stator.STATOR_PARTS):
        for j, bowl in enumerate(_drive.BOWL_PARTS):
            if _arc_overlap_deg(LINER_SEAM_PHASE_DEG + i * span, bowl_seam_phase_deg + j * span, span) > 0.0:
                a = a.assert_tangent_contact(f"stator.{liner}", f"drive.{bowl}")
    # Nothing in the drive reaches the stator subtree except at those
    # rests (a tangent pair carries no overlap; group form, declared
    # after composition).
    a = a.assert_no_interference_between("stator", "drive")
    return paint(a, design_system(schema))


def _arc_overlap_deg(a0: float, b0: float, span: float) -> float:
    """Angular overlap of the arcs [a0, a0+span] and [b0, b0+span]."""
    d = (b0 - a0) % 360.0
    return max(0.0, span - min(d, 360.0 - d))


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``. Imported as an object by ``m02_chain`` and
# the top level.
assembly = build_m02(DEFAULT_SCHEMA)
