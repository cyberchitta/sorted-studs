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
- liner free ends ↔ bowl — on a plain-wall stage the exit opening
                     splits the liner sector it falls in; each free
                     end's boss bears on the skirt's inner face
                     (measured patch area) and a countersunk screw
                     (``arc_screw_pos`` / ``_neg``, placed here) seats
                     in the skirt (tangent contact). ``_with_arc_screws``
                     lists the claims; driver access is
                     held in ``cad/assembly.py`` (keep-outs against
                     the whole machine).
- exit floor ↔ bowl — the tongue's rib rests tangent on the floor;
                     the tongue passes through the skirt's slot.
- exit floor ↔ rotor — the tongue's inner edge stays
                     EXIT_FLOOR_GEAR_CLEAR outside the ring gear's tip
                     circle (the pitch cylinder grown by ADDENDUM).

Deferred:
- exit_sensor ↔ stator exit arc (beam alignment)
"""


import math
from functools import cache

from build123d import Axis

from cad_khana.mechanism.assembly import Assembly, RevoluteJoint

from ..design_systems import DEFAULT_SCHEMA, paint
from .design_systems import color_for, design_system
from . import arc_fastening as _fastening
from .drive import assembly as _drive
from .params import (
    BOWL_SEAM_PHASE_DEG,
    LINER_SEAM_PHASE_DEG,
    ADDENDUM,
    MOTOR_AZIMUTH_DEG,
    R_STATOR_OUTER_KINK,
    BEARING_H,
    MESH_BACKLASH,
    PINION_TOP_CLEAR,
    ARC_SCREW_IN_HOLE_MIN,
    EXIT_FLOOR_SIDE_CLEAR,
    ROTOR_STATOR_GAP,
    STATOR_WALL_H,
)
from .materials import material_of
from .rotor import assembly as _rotor
from .seams import SEAM_COUNT, SEAM_EAR_T, angular_gap_deg, angular_gap_to_span_deg, sector_index, seam_azimuths
from .stator import assembly as _stator


GAP_CLEARANCE_TOLERANCE = 0.5   # mm — catches a shrunk V-gap early

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
    wall_h: float = STATOR_WALL_H,
) -> Assembly:
    """The singulator unit, painted under ``schema``.

    ``motor_azimuth_deg`` places the drive's motor / pinion around the
    rotor axis (default: away from the exit arc). The chain passes a
    per-stage value where a stage's gantry member would otherwise run
    into its motor; the unit's own claims (mesh, rests, clearances)
    hold at any azimuth. ``bowl_seam_phase_deg`` is the same kind of
    per-stage choice for the bowl's thirds seams (``seams.py``), and
    ``wall_h`` the liner's height — HOPPER_WALL_H on the chain's top
    stage, STATOR_WALL_H elsewhere.

    Sub-assembly builders emit no colour; this root paints the finished
    tree once. ``paint`` also carries the design-system coverage check
    that used to be a module-level assert here: every leaf name must be
    tagged, and a missing one names all offenders at build time instead
    of crashing the import.
    """
    bowl_seams = seam_azimuths(bowl_seam_phase_deg)
    liner_seams = seam_azimuths(LINER_SEAM_PHASE_DEG)
    stagger = min(angular_gap_deg(b, l) for b in bowl_seams for l in liner_seams)
    # Where the exit opening cuts the liner through, its two arcs are
    # fastened at their free ends into the skirt (`arc_fastening.py`).
    split = _stator.exit_splits_wall(wall_h)
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
        .with_subassembly("stator", _stator.build_stator(wall_h))
        .with_subassembly("drive", _drive.build_drive(motor_azimuth_deg, bowl_seam_phase_deg, split))
        .assert_scalar(
            "seam_stagger_deg", stagger, ge=SEAM_STAGGER_MIN_DEG,
            detail="closest bowl seam to a liner seam, degrees round the axis",
        )
        # Gear mesh: pitch cylinders sit MESH_BACKLASH apart. Measured
        # against the whole rotor part — the flange OD is the closest
        # thing to the pinion.
        .assert_distance(
            "rotor.rotor", "drive.pinion.pinion",
            min_mm=MESH_BACKLASH, max_mm=MESH_BACKLASH + 0.1,
            name="gear_mesh_backlash",
        )
        # Rotor rests on the bearing, which is BEARING_H above the plate.
        .assert_tangent_contact("rotor.rotor", "drive.bearing")
        .assert_no_interference("rotor.rotor", "drive.motor")
        .assert_distance("rotor.rotor", "drive.motor", min_mm=10.0)
        # Exit floor: inner edge outside the ring gear's tips (the cheap
        # rotor is the pitch cylinder — grow it by the addendum), rib
        # standing on the plate.
        .assert_no_interference("rotor.rotor", "stator.exit_floor")
        .assert_distance(
            "stator.exit_floor", "rotor.rotor",
            min_mm=_stator.EXIT_FLOOR_GEAR_CLEAR, grow_b_mm=ADDENDUM,
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
            .assert_distance(
                "rotor.rotor", f"stator.{part}",
                min_mm=ROTOR_STATOR_GAP - GAP_CLEARANCE_TOLERANCE,
            )
            # Pinion runs just under the stator's bottom face.
            .assert_distance(
                f"stator.{part}", "drive.pinion.pinion", min_mm=PINION_TOP_CLEAR,
            )
        )
    for part in _drive.BOWL_PARTS:
        a = a.assert_distance("rotor.rotor", f"drive.{part}", min_mm=BEARING_H)
    # Each liner sector's flange rests on the skirt's top edge of every
    # bowl sector it overlaps round the axis (its cone seats on the
    # edge's inner corner at the same z — a line contact, zero overlap).
    span = 360.0 / SEAM_COUNT
    for i, liner in enumerate(_stator.STATOR_PARTS):
        for j, bowl in enumerate(_drive.BOWL_PARTS):
            if _arc_overlap_deg(LINER_SEAM_PHASE_DEG + i * span, bowl_seam_phase_deg + j * span, span) > 0.0:
                a = a.assert_tangent_contact(f"stator.{liner}", f"drive.{bowl}")
    if split:
        a = _with_arc_screws(a, bowl_seam_phase_deg)
    # Nothing in the drive reaches the stator subtree except at those
    # rests (a tangent pair carries no overlap; group form, declared
    # after composition).
    a = a.assert_no_interference_between("stator", "drive")
    return paint(a, design_system(schema), color_for=color_for)


def _with_arc_screws(a: Assembly, bowl_seam_phase_deg: float) -> Assembly:
    """The free-end screws and their claims, for a unit whose liner the
    exit opening cuts through. Each screw is bought hardware joining two
    subtrees (liner and bowl), so it lives here at the root.

    What each claim stands for:
      - the head seats in the skirt's countersink (tangent contact with
        the bowl sector it passes through) — the screw's bearing face;
      - the boss bears on the skirt's inner face: the shared patch's
        area (``bearing_area_mm2``), not a tangent contact, which any
        touching face would satisfy — the liner already touches every
        bowl sector it overlaps at the flange;
      - the shank runs in the insert hole without touching it:
        ARC_SCREW_IN_HOLE_MIN, a floor of its own — bounded by the
        hole's and screw's diameters it would move with them and never
        go red (it did not, on a Ø3.5 hole, until it was changed);
      - the screw keeps out of the exit opening's volume, off the
        tongue and clear of the rotor (the rotor claim moves in
        ``check_rotor_turn.py``);
      - the fastening stays off the bowl's seams by an ear's width, so
        it lands on one bowl sector, clear of the ear pair.

    Each screw's driver corridor is declared here as a named keep-out
    (``arc_screw_{end}_driver``) and held in ``cad/assembly.py``: the
    corridor reaches past the unit, into bodies only the machine has.
    """
    exit_liner = f"stator.{_stator.STATOR_PARTS[sector_index(0.0, LINER_SEAM_PHASE_DEG)]}"
    corridor = _fastening.exit_corridor()
    ear_deg = math.degrees(2.0 * SEAM_EAR_T / R_STATOR_OUTER_KINK)
    seam_gap = min(
        angular_gap_to_span_deg(seam, *_fastening.boss_span_deg(sign))
        for seam in seam_azimuths(bowl_seam_phase_deg)
        for sign in _fastening.FREE_END_SIGNS
    )
    a = a.assert_scalar(
        "arc_fastening_to_bowl_seam_deg", seam_gap, ge=ear_deg,
        detail="closest bowl seam to either free-end boss, degrees round the axis (floor: an ear pair's width)",
    )
    for sign in _fastening.FREE_END_SIGNS:
        end = _fastening.free_end_name(sign)
        screw = f"arc_screw_{end}"
        loc = _fastening.arc_screw_location(sign)
        bowl = f"drive.{_drive.bowl_part_at(_fastening.arc_screw_azimuth_deg(sign), bowl_seam_phase_deg)}"
        a = (
            a
            .with_part(screw, _fastening.arc_screw(), location=loc, material=material_of(screw))
            .assert_tangent_contact(screw, bowl)
            .assert_scalar(
                f"arc_boss_{end}_bearing_area_mm2",
                _fastening.bearing_area_mm2(a.part(exit_liner).part, a.part(bowl).part, sign),
                ge=_fastening.ARC_BOSS_BEARING_MIN_MM2,
                detail="area the free end's boss shares with the skirt's inner face, holes out",
            )
            .assert_no_interference(screw, exit_liner)
            .assert_distance(screw, exit_liner, min_mm=ARC_SCREW_IN_HOLE_MIN)
            .assert_distance(screw, "stator.exit_floor", min_mm=EXIT_FLOOR_SIDE_CLEAR)
            .assert_distance("rotor.rotor", screw, min_mm=ROTOR_STATOR_GAP)
            .assert_clear_of(screw, corridor, name=f"arc_screw_{end}_exit", min_mm=EXIT_FLOOR_SIDE_CLEAR)
            .with_keepout(f"arc_screw_{end}_driver", _fastening.driver_corridor(sign))
        )
    return a


def _arc_overlap_deg(a0: float, b0: float, span: float) -> float:
    """Angular overlap of the arcs [a0, a0+span] and [b0, b0+span]."""
    d = (b0 - a0) % 360.0
    return max(0.0, span - min(d, 360.0 - d))


# Degenerate memoized master, so ``khana check <file>`` resolves
# without a ``:factory``. Imported as an object by ``m02_chain`` and
# the top level.
assembly = build_m02(DEFAULT_SCHEMA)
