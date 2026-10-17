"""Printability batch for the chain's printed bodies — run, don't import.

    .venv/bin/khana run cad/m02_chain/printability.py

Orchestration only: `assembly.py` declares the geometry and its
mechanism claims (`khana check cad/m02_chain/assembly.py`); this
script runs the per-part `inspect()` pass over the five printed joint
bodies from `joints.py`, each at the angles the chain actually places
it at, over every stage's three bowl sectors, each cut at that
stage's own seam phase and motor azimuth, and over the LOAD ramp in
its print orientation (on its side — see the call). The 2020 members
and the corner brackets are bought and are not inspected.

Each body comes out of the assembly by name (`Assembly.part`) rather
than from a re-typed constructor call. The two forms were byte-equal
on volume and bbox when this changed (2026-09-20), so nothing measured
moves — but the re-typed form was a mirror of nine `params` values, and
a drift in it would have silently inspected a body the chain does not
place.

The bowl sectors are inspected here, not in the drive's own
`printability.py`, because the chain is the only level that knows the
stages: the drive builds one bowl at whatever phase and azimuth it is
handed, and `build_m02`'s defaults are S1's. Before 2026-09-27 only
that default bowl was inspected, so S2's and S3's were never looked at.
"""

from cad_khana.printability.feature import Feature
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM
from cad_khana.printability.waiver import Waiver

from ..m02_c_channel_singulator.drive.assembly import (
    BOWL_PARTS, WALL_MIN, bowl_part_at, motor_cbore_skirt_foot,
)
from .assembly import build_chain
from .joints import brace_foot_wedge_edges
from .params import BRACE_COLS, BRACE_FOOT_STOP_H, S1, S3, STAGES, brace_run


# Every joint body is built in its print orientation: the seats
# socket-down (the socket's ceiling over the column's protruding
# corners is roofed at 40° from vertical), the brace collar on its
# flat shared bottom, the feet on their deck face. Nothing faces down
# above the bed, so the default 45° overhang threshold stands. The
# placement the chain gives each body is a translation onto its
# column; `PlacedPart.part` is the body itself, in that print
# orientation.
chain = build_chain()

# A vertical cut meeting a face parallel to the 41.9° brace leaves a
# 48.1° edge (90° − elevation) in each foot: the stop wall's top ridge,
# and the channel floor's lip at the block's end face. The crease rays
# read ~0.59 there (foot_3 at the ridge, foot_5 at the lip; each foot
# has both, and only the thinner shows). Alignment 0.65 ≈ cos 48.1° =
# 0.67 is the edge's own dihedral, not two near-parallel faces, and a
# chamfer on the ridge would take height off the face the brace bears
# on. Owner's call, 2026-09-22. The waiver is scoped to the two edges
# (`joints.brace_foot_wedge_edges`, built from each foot's own
# arguments), so a thin wall anywhere else on a foot traces to no
# feature and fails.
def _foot_features(col: int) -> dict[str, Feature]:
    az, el, _ = brace_run(col)
    return {"wedge_edges": Feature(
        brace_foot_wedge_edges(az + 180.0, el, BRACE_FOOT_STOP_H),
        waive={"wall_min": Waiver(
            reason="48.1° wedge tip, not a sliver: min_wall_at on the stop "
            "wall's top ridge (foot_3, 0.594) or the channel floor's end lip "
            "(foot_5, 0.582), alignment 0.65 ≈ cos 48.1°; the edge prints as "
            "an ordinary 48° corner",
            min_wall_mm=0.55,
        )},
    )}


for name, features in (
    (f"{S1.name}_post_seat", None),
    (f"{S3.name}_post_seat", None),
    ("brace_head", None),
    *((f"brace_foot_{col}", _foot_features(col)) for col in BRACE_COLS),
):
    inspect(
        chain.part(f"gantry.{name}").part,
        method=FDM(), out="outputs", name=name, features=features,
    )

# The bowl sectors print floor-down: the drive places each at identity
# in its own frame, so `PlacedPart.part` is the sector in print
# orientation (floor bottom on the bed at z −24.5). Skirt, slot and ear
# faces are vertical; on S1 and S2 the skirt's two free-end screw holes
# (`m02_c_channel_singulator/arc_fastening.py`) are teardrops, their
# roofs and countersinks facing down at 40° at most — under the 45°
# threshold, so they add no area. The only down-facing area past it is
# the skirt foot's first layer over the rims of the motor pattern's two
# outboard M3 counterbores (params NEMA17_BOLT_CBORE_D), so the
# overhang waiver belongs to each stage's motor sector alone, and each
# is witnessed on that stage's own body: the slivers' azimuths are
# m02-local, sagitta and chord read off the down-facing facets'
# vertices against the skirt's inner face at r 152. Owner accepted the
# waiver as written, 2026-09-17; that the first layer bridges the
# slivers is unsliced (`_notes/backlog/bench-validation.md` item 14).
#
# The waiver is scoped to `motor_cbore_skirt_foot`, the skirt ring
# intersected with the counterbore cutters `bowl()` cuts: its bottom
# face is exactly those two slivers, so a down-facing face anywhere
# else on the sector traces to no feature and fails.
#
# Keyed by (stage, motor azimuth, seam phase): the measurement covers
# that configuration only, so a changed stage parameter is a KeyError
# here — re-witness it, don't carry the text over.
_MOTOR_WAIVER = {
    ("s1", 180.0, 41.0): "0.83 mm² at 90° (s1, measured 2026-09-27 on "
        "this body): two slivers of 0.42 mm² each, the Ø6 counterbores' "
        "rims about the motor pattern's outboard corners (r 149.26, "
        "az 174° and 186°) lying past the skirt's inner face at r 152 — "
        "sagitta 0.26 mm, chord 2.45 mm each, at the floor top z −18.5. "
        "The skirt's first layer bridges them; nothing else in the part "
        "faces down.",
    ("s2", 210.0, 31.0): "0.83 mm² at 90° (s2, measured 2026-09-27 on "
        "this body): two slivers of 0.42 mm² each, the Ø6 counterbores' "
        "rims about the motor pattern's outboard corners (r 149.26, "
        "az 204° and 216°) lying past the skirt's inner face at r 152 — "
        "sagitta 0.26 mm, chord 2.45 mm each, at the floor top z −18.5. "
        "The skirt's first layer bridges them; nothing else in the part "
        "faces down.",
    ("s3", 210.0, 38.0): "0.83 mm² at 90° (s3, measured 2026-09-27 on "
        "this body): two slivers of 0.42 mm² each, the Ø6 counterbores' "
        "rims about the motor pattern's outboard corners (r 149.26, "
        "az 204° and 216°) lying past the skirt's inner face at r 152 — "
        "sagitta 0.26 mm, chord 2.45 mm each, at the floor top z −18.5. "
        "The skirt's first layer bridges them; nothing else in the part "
        "faces down.",
}

for s in STAGES:
    motor_sector = bowl_part_at(s.motor_azimuth_deg, s.bowl_seam_phase_deg)
    for leaf in BOWL_PARTS:
        inspect(
            chain.part(f"{s.name}.drive.{leaf}").part,
            method=FDM(wall_min_mm=WALL_MIN), out="outputs", name=f"{s.name}_{leaf}",
            features={"cbore_skirt_foot": Feature(
                motor_cbore_skirt_foot(motor_azimuth_deg=s.motor_azimuth_deg),
                waive={"overhang_max": Waiver(
                    reason=_MOTOR_WAIVER[(s.name, s.motor_azimuth_deg, s.bowl_seam_phase_deg)],
                    max_area_mm2=0.85, max_regions=2,
                )},
            )}
            if leaf == motor_sector else None,
        )

# The LOAD ramp prints on its side: the side wall's outer face (chain
# y 50, 2985 mm²) on the bed, so up is chain −Y. The chain places the
# ramp at identity (it is built in chain-local, cut by S1's stator
# cylinder), so `PlacedPart.part` is the body in the chain frame and
# `up_axis` is given in that frame. On its side nothing faces down
# (the JSON's `overhang` is null): the floor, both ends and the kink
# stand vertical, the stator cut and the wall's inner face face up —
# so the default 45° stands with no waiver. The brick-contact faces
# (slope and landing floor tops) stand vertical: perimeters, with
# layer lines along the slope's fall line rather than across the flow.
# Chosen 2026-09-27 from a sweep of ±X/±Y/±Z and the large faces'
# normals; every other candidate leaves ≥ 2574 mm² past 45° (the
# nearest, landing end down, is the slope's underside at 46.3°).
# The LOAD beam-break keeps it so (2026-10-17): the receiver tab is the
# floor carried on past its −Y edge, flush with the floor top, so in
# print it is the floor growing upward with nothing over air; the
# emitter's Ø3 hole runs through the bed-side wall along the up axis.
# A wall-with-a-hole tab was rejected here — see `params.py` §LOAD
# beam-break.
inspect(
    chain.part("load_ramp").part,
    method=FDM(up_axis=(0, -1, 0)), out="outputs", name="load_ramp",
)
