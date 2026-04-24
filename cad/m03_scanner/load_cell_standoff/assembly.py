"""
M03 scanner — load-cell standoff (rev 1, single design).

Printed rectangular column that carries one 500 g beam load cell's
fixed end from the enclosure floor. Three instances are placed in the
top-level scanner assembly (one per tripod leg); they share this single
geometry and differ only in their `Ry(heading)` placement rotation —
same pattern as `below_mirror_floor_bracket/` one directory up.

Coordinate frame (local to this sub-assembly):
    origin = centre of column bottom face, on the enclosure floor,
             directly under the load-cell fixed-end bolt pattern centre
    +Y     = world +Y (up)
    +X     = beam radial outward direction (horizontal)
    +Z     = beam tangential (horizontal; completes right-handed frame
             with +X, +Y)
    y = 0                : bottom face (floor contact)
    y = HEIGHT           : top face; beam bottom sits MOUNT_GAP above

Two bolt groups, both modelled as plain M4 through-cylinders along
local +Y (same rev 1 convention as every other bolted interface in the
scanner — heat-set insert pockets are a rev 2 follow-up):

    top bolts (2×)   — along local +X at ±FIXED_END_BOLT_SPACING/2,
                       matching the beam's fixed-end pattern.
    floor bolts (4×) — square on local XZ at
                       ±FLOOR_BOLT_SPACING/2 in each axis, matching the
                       (future) through-hole pattern in
                       `enclosure.wall_floor`.

Column cross-section (BASE_X × BASE_Z) is bounded from above by two
separate constraints and from below by the bolt-to-edge walls:

  * Upper bound in local +Z (beam tangential): clearance against the
    below-mirror floor bracket. The az=210 and az=330 standoffs sit
    at world (±190.5, -240, +110); the supported-side below-mirror
    bracket sits at world (±206.92, -240, +28.44) with heading
    ≈ -61.2° / +61.2°. In the bracket's own frame the standoff
    centre lands at bracket-local (X, Z) ≈ (+79.35, +24.98), and the
    standoff's local +Z maps onto bracket-local ≈ -X (projection
    magnitude ≈ 1.0). The standoff's bracket-+X extent is therefore
    controlled by BASE_Z/2; for a 2 mm margin against the bracket's
    +X face at bracket-local X=+60, BASE_Z/2 ≤ 17 → BASE_Z ≤ 34.
  * Upper bound in local +X: no explicit clearance constraint (the
    beam itself sits centred here), but we pick BASE_X = BASE_Z for a
    square column that prints with equal four-fold symmetry on the
    bed.
  * Lower bound in both axes: the floor bolts at
    ±FLOOR_BOLT_SPACING/2 must leave ≥ WALL_MIN from the bolt hole
    edge to the column face.

Print orientation: bottom face on the build plate, column rising along
+Y. All six outer faces are axis-aligned — no overhangs; the six
vertical bolt bores are 4.5 mm diameter, well inside PLA's axial-bore
bridging capability.

What's deferred to rev 2:
  * Heat-set M4 insert pockets on both the top and bottom faces
    (replaces the rev 1 through-holes so bolts can thread rather than
    relying on a nut on the opposite face).
  * Floor-panel through-hole pattern in `enclosure.wall_floor` matching
    the bottom-face pattern — bundled with the below-mirror bracket
    floor-bolt pattern and the back-upper back-wall-bolt pattern in a
    single future `wall_floor` / `wall_back` edit (see
    `../NEXT_STEPS.md`).
  * Weight-relief pocket / ribbed interior — the column prints solid
    at ≈ 232 mL PLA per instance today.
  * Load-cell stub bolt-hole cutouts in `platform/assembly.py` so the
    two top bolt axes appear as geometric features (today they live
    only in params).
"""

import math
import sys
from pathlib import Path

from build123d import Box, Cylinder, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

# Standalone-run import: params.py lives one directory up. Same
# pattern as the other m03 sub-assemblies.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    LOAD_CELL_FIXED_END_BOLT_SPACING,
    LOAD_CELL_STANDOFF_BOLT_HOLE_D,
    LOAD_CELL_STANDOFF_FLOOR_BOLT_SPACING,
    LOAD_CELL_STANDOFF_HEIGHT,
    WALL_MIN,
)


# --- Parameters ---------------------------------------------------------

HEIGHT = LOAD_CELL_STANDOFF_HEIGHT
TOP_BOLT_SPACING = LOAD_CELL_FIXED_END_BOLT_SPACING
FLOOR_BOLT_SPACING = LOAD_CELL_STANDOFF_FLOOR_BOLT_SPACING
BOLT_HOLE_D = LOAD_CELL_STANDOFF_BOLT_HOLE_D

# Column cross-section. Picked at the below-mirror-bracket clearance
# upper bound (see module docstring) with ≥ 2 mm margin, then locked
# symmetric so the column is 4-fold symmetric on the bed.
BASE_X = 34.0
BASE_Z = 34.0


# --- Parametric sanity checks -------------------------------------------

assert HEIGHT > 0, (
    f"standoff height {HEIGHT:.2f} mm is non-positive — did the load-cell "
    f"bottom drop below the enclosure floor?"
)

# Top bolt to column-edge walls.
_top_edge_x = BASE_X / 2 - TOP_BOLT_SPACING / 2 - BOLT_HOLE_D / 2
assert _top_edge_x >= WALL_MIN, (
    f"top bolt → column +X edge wall {_top_edge_x:.2f} mm < {WALL_MIN} mm "
    f"(BASE_X = {BASE_X}, TOP_BOLT_SPACING = {TOP_BOLT_SPACING})"
)

# Floor bolt to column-edge walls (square pattern → both +X and +Z
# symmetric; check the tighter axis).
_floor_edge_x = BASE_X / 2 - FLOOR_BOLT_SPACING / 2 - BOLT_HOLE_D / 2
_floor_edge_z = BASE_Z / 2 - FLOOR_BOLT_SPACING / 2 - BOLT_HOLE_D / 2
assert _floor_edge_x >= WALL_MIN, (
    f"floor bolt → column +X edge wall {_floor_edge_x:.2f} mm < {WALL_MIN} mm "
    f"(BASE_X = {BASE_X}, FLOOR_BOLT_SPACING = {FLOOR_BOLT_SPACING})"
)
assert _floor_edge_z >= WALL_MIN, (
    f"floor bolt → column +Z edge wall {_floor_edge_z:.2f} mm < {WALL_MIN} mm "
    f"(BASE_Z = {BASE_Z}, FLOOR_BOLT_SPACING = {FLOOR_BOLT_SPACING})"
)

# Inter-hole centre pitch (top vs. floor bolts). Worst case: top bolt
# at (±TOP_BOLT_SPACING/2, *, 0) vs. nearest floor bolt at
# (±FLOOR_BOLT_SPACING/2, *, ±FLOOR_BOLT_SPACING/2). With
# TOP_BOLT_SPACING < FLOOR_BOLT_SPACING the minimum centre distance is
# FLOOR_BOLT_SPACING/2 along +Z (a right triangle with one leg ≥
# |FLOOR_BOLT_SPACING/2 − TOP_BOLT_SPACING/2| along +X); bound the
# surface gap by that single-axis component for a conservative check.
_min_pitch = FLOOR_BOLT_SPACING / 2
assert _min_pitch >= BOLT_HOLE_D + WALL_MIN, (
    f"top-to-floor bolt centre pitch {_min_pitch:.2f} mm < "
    f"{BOLT_HOLE_D + WALL_MIN:.2f} mm (hole walls would merge)"
)


# --- Part function ------------------------------------------------------

def bracket(
    base_x: float = BASE_X,
    base_z: float = BASE_Z,
    height: float = HEIGHT,
    top_bolt_spacing: float = TOP_BOLT_SPACING,
    floor_bolt_spacing: float = FLOOR_BOLT_SPACING,
    bolt_hole_d: float = BOLT_HOLE_D,
) -> Part:
    """Rectangular column standing on +Y, with two bolt groups punched
    as vertical through-holes top-to-bottom:
      * 2× M4 on local +X at ±top_bolt_spacing/2 (load-cell side)
      * 4× M4 in a ±floor_bolt_spacing/2 square (floor side)

    All holes share a vertical axis (local +Y). Each is longer than
    the column so the boolean cut punches cleanly through both faces.
    """
    body = Pos(0, height / 2, 0) * Box(base_x, height, base_z)

    # Vertical through-hole: Rot(-90, 0, 0) takes the default +Z-axis
    # Cylinder onto +Y (Rx(-90)·(0,0,1) = (0,1,0)). Length = 2·height
    # so the cut extends well past both faces regardless of numerical
    # tolerance.
    cyl_len = 2 * height

    # Top (load-cell) bolts — along local +X, z = 0.
    for sx in (-1, 1):
        body -= (
            Pos(sx * top_bolt_spacing / 2, height / 2, 0)
            * Rot(-90, 0, 0)
            * Cylinder(bolt_hole_d / 2, cyl_len)
        )

    # Floor bolts — square in local XZ.
    for sx in (-1, 1):
        for sz in (-1, 1):
            body -= (
                Pos(
                    sx * floor_bolt_spacing / 2,
                    height / 2,
                    sz * floor_bolt_spacing / 2,
                )
                * Rot(-90, 0, 0)
                * Cylinder(bolt_hole_d / 2, cyl_len)
            )

    return body


# --- Assembly -----------------------------------------------------------

# Single-part sub-assembly — the meaningful clearance / interference
# checks live on the adjacent geometry (platform load cells, enclosure
# floor, below-mirror brackets) and so run in the scanner top-level
# `assembly.py`. Standalone, `check()` reports volume + bbox so a
# numerical sanity glance is possible without spinning up the rest of
# the rig.
assembly = Assembly().add("bracket", bracket())


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: +Y up (bottom face on build plate). All six
    # outer faces are axis-aligned and overhang-free; the six vertical
    # bolt bores print as clean axial holes at the default 45° FDM
    # overhang threshold.
    inspect(
        bracket(),
        method=FDM(
            up_axis=(0, 1, 0),
            wall_min_mm=WALL_MIN,
            overhang_max_deg=45.0,
        ),
        out="outputs",
        name="bracket",
    )
