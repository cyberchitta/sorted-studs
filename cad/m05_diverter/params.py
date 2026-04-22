"""Shared interface constants for the M05 diverter tower.

Top-level inputs and derived column-outer geometry that multiple
sub-assemblies (basket, ramp_mechanism) must agree on. Constants
internal to a single sub-assembly stay in that sub-assembly's file.
"""

import math

MAX_BRICK_DIM = 80.0            # largest LEGO dim this sorter handles (mm)
BRICK_SLOP = 5.0                # radial clearance brick-to-column-wall (mm)
N_BASKETS_PER_FLOOR = 6         # 6 / 12 / 18; affects ramp-tip targeting
R_BASKET_INNER = 150.0          # radial distance column axis → basket lip (mm)
R_HEX_OUTER = 270.0             # outer-hex apothem (center → edge midpoint, mm)
R_HEX_INNER = 118.0             # inner-hex apothem (mm); inner-hex edge lies
                                # inside basket inner face (R_BASKET_INNER
                                # *cos(30°)≈129.9 mm) with ≈1.9 mm clearance
FLOOR_H = 220.0                 # vertical corner-column height spanning one
                                # floor (top face z=0 → base z=-FLOOR_H)

COL_WALL_T = 3.0
COL_INTERIOR_SIDE = MAX_BRICK_DIM + 2 * BRICK_SLOP   # 90
COL_OUTER_SIDE = COL_INTERIOR_SIDE + 2 * COL_WALL_T  # 96

# Inner-hex circumradius (center → vertex). Vertices at basket
# boundaries = (k+0.5)·60°; radial rails run along these directions.
_APOTHEM_TO_VERTEX = 2.0 / math.sqrt(3.0)
R_VERT_INNER = R_HEX_INNER * _APOTHEM_TO_VERTEX      # ≈ 136.26

# Radial rails tying the inner hex to the outer hex. 2020 aluminum
# extrusion. Axis at azimuth (k+0.5)·60° (basket boundary = inner-hex
# vertex direction = outer-hex edge-midpoint direction). Top face at
# z=0 like every other horizontal frame member. The spider's saddles
# grip the inner end of each rail; the outer end needs a mid-edge
# bracket on the outer hex (not yet modeled).
RAIL_SIZE = 20.0
RAIL_INNER_R = R_VERT_INNER
RAIL_OUTER_R = R_HEX_OUTER
