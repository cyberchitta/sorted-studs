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

# Hex circumradii (center → vertex). With Phase 1's rotation-aligned
# layout, inner and outer hex vertices share azimuth (k+0.5)·60° =
# basket boundaries; radial rails run along these directions and
# terminate at both vertex pairs.
_APOTHEM_TO_VERTEX = 2.0 / math.sqrt(3.0)
R_VERT_INNER = R_HEX_INNER * _APOTHEM_TO_VERTEX      # ≈ 136.26
R_VERT_OUTER = R_HEX_OUTER * _APOTHEM_TO_VERTEX      # ≈ 311.77

# Bracket-arm reserve: each vertex bracket occupies this many mm
# along every extrusion axis it terminates. Must match
# `HOR_INNER_OFFSET` in both `outer_vertex_bracket/assembly.py` and
# `inner_vertex_bracket/assembly.py` — duplicated here so params
# remains self-contained and bracket sub-assemblies stay standalone.
BRACKET_HOR_OFFSET = 22.0

# Radial rails tying the inner hex to the outer hex. 2020 aluminum
# extrusion. Axis at azimuth (k+0.5)·60° (basket boundary = shared
# inner/outer vertex direction). Top face at z=0 like every other
# horizontal frame member.
#
# Each radial runs as a single through-line from just outside the
# bearing-spider hub out to the outer-hex vertex. The outer end
# reserves BRACKET_HOR_OFFSET for the outer-vertex bracket. The
# inner end sits at RAIL_INNER_R — far enough inside the inner-hex
# vertex that the spider saddle's grip zone
# ([RAIL_INNER_R, RAIL_INNER_R + SADDLE_GRIP_LEN] in the ramp
# sub-assembly) stays clear of the inner-hex vertex region where
# inner_vertex_bracket sits. This also shrinks the spider's saddle
# cantilever dramatically, which was the point.
#
# RAIL_INNER_R lower bound = ramp_mechanism's SPIDER_HUB_OD/2 (≈ 85.5
# mm for the current column geometry) + a small clearance. Hardcoded
# rather than derived because SPIDER_HUB_OD lives inside the ramp
# sub-assembly; a cross-sub-assembly assert in the top-level
# `assembly.py` catches drift if the spider hub grows past this.
RAIL_SIZE = 20.0
RAIL_INNER_R = 90.0                                   # physical inner end
RAIL_OUTER_R = R_VERT_OUTER - BRACKET_HOR_OFFSET      # ≈ 289.77 — physical outer end
