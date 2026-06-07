"""Shared interface constants for the M05 diverter tower.

Top-level inputs and derived column-outer geometry that multiple
sub-assemblies (basket, ramp_mechanism) must agree on. Constants
internal to a single sub-assembly stay in that sub-assembly's file.
"""

import math

MAX_BRICK_DIM = 80.0            # largest LEGO dim this sorter handles (mm)
BRICK_SLOP = 5.0                # radial clearance brick-to-column-wall (mm)
N_BASKETS_PER_FLOOR = 6         # N of the ramp-facing basket (drives
                                # ramp_mechanism's BASKET_SLOT_W_AT_TIP
                                # calc). The floor itself is a mixed
                                # layout — see FLOOR_LAYOUT below.
R_BASKET_INNER = 130.0          # radial distance column axis → basket lip (mm).
                                # Constant arc radius: the basket's inner
                                # wall is a cylindrical arc at this radius
                                # at every azimuth, so the lip overhangs
                                # the ramp tip by `r_tip - R_BASKET_INNER`
                                # = 14.5 mm uniformly across all basket
                                # sizes (N=6 / 9 / 12 / 18). The earlier
                                # straight-chord inner wall let lip radius
                                # vary 129.9 (N=6) → 147.7 (N=18); N=18
                                # sat 1.3 mm *outboard* of the tip, and a
                                # near-vertical drop fell into the gap.
                                # 130 = R_HEX_INNER - EXT_SIZE/2 = hex
                                # inner-face apothem, so the basket floor
                                # fully covers the hex top everywhere
                                # along bisectors; it cantilevers up to
                                # 20.1 mm inboard at each vertex
                                # direction as a small flap (resolved by
                                # local bracket / floor-notch geometry).
R_HEX_OUTER = 270.0             # outer-hex apothem (center → edge midpoint, mm)
R_HEX_INNER = 140.0             # inner-hex apothem (mm). With both
                                # vertex brackets flipped below the
                                # extrusions, the bracket-arm ceiling is
                                # gone; the inner hex can move out until
                                # the inner-vertex bracket's radial arm
                                # (bolts at r = R_VERT_INNER + 32 and
                                # +52) runs off the end of the radial
                                # rail at r = RAIL_OUTER_R ≈ 290, or
                                # collides with the outer-vertex
                                # bracket's radial arm — real ceiling
                                # ≈ 180. Landed at 140 so the basket's
                                # arc inner wall at R_BASKET_INNER = 130
                                # lands exactly on the hex inner face
                                # at R_HEX_INNER - EXT_SIZE/2 = 130 along
                                # bisector azimuths (full 20 mm of hex
                                # top face sits under the basket floor
                                # along bisectors; at vertex directions
                                # the basket floor cantilevers ~20 mm
                                # inboard of the hex inner corner).
FLOOR_H = 220.0                 # vertical corner-column height spanning one
                                # floor (top face z=0 → base z=-FLOOR_H)

COL_WALL_T = 3.0
COL_INTERIOR_SIDE = MAX_BRICK_DIM + 2 * BRICK_SLOP   # 90
COL_OUTER_SIDE = COL_INTERIOR_SIDE + 2 * COL_WALL_T  # 96

# Column circular-collar OD. Circular features on the column (rotating
# collar at mid-height, top joint flange, base column's bottom collar)
# all share this OD — it circumscribes the square section's corners
# (96·√2 ≈ 135.8) with COLLAR_CLEARANCE radial overhang so a bearing
# rolls on a continuous circular surface. Lives here (not inside
# ramp_mechanism) because the base column's bottom collar is the drive
# ring-gear pitch surface, so the sibling `base_floor` sub-assembly
# needs it to size the pinion center distance — and sibling imports are
# forbidden, params is the only shared surface.
COLLAR_CLEARANCE = 1.0
COLLAR_OD = COL_OUTER_SIDE * math.sqrt(2) + 2 * COLLAR_CLEARANCE  # ≈ 138

# Hex circumradii (center → vertex). With Phase 1's rotation-aligned
# layout, inner and outer hex vertices share azimuth (k+0.5)·60° =
# basket boundaries; radial rails run along these directions and
# terminate at both vertex pairs.
_APOTHEM_TO_VERTEX = 2.0 / math.sqrt(3.0)
R_VERT_INNER = R_HEX_INNER * _APOTHEM_TO_VERTEX      # ≈ 161.66
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
# Each radial is split at the inner-hex vertex into two compression
# segments so the inner-hex through-column at R_VERT_INNER backs
# both adjacent rail ends. The inner-vertex bracket reserves
# BRACKET_HOR_OFFSET on each side along the rail axis, mirroring how
# the outer-vertex bracket reserves a stub on the outer side.
#
#                    spider          inner-vertex          outer-vertex
#  r:  90 ─────────  saddle ──────── bracket ─────────── bracket ─────── 289.77
#       └ radial_inner_k ─┘         ├─ radial_outer_k ───────────────────┘
#       (RAIL_INNER_R                (R_VERT_INNER +              (RAIL_OUTER_R)
#        → R_VERT_INNER -             BRACKET_HOR_OFFSET
#        BRACKET_HOR_OFFSET)
#
# Outer end of `radial_outer_k` reserves BRACKET_HOR_OFFSET for the
# outer-vertex bracket. Inner end of `radial_inner_k` sits at
# RAIL_INNER_R — far enough inside the inner-hex vertex that the
# spider saddle's grip zone
# ([RAIL_INNER_R, RAIL_INNER_R + SADDLE_GRIP_LEN] in the ramp
# sub-assembly) lands on this inner segment and stays well clear of
# the inner-vertex bracket's hub region. Each segment is in pure
# compression between its two anchors — no rail-as-beam under
# basket load.
#
# RAIL_INNER_R lower bound = ramp_mechanism's SPIDER_HUB_OD/2 (≈ 85.5
# mm for the current column geometry) + a small clearance. Hardcoded
# rather than derived because SPIDER_HUB_OD lives inside the ramp
# sub-assembly; a cross-sub-assembly assert in the top-level
# `assembly.py` catches drift if the spider hub grows past this.
RAIL_SIZE = 20.0
RAIL_INNER_R = 90.0                                            # physical inner end of inner segment
RAIL_INNER_OUTER_END_R = R_VERT_INNER - BRACKET_HOR_OFFSET     # ≈ 139.66 — inner segment butts here
RAIL_OUTER_INNER_END_R = R_VERT_INNER + BRACKET_HOR_OFFSET     # ≈ 183.66 — outer segment starts here
RAIL_OUTER_R = R_VERT_OUTER - BRACKET_HOR_OFFSET               # ≈ 289.77 — physical outer end


# Mixed floor layout. Each entry: (center_azimuth_deg, N_size, notch_set).
# `notch_set` ⊆ {"left", "right", "middle"}; LEFT/RIGHT refer to the
# basket's local frame (−/+ half-angle boundaries), MIDDLE to the
# basket's own center azimuth (used only where a column sits inside a
# basket's interior, i.e. N=9 MIDDLE baskets).
#
# Counterclockwise from the ramp (world +X = 0°):
#   S1    [330°, 30°]   1× N=6 in front of ramp, notches L + R
#   S2+3  [30°, 150°]   N=9 triple (L, M, R) spanning 2 hex sections
#   S4    [150°, 210°]  N=12 pair (L, R)
#   S5    [210°, 270°]  N=18 triple (L, NONE, R)
#   S6    [270°, 330°]  N=9 (L at V_4) + N=18 (R at V_5)
#
# Every outer-hex vertex V_k = (k+0.5)·60° lands on exactly the
# notches it needs; V_1 (90°) is the sole vertex inside a basket
# interior — absorbed by the N=9 MIDDLE notch on `basket_090_09`.
FLOOR_LAYOUT = [
    (  0.0,  6, {"left", "right"}),
    ( 50.0,  9, {"left"}),
    ( 90.0,  9, {"middle"}),
    (130.0,  9, {"right"}),
    (165.0, 12, {"left"}),
    (195.0, 12, {"right"}),
    (220.0, 18, {"left"}),
    (240.0, 18, set()),
    (260.0, 18, {"right"}),
    (290.0,  9, {"left"}),
    (320.0, 18, {"right"}),
]
