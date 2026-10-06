"""Shared interface constants for the M05 diverter tower.

Top-level inputs and derived column-outer geometry that multiple
sub-assemblies (basket, ramp_mechanism) must agree on. Constants
internal to a single sub-assembly stay in that sub-assembly's file.
"""

import math

MAX_BRICK_DIM = 80.0            # largest LEGO dim this sorter handles (mm)
BRICK_SLOP = 5.0                # radial clearance brick-to-column-wall (mm)
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
                                # (62 long, out to R_VERT_INNER + 62)
                                # meets the outer-vertex bracket's
                                # (in to R_VERT_OUTER − 62) on the same
                                # radial rail: ceiling ≈ 162.6. The
                                # frame's bracket no-interference claim
                                # is green at 160 and red at 165
                                # (2026-09-28). Landed at 140 so the basket's
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

# Basket vertical envelope, floor-local. The rim plane is the sweep
# interface between basket and ramp_mechanism: the rotating ramp's
# lowest point must clear z = BASKET_RIM_Z at every azimuth (the
# radial side walls between baskets stand at rim height), and
# ramp_mechanism owns that assertion — so the datum lives here, on the
# shared params surface, not inside basket/.
BASKET_FLOOR_Z = 0.0            # shared floor datum
BASKET_DEPTH_Z = 130.0
BASKET_RIM_Z = BASKET_FLOOR_Z + BASKET_DEPTH_Z     # 130

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

# Base-floor turntable bearing (2026-09-11). The base floor carries the
# whole rotating stack's weight, so its bearing is a bought aluminium
# turntable ("lazy susan", nominal 8-inch class) — the same part m02's
# rim drive sits on (`cad/m02_c_channel_singulator/params.py` §Drive,
# `_notes/decisions/m02-rotor-drive.md`), so one bought bearing serves
# both modules. Stub dims — confirm against the part actually ordered;
# the seat and the base flange are parametric on these three numbers.
# Middle/top floors keep the printed radial-only ring bearing: they
# carry no axial load (the stack stands on the base).
TURNTABLE_OD = 200.0
TURNTABLE_ID = 165.0
TURNTABLE_H = 8.5
# Base-floor z-stack (rail-top datum z = 0), bottom → top: the base
# spider's hub is the turntable *seat*, top face at TURNTABLE_Z_BOT;
# the turntable sits on it; the base column's bottom flange sits on
# the turntable. Shared here because ramp_mechanism owns the seat,
# the flange and the column bottom while base_floor owns the ring
# gear that bolts to that flange's underside.
TURNTABLE_Z_BOT = 8.0             # seat top; must clear the spider saddle top plates (asserted in the spider)
TURNTABLE_Z_TOP = TURNTABLE_Z_BOT + TURNTABLE_H            # 16.5
BASE_COL_Z_BOT = TURNTABLE_Z_TOP                            # base column bottom / ring-gear standoff top
BASE_FLANGE_OD = TURNTABLE_OD + 4.0                         # 204 — covers the turntable's top ring

# Through-deck posts (2026-09-15, owner). The m02 singulator chain's
# gantry stands on two of the outer-hex through-columns *continued
# through the top deck* — no foot joint, no separate post: V_4
# (azimuth 270°, carries S3 + hopper and S2's T arm) and V_5 (330°,
# props S1). Keyed by outer-vertex index k (azimuth (k + 0.5)·60°),
# value = column height above the deck's TOP face. Every other m05
# part stays identical at all six vertices; what differs at these two
# is the column length, a notch in the deck, and a vertex bracket
# under the deck in the through form the middle floors use
# (`assembly.py` §Top deck). Each height is the propped stage's plate
# underside less the 2020 prop bar lying across the column's end
# (`cad/m02_chain/params.py:POST_H_ABOVE_DECK`: S1 V-bottom 1095 →
# 1095 − 24.5 − 20 − 890; S3 at 1384 likewise) — mirrored here so m05
# builds standalone, and asserted coincident with the chain's
# `post_top_k` belief anchors in `cad/assembly.py`, so drift fails
# there. Layout: `_notes/decisions/m02-chain.md` §6; sizing: the
# structure pass in `_notes/decisions/m02-chain.md` §6.1 (hand
# statics, unverified — it quoted the plate underside, 181 / 470).
THROUGH_DECK_POSTS: dict[int, float] = {
    4: 449.5,
    5: 160.5,
}

# Top deck OD — the clear-acrylic disc capping the tower, which the
# m03 turret floor frame and the m02 gantry's brace feet stand on. A
# knob (owner 2026-10-06), not derived: the deck must circumscribe the
# column tops it rests on (outer corners at r ≈ 322), and grows past
# that to carry whatever stands on it — m03 moves outward by
# `cad/assembly.py:M03_DUMP_OFFSET` for the top feed. The measure is
# the top level's `on_deck_*` claims: every deck-standing footprint
# inside the rim by `DECK_EDGE_MARGIN`; grow this until they are green.
# At an OD whose rim leaves less than a sheet thickness of acrylic
# outside a through-deck post's hole, the hole runs out through the rim
# as a notch (the 660 case, 2026-09-15); past that it is a closed hole
# (accepted, owner 2026-10-06). `build_m05(top_deck_od=…)` overrides it.
# 720 (owner 2026-10-06): the smallest multiple of 10 that clears the
# outriggers (r 351.07) by DECK_EDGE_MARGIN at M03_DUMP_OFFSET = 0, by
# arithmetic: 8.93 expected. At offset 40 the outriggers reach r 387.36
# and 790 is the smallest green (780 leaves 2.64). Mirrored by
# `m02_chain/params.py:M05_TOP_DECK_OD` for the chain's stand-in.
TOP_DECK_OD = 720.0

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
# RAIL_INNER_R has two lower bounds, both owned by sub-assemblies that
# cannot be imported here: ramp_mechanism's SPIDER_HUB_OD/2 (≈ 85.5 mm)
# + a small clearance, and — the binding one since 2026-09-11 — the
# base-floor pinion's tip circle (17 T, module 1.5 → r ≈ 96.2), which
# the tower root measures against the drive-azimuth rail as
# `pinion_tip_rail_gap`. 100 leaves that gap ≈ 3.8 mm. Hardcoded
# rather than derived; the tower-root asserts catch drift either way.
RAIL_SIZE = 20.0
RAIL_INNER_R = 100.0                                           # physical inner end of inner segment
RAIL_INNER_OUTER_END_R = R_VERT_INNER - BRACKET_HOR_OFFSET     # ≈ 139.66 — inner segment butts here
RAIL_OUTER_INNER_END_R = R_VERT_INNER + BRACKET_HOR_OFFSET     # ≈ 183.66 — outer segment starts here
RAIL_OUTER_R = R_VERT_OUTER - BRACKET_HOR_OFFSET               # ≈ 289.77 — physical outer end


# Basket layouts. A layout is a list of (center_azimuth_deg, N_size,
# notch_set) entries; `notch_set` ⊆ {"left", "right"}, LEFT/RIGHT in the
# basket's local frame (−/+ half-angle boundaries).
#
# A column may only ever sit on a basket *boundary*: a basket with a
# column inside it cannot be drawn out past the continuous through-
# columns (the N=9 once centred on V_1, dropped 2026-09-15). So a floor
# is six independent **sector fills**, one per 60° sector between
# vertices, sector k centred on 60·k (sector 0 faces the ramp's home at
# world +X). Each fill is its basket widths in degrees, counterclockwise,
# summing to 60 — the four sizes are 60 / 40 / 30 / 20 (N = 6 / 9 / 12 /
# 18), so the fills are (60,), (30, 30), (40, 20), (20, 40),
# (20, 20, 20). A sector's first basket carries the left notch and its
# last the right one; `layout_from_sectors` derives both, so any fill is
# valid by construction.
#
# Which fill goes where is not a design decision: it is set by how the
# collection is actually being sorted, and differs floor to floor. The
# four below are one illustration of that range, not a recommendation.
# Entries come out counterclockwise from sector 0 — the basket
# standalone checks each consecutive pair, wrap included, as neighbours.
_SECTOR_WIDTHS = {60: 6, 40: 9, 30: 12, 20: 18}


def layout_from_sectors(fills):
    """Six sector fills → a basket layout (see above)."""
    if len(fills) != 6:
        raise ValueError(f"need six sector fills, got {len(fills)}")
    layout = []
    for k, widths in enumerate(fills):
        if sum(widths) != 60 or any(w not in _SECTOR_WIDTHS for w in widths):
            raise ValueError(f"sector {k} fill {widths} is not a 60° split")
        edge = 60.0 * k - 30.0
        for i, w in enumerate(widths):
            notches = set()
            if i == 0:
                notches.add("left")
            if i == len(widths) - 1:
                notches.add("right")
            layout.append(((edge + w / 2.0) % 360.0, _SECTOR_WIDTHS[w], notches))
            edge += w
    return layout


# Per-floor layouts, keyed by floor index (0 = base). The base floor is
# the reference layout: the wide N=6 facing the ramp's home, the rest a
# mix. Each layout gets the widest ramp variant every one of its baskets
# can take (`aim.ramp_exit_w_for`), and a floor accepts only parts that
# pass its ramp: 30° baskets take the 6-wide ramp, and a 20° basket
# beside a vertex only the 3-wide one, so floor 1 is the small-parts
# floor and holds every 20° basket. A 40° basket pairs only with a 20°
# in a sector, so it lives on a 3-wide floor or not at all; only a
# floor of 60° baskets keeps the full channel (2026-09-28).
FLOOR_LAYOUTS = {
    0: layout_from_sectors([(60,), (30, 30), (60,), (30, 30), (30, 30), (60,)]),
    1: layout_from_sectors([(20, 20, 20)] * 6),
    2: layout_from_sectors([(60,), (30, 30), (60,), (30, 30), (60,), (30, 30)]),
    3: layout_from_sectors([(30, 30), (60,), (30, 30), (60,), (30, 30), (30, 30)]),
}

# The reference single-floor layout — `single_floor.py`, the basket
# standalone's first ring, and anything that checks one floor.
FLOOR_LAYOUT = FLOOR_LAYOUTS[0]


def basket_name(center_deg: float, n_size: int) -> str:
    """Leaf name of the basket a layout entry places (``basket_090_12``)
    — the one spelling every claim about it addresses."""
    return f"basket_{int(round(center_deg)):03d}_{n_size:02d}"


def all_basket_names() -> list[str]:
    """Every basket leaf name any floor layout uses, sorted — the set
    the paint and material maps must cover."""
    return sorted({
        basket_name(c, n)
        for layout in FLOOR_LAYOUTS.values()
        for c, n, _ in layout
    })
