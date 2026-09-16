"""
m02 chain — the gantry's joint parts (pure part functions).

Cheap-tier bodies for the joints that hang the gantry on m05's two
through-deck columns, placed and claimed by `assembly.py`. Four kinds,
split by whether a standard 90° 2020 bracket fits the geometry:

- `corner_bracket` — a bought 2020 corner bracket, where two members
  meet square: S2's stem on V_4's +Y face (top and both sides — the
  bottom corner is where the brace collar sits), the stem's T onto
  its bar, and S3's cross halves onto the prop bar. Bought, not
  inspected.
- `post_seat` — printed. The prop bars cross their columns' ends
  askew (S1 on V_5 by ~42°, S3 on V_4 by ~13°: the bar must pass over
  the stage axis and the column's faces are fixed by the hex frame),
  so no corner bracket lands. A cap sockets over the column's top
  SEAT_SOCKET_H and cradles the bar in a channel at the bar's azimuth;
  the column's end face and the channel floor are one plane, so the
  bar still bears on the aluminium. The channel's side walls double
  as the saddle the structure pass wanted under the plate (a 20 mm
  line becomes SEAT_OD). Where the column's corners poke past the
  channel walls the socket's ceiling is roofed at SEAT_ROOF_TAPER
  from vertical, so printed socket-down nothing faces down.
- `brace_head` — printed. A collar on V_4 at the brace node with one
  U-channel per brace, each descending at the brace's angle, its
  inner end a vertical stop the brace's (vertical) end face bears
  on. The gusset under each channel runs down to one flat bottom
  shared with the collar, so printed bottom-down nothing faces down.
  The volume is the cheap tier's — hollowing is later.
- `brace_foot` — printed. The same U-channel-and-stop on the deck at
  V_3 / V_5, rising toward V_4; flat on the deck, nothing faces down.
  V_3's runs over its column axis with a flange behind; V_5's starts
  clear of the post and its vertex bracket (`params.py`).

Frames (all +Z up, chain axes):
    post_seat    origin = column axis at the column's END face; the
                 channel floor is z = 0; socket below, cradle above.
    corner_bracket  the corner line is local Y through the origin;
                 leg A lies on the z = 0 face along +X, leg B on the
                 x = 0 face along +Z. Place with
                 `Plane(origin=corner, x_dir=a, z_dir=b).location`.
    brace_head   origin = V_4 axis at the node height.
    brace_foot   origin = V_k axis at the deck top; the brace rises
                 along +azimuth.

The brace's end faces (`assembly.py:_brace`) are cut vertical at the
same stop distances these sockets use (`params.py:
BRACE_HEAD_STOP_H` / `BRACE_FOOT_STOP_H`); a socket's channel is the
brace's own prism beyond the stop, so the fit is exact.
"""

from __future__ import annotations

import math

from build123d import (
    Box, Cylinder, Keep, Part, Plane, Polygon, Pos, Rectangle, Rot, extrude,
)

from .params import BAR_DEPTH, EXT_SIZE


# --- Printed-joint parameters (internal to the joint family) -----------

WALL = 6.0                       # printed wall around a socket / channel
SEAT_OD = 56.0                   # post seat: round cap over the column
SEAT_SOCKET_H = 40.0             # column engagement below the end face
SEAT_ROOF_TAPER_DEG = 40.0       # from vertical: the socket-ceiling roof
SEAT_ROOF_H = 10.0               # < half-side / tan(taper), so it closes
BRACKET_LEG = 20.0               # 2020 corner bracket, cheap L
BRACKET_W = 17.0
BRACKET_T = 4.0
COLLAR_OD = EXT_SIZE + 2.0 * WALL                                  # 32
CHANNEL_W = EXT_SIZE + 2.0 * WALL                                  # 32
BRACE_ENGAGE = 30.0              # channel length past the stop, horizontal


# --- Parts ---------------------------------------------------------------

def post_seat(bar_az_deg: float, column_rot_deg: float) -> Part:
    """Cap over a column's end cradling a bar at `bar_az_deg`; the
    socket takes the column at `column_rot_deg`."""
    h = SEAT_SOCKET_H + BAR_DEPTH
    body = Pos(0.0, 0.0, BAR_DEPTH - h / 2.0) * Cylinder(SEAT_OD / 2.0, h)
    socket = Rot(0, 0, column_rot_deg) * (
        Pos(0.0, 0.0, -SEAT_SOCKET_H / 2.0 - 1.0)
        * Box(EXT_SIZE, EXT_SIZE, SEAT_SOCKET_H + 2.0)
    )
    roof = Rot(0, 0, column_rot_deg) * extrude(
        Rectangle(EXT_SIZE, EXT_SIZE), amount=SEAT_ROOF_H, taper=SEAT_ROOF_TAPER_DEG,
    )
    channel = Rot(0, 0, bar_az_deg) * (
        Pos(0.0, 0.0, BAR_DEPTH / 2.0 + 0.5) * Box(SEAT_OD + 2.0, EXT_SIZE, BAR_DEPTH + 1.0)
    )
    return body - socket - roof - channel


def corner_bracket() -> Part:
    """Bought 2020 corner bracket as a plain L: leg A on z = 0 along
    +X, leg B on x = 0 along +Z, BRACKET_W wide about the corner
    line (local Y)."""
    leg_a = Pos(BRACKET_LEG / 2.0, 0.0, BRACKET_T / 2.0) * Box(BRACKET_LEG, BRACKET_W, BRACKET_T)
    leg_b = Pos(BRACKET_T / 2.0, 0.0, BRACKET_LEG / 2.0) * Box(BRACKET_T, BRACKET_W, BRACKET_LEG)
    return leg_a + leg_b


def _half_diag(el_deg: float) -> float:
    """Vertical half-extent of a 20 square section on a member
    pitched `el_deg` from horizontal."""
    return (EXT_SIZE / 2.0) / math.cos(math.radians(el_deg))


def _brace_prism_beyond(az_deg: float, el_deg: float, stop_h: float, sign: float, length: float) -> Part:
    """The brace's own 20 × 20 prism along its axis from the origin,
    pitched `sign · el_deg` (−1 descends), kept only past the vertical
    plane `stop_h` out along the azimuth — the socket's channel."""
    prism = (
        Rot(0, 0, az_deg) * Rot(0, -sign * el_deg, 0)
        * Pos(length / 2.0, 0.0, 0.0) * Box(length, EXT_SIZE, EXT_SIZE)
    )
    c, s = math.cos(math.radians(az_deg)), math.sin(math.radians(az_deg))
    stop = Plane(origin=(stop_h * c, stop_h * s, 0.0), z_dir=(c, s, 0.0))
    return prism.split(stop, keep=Keep.TOP)


def _vertical_strip(az_deg: float, pts_hz: list[tuple[float, float]]) -> Part:
    """Extrude an (h, z) profile — h out along the azimuth, z up —
    symmetric CHANNEL_W across the azimuth line."""
    c, s = math.cos(math.radians(az_deg)), math.sin(math.radians(az_deg))
    pl = Plane(origin=(0.0, 0.0, 0.0), x_dir=(c, s, 0.0), z_dir=(s, -c, 0.0))
    return pl.location * extrude(Polygon(*pts_hz, align=None), amount=CHANNEL_W / 2.0, both=True)


def brace_head(
    braces: tuple[tuple[float, float], ...],
    stop_h: float,
    column_rot_deg: float = 0.0,
) -> Part:
    """Collar on the post at the brace node with one channel per
    `(azimuth, descent)` in `braces`, each stopped `stop_h` out.
    Origin at the post axis, node height."""
    s_out = stop_h + BRACE_ENGAGE
    # Lowest channel floor sets the shared flat bottom; the brace's
    # top edge at the column sets the collar's top.
    hd_max = max(_half_diag(el) for _, el in braces)
    z_bottom = min(-s_out * math.tan(math.radians(el)) - _half_diag(el) for _, el in braces) - WALL
    z_top = hd_max + WALL
    collar = Pos(0.0, 0.0, (z_top + z_bottom) / 2.0) * Box(COLLAR_OD, COLLAR_OD, z_top - z_bottom)
    part = collar
    for az, el in braces:
        t, hd = math.tan(math.radians(el)), _half_diag(el)
        gusset = _vertical_strip(az, [
            (0.0, hd), (s_out, -s_out * t + hd), (s_out, z_bottom), (0.0, z_bottom),
        ])
        part = part + gusset
    for az, el in braces:
        part = part - _brace_prism_beyond(az, el, stop_h, -1.0, 4.0 * s_out)
    column = Rot(0, 0, column_rot_deg) * Box(EXT_SIZE, EXT_SIZE, 4.0 * (z_top - z_bottom))
    return part - column


def brace_foot(
    az_deg: float, el_deg: float, stop_h: float, start_h: float, flange: bool,
) -> Part:
    """Block on the deck from `start_h` out along the azimuth with a
    channel for a brace rising at `el_deg`, stopped `stop_h` out.
    Origin at the column axis on the deck top. `flange` adds a plate
    behind the block over the axis (V_3, where nothing stands on the
    deck)."""
    s_out = stop_h + BRACE_ENGAGE
    t, hd = math.tan(math.radians(el_deg)), _half_diag(el_deg)
    block = _vertical_strip(az_deg, [
        (start_h, 0.0), (s_out, 0.0), (s_out, s_out * t + hd), (start_h, start_h * t + hd),
    ])
    if flange:
        block = block + _vertical_strip(az_deg, [
            (-COLLAR_OD / 2.0, 0.0), (start_h + 1.0, 0.0), (start_h + 1.0, WALL), (-COLLAR_OD / 2.0, WALL),
        ])
    return block - _brace_prism_beyond(az_deg, el_deg, stop_h, +1.0, 4.0 * s_out)
