"""Where the ramp aims for each basket, and which ramp a layout gets.

Pure functions over the basket and ramp geometry — no parts, no claims.
The column can stop at any azimuth, so the ramp need not face a
basket's centreline: a basket with a column channel on one boundary
has more free width on its other side, and the aim moves toward it.
`check_basket_landing.py` measures the real faces at the aim computed
here; this module is the prediction it tests.

The model, in a basket's local frame (centred on +X, boundaries at
±half-angle): a brick leaves the ramp anywhere across the exit width,
a strip centred on the aim line, from `RAMP_TIP_X` outward. The free
region is bounded by a line parallel to each boundary, offset into the
basket by that side's cavity face (`CAVITY_FACE_OFFSET_*`). The strip's
clearance to each face is least at the tip, so the tip is where the
aim balances the two.
"""

import math

from .basket.assembly import CAVITY_FACE_OFFSET_NOTCHED, CAVITY_FACE_OFFSET_PLAIN
from .ramp_mechanism.assembly import RAMP_EXIT_WIDTHS, RAMP_TIP_X

# Lateral scatter a falling brick may add beyond the exit width, each
# side. A claim, not a measurement: `backlog/bench-validation.md`.
LANDING_MARGIN = 2.5


def _face_offset(notched: bool) -> float:
    return CAVITY_FACE_OFFSET_NOTCHED if notched else CAVITY_FACE_OFFSET_PLAIN


def _clearances(half_deg: float, notches, aim_deg: float, exit_w: float):
    """Clearance from the exit strip's edges to the right (+) and left
    (−) cavity faces, at the tip, for a ramp aimed ``aim_deg`` off the
    basket's centreline."""
    h = math.radians(half_deg)
    a = math.radians(aim_deg)
    right = (RAMP_TIP_X * math.sin(h - a) - exit_w / 2 * math.cos(h - a)
             - _face_offset("right" in notches))
    left = (RAMP_TIP_X * math.sin(h + a) - exit_w / 2 * math.cos(h + a)
            - _face_offset("left" in notches))
    return right, left


def aim_offset_deg(n_size: int, notches, exit_w: float) -> float:
    """Aim, in degrees off the basket's centreline (+ toward its right
    boundary), that gives the exit strip equal clearance to both faces."""
    half = 180.0 / n_size
    lo, hi = -half, half
    for _ in range(60):
        mid = (lo + hi) / 2
        right, left = _clearances(half, notches, mid, exit_w)
        if right > left:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def landing_clearance(n_size: int, notches, exit_w: float) -> float:
    """Clearance to each face at the balanced aim."""
    aim = aim_offset_deg(n_size, notches, exit_w)
    return min(_clearances(180.0 / n_size, notches, aim, exit_w))


def ramp_exit_w_for(layout) -> float:
    """The widest ramp variant every basket in ``layout`` —
    ``(center_deg, n_size, notches)`` entries, see ``params`` — can
    take with ``LANDING_MARGIN`` each side."""
    for exit_w in RAMP_EXIT_WIDTHS:
        if all(
            landing_clearance(n, notches, exit_w) >= LANDING_MARGIN
            for _c, n, notches in layout
        ):
            return exit_w
    raise ValueError("no ramp variant lands in every basket of this layout")
