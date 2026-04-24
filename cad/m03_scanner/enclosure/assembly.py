"""
M03 scanner — enclosure sub-assembly (rev 1, plain six-panel box).

First-rev envelope: six rectangular printed panels (floor, ceiling,
back, front, left, right) bounding the scanner's interior volume.
Panels are modelled as solid rectangular prisms with no mount-interface
holes, edge-joint hardware, chute aperture, access door, or ring-light
rim — all of those are rev 2+ concerns (see `../NEXT_STEPS.md`).

The goal at rev 1 is narrow: establish the world-frame interior volume,
let the top-level `cad/m03_scanner/assembly.py` drop every existing
sub-assembly into that volume, and run cross-boundary interference
checks. Once that passes, mount interfaces + cut-outs can be added
incrementally knowing the base envelope clears every internal part.

Coordinate frame = world frame of the scanner:
    origin = brick centroid on the platform top face
    +X     = operator's right
    +Y     = up
    +Z     = away from operator (toward the back of the machine)

Interior bounds and wall thickness come from
`cad/m03_scanner/params.py`; matching limits derive here.

Panels are named for the world-frame face they occupy:
    wall_floor     — y = Y_MIN (under the platform + load cells)
    wall_ceiling   — y = Y_MAX (over the back-upper mirror + ToF zone)
    wall_back      — z = Z_MAX (behind the back-upper mirror + chute)
    wall_front     — z = Z_MIN (behind the camera bracket)
    wall_left      — x = X_MIN (outboard of the below-left mirror)
    wall_right     — x = X_MAX (outboard of the below-right mirror)

What's deferred to rev 2 (documented in `../NEXT_STEPS.md`):
  * Camera bracket mount interface on the front wall — the bracket is
    tilted 25°, so a flat front-wall flange doesn't match it cleanly;
    likely a tilted internal boss or a tilted front panel.
  * Mirror-mount floor/wall brackets — the three mirrors are tilted
    and don't sit flush against any axis-aligned wall; each needs an
    intermediate support bracket.
  * Platform load-cell floor mounts — M4 heat-set pockets on the
    floor panel under each load-cell fixed end.
  * Chute entry aperture on the back wall — pending Module 2 exit
    geometry.
  * Access door on the front/top wall + hinge + latch — pending a
    service-access walk-through.
  * Ring-light mount rim around the platform collar OD — pending LED
    ring specification.
  * ToF sensor ceiling mount.
  * Edge-joint hardware — how the six panels bolt to each other
    (M4 through-holes + heat-set inserts in the abutting edges, or
    printed corner brackets).
"""

from build123d import Box, Cylinder, Part, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

# Standalone-run import: params lives one directory up. See
# `../../m05_diverter/assembly.py` for the same pattern.
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    ADAPTER_WALL_BOLT_HOLE_D,
    ADAPTER_WALL_BOLT_SPACING,
    CAMERA_BRACKET_BACK_CENTRE,
    ENCLOSURE_INNER_X_MAX,
    ENCLOSURE_INNER_X_MIN,
    ENCLOSURE_INNER_Y_MAX,
    ENCLOSURE_INNER_Y_MIN,
    ENCLOSURE_INNER_Z_MAX,
    ENCLOSURE_INNER_Z_MIN,
    ENCLOSURE_WALL_T,
    WALL_MIN,
)


# --- Derived outer bounds -----------------------------------------------

_T = ENCLOSURE_WALL_T
X_OUTER_MIN = ENCLOSURE_INNER_X_MIN - _T
X_OUTER_MAX = ENCLOSURE_INNER_X_MAX + _T
Y_OUTER_MIN = ENCLOSURE_INNER_Y_MIN - _T
Y_OUTER_MAX = ENCLOSURE_INNER_Y_MAX + _T
Z_OUTER_MIN = ENCLOSURE_INNER_Z_MIN - _T
Z_OUTER_MAX = ENCLOSURE_INNER_Z_MAX + _T

# Interior width / height / depth — reported for quick orientation.
_INTERIOR_W = ENCLOSURE_INNER_X_MAX - ENCLOSURE_INNER_X_MIN
_INTERIOR_H = ENCLOSURE_INNER_Y_MAX - ENCLOSURE_INNER_Y_MIN
_INTERIOR_D = ENCLOSURE_INNER_Z_MAX - ENCLOSURE_INNER_Z_MIN


# --- Parametric sanity checks -------------------------------------------

assert _T >= WALL_MIN, (
    f"panel thickness {_T:.2f} mm < FDM wall minimum {WALL_MIN} mm"
)

assert _INTERIOR_W > 0 and _INTERIOR_H > 0 and _INTERIOR_D > 0, (
    "enclosure interior bounds invalid"
)


# --- Panel construction -------------------------------------------------
#
# Each panel is an axis-aligned rectangular slab, nested outward-to-
# inward to keep adjacent panels disjoint while still forming a closed
# box:
#
#   back / front    — outermost: full outer X × outer Y footprint;
#                     the corners at the four outer edges are owned
#                     by these two panels
#   floor / ceiling — middle: full outer X × *interior* Z footprint
#                     (nested between back/front); shares the X edges
#                     with back/front but not volume
#   left  / right   — innermost: interior Y × interior Z footprint
#                     (nested between floor/ceiling and between
#                     back/front)
#
# Real edge hardware (bolts, heat-set inserts, corner brackets) is a
# rev 2 concern; rev 1 only enforces "panels don't share volume".

def _panel_x_z(y_center: float) -> Part:
    """Horizontal panel (floor or ceiling). Spans the full outer X and
    the interior Z (nested between the back and front panels);
    thickness along Y centred at `y_center`."""
    w = X_OUTER_MAX - X_OUTER_MIN
    d = ENCLOSURE_INNER_Z_MAX - ENCLOSURE_INNER_Z_MIN
    cx = (X_OUTER_MIN + X_OUTER_MAX) / 2
    cz = (ENCLOSURE_INNER_Z_MIN + ENCLOSURE_INNER_Z_MAX) / 2
    return Pos(cx, y_center, cz) * Box(w, _T, d)


def _panel_x_y(z_center: float) -> Part:
    """Vertical panel facing along Z (back or front). Spans the full
    outer X×Y footprint (owns the outer corners); thickness along Z
    centred at `z_center`."""
    w = X_OUTER_MAX - X_OUTER_MIN
    h = Y_OUTER_MAX - Y_OUTER_MIN
    cx = (X_OUTER_MIN + X_OUTER_MAX) / 2
    cy = (Y_OUTER_MIN + Y_OUTER_MAX) / 2
    return Pos(cx, cy, z_center) * Box(w, h, _T)


def _panel_y_z(x_center: float) -> Part:
    """Vertical panel facing along X (left or right). Spans the
    interior Y × interior Z footprint (nested between floor/ceiling
    and between back/front); thickness along X centred at `x_center`."""
    h = ENCLOSURE_INNER_Y_MAX - ENCLOSURE_INNER_Y_MIN
    d = ENCLOSURE_INNER_Z_MAX - ENCLOSURE_INNER_Z_MIN
    cy = (ENCLOSURE_INNER_Y_MIN + ENCLOSURE_INNER_Y_MAX) / 2
    cz = (ENCLOSURE_INNER_Z_MIN + ENCLOSURE_INNER_Z_MAX) / 2
    return Pos(x_center, cy, cz) * Box(_T, h, d)


def wall_floor() -> Part:
    """Bottom panel, face at y = Y_MIN (inner floor) / Y_OUTER_MIN."""
    return _panel_x_z((ENCLOSURE_INNER_Y_MIN + Y_OUTER_MIN) / 2)


def wall_ceiling() -> Part:
    """Top panel, face at y = Y_MAX (inner ceiling) / Y_OUTER_MAX."""
    return _panel_x_z((ENCLOSURE_INNER_Y_MAX + Y_OUTER_MAX) / 2)


def wall_back() -> Part:
    """Back panel, face at z = Z_MAX (inner back) / Z_OUTER_MAX."""
    return _panel_x_y((ENCLOSURE_INNER_Z_MAX + Z_OUTER_MAX) / 2)


def wall_front() -> Part:
    """Front panel, face at z = Z_MIN (inner front) / Z_OUTER_MIN.

    Carries the adapter wall-bolt through-hole pattern: 4× M4 bolt
    clearance holes on a square of side `ADAPTER_WALL_BOLT_SPACING`,
    centred on (0, CAMERA_BRACKET_BACK_CENTRE.y) in world. Bolts drive
    from outside the enclosure (world -Z side) through the panel and
    into the adapter's matching holes on the back face. The 0.3 mm
    radial slop between the 4.5 mm clearance hole and the M4 bolt
    absorbs any sub-millimetre print-dimension drift between the two
    parts."""
    z_center = (ENCLOSURE_INNER_Z_MIN + Z_OUTER_MIN) / 2
    panel = _panel_x_y(z_center)

    # Adapter wall-bolt through-holes. Cylinder length spans the full
    # panel thickness with margin on both sides so the boolean cut
    # lands cleanly regardless of the panel's precise Z extent.
    cyl_len = _T * 4
    half = ADAPTER_WALL_BOLT_SPACING / 2
    y_center = CAMERA_BRACKET_BACK_CENTRE[1]
    for sx in (-1, 1):
        for sy in (-1, 1):
            panel -= (
                Pos(sx * half, y_center + sy * half, z_center)
                * Cylinder(ADAPTER_WALL_BOLT_HOLE_D / 2, cyl_len)
            )
    return panel


def wall_left() -> Part:
    """Left panel, face at x = X_MIN (inner left) / X_OUTER_MIN."""
    return _panel_y_z((ENCLOSURE_INNER_X_MIN + X_OUTER_MIN) / 2)


def wall_right() -> Part:
    """Right panel, face at x = X_MAX (inner right) / X_OUTER_MAX."""
    return _panel_y_z((ENCLOSURE_INNER_X_MAX + X_OUTER_MAX) / 2)


# --- Assembly -----------------------------------------------------------

_PANELS = {
    "wall_floor": wall_floor(),
    "wall_ceiling": wall_ceiling(),
    "wall_back": wall_back(),
    "wall_front": wall_front(),
    "wall_left": wall_left(),
    "wall_right": wall_right(),
}

assembly = Assembly()
for _name, _part in _PANELS.items():
    assembly = assembly.add(_name, _part)

# All panel pairs must be non-overlapping: horizontal-vs-vertical panels
# could conflict at corners if the nesting convention slipped; left/right
# are nested between floor/ceiling/front/back and so are disjoint from
# all of them. Assert the full pairwise set to catch any future edit
# that mis-sizes or mis-positions a panel.
_names = list(_PANELS.keys())
for _i, _a in enumerate(_names):
    for _b in _names[_i + 1:]:
        assembly = assembly.assert_no_interference(_a, _b)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Each panel prints flat on the build plate — its own thickness
    # direction is the print's up axis. The default FDM() up axis is
    # +Z, which matches the back and front panels directly; the floor
    # and ceiling panels' native +Y "up" needs `up_axis=(0, 1, 0)`
    # and the left/right panels need `up_axis=(1, 0, 0)`. Since rev 1
    # panels are plain rectangular prisms the overhang check is moot
    # regardless; the wall-thickness check is what matters, and it is
    # orientation-independent.
    for _name, _part in _PANELS.items():
        inspect(
            _part,
            method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=45.0),
            out="outputs",
            name=_name,
        )
