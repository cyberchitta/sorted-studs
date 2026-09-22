"""The pass-through fall path — a probe fixture, not the floor.

    .venv/bin/khana check cad/m05_diverter/ramp_mechanism/check_fall_path.py

A brick bound for a lower floor falls down this floor's column past the
hanging trap. The column interior is sized `MAX_BRICK_DIM + 2·BRICK_SLOP`
so that the longest brick the sorter handles — a 1×10 / 2×10, long and
narrow — passes lying flat. This fixture holds that: one probe per yaw,
a vertical slab `MAX_BRICK_DIM` long and `PROBE_W` wide swept through
the column's full height, against every part of the floor at rest.

What "passes" means (owner, 2026-09-22): a long narrow brick passes flat
at **any** yaw. Not any footprint — a 48 × 80 plate at 45° needs 90.5
and fails the bare column. Each probe stands where the brick has most
room across X, midway between the hanging trap's face and whichever is
nearer of the ramp root and the +X wall's inner face (solid below the
exit opening, where the trap still hangs) — read off the placed parts,
so the probe follows the design. The yaws cover a half
turn, so one floor's claims cover every floor whatever its opening
azimuth.

Scope: the middle role (top is identical here). The base floor's pass
to the fallback bin runs through the turntable and is not covered.
The trap is at rest (hanging) — the swing is `make_assembly`'s.
"""

from build123d import Box, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from ..params import COL_INTERIOR_SIDE, MAX_BRICK_DIM
from .assembly import FALL_PATH_MARGIN, build_column_side_rotating, build_column_side_static

PROBE_W = 8.0                      # a 1×N brick is 7.8 wide
PROBE_YAWS_DEG = tuple(range(0, 180, 15))


def _placed_bbox(a: Assembly, path: str):
    p = a.part(path)
    return (p.location * p.part).bounding_box()


def build_fall_path(role: str = "middle") -> Assembly:
    """The floor at rest, plus one falling-stick probe per yaw."""
    floor = (
        build_column_side_static(role)
        .with_subassembly("rotating", build_column_side_rotating(role))
    )
    column = _placed_bbox(floor, "rotating.column")
    x_far = min(_placed_bbox(floor, "rotating.ramp").min.X, COL_INTERIOR_SIDE / 2)
    x_mid = (_placed_bbox(floor, "rotating.trap.trap").max.X + x_far) / 2
    z_lo, z_hi = column.min.Z, column.max.Z
    stick = Box(MAX_BRICK_DIM, PROBE_W, z_hi - z_lo)
    a = floor
    for yaw in PROBE_YAWS_DEG:
        name = f"stick_{yaw:03d}"
        # Two calls: a subtree path expands only as a whole group — inside
        # a list it is read as a part name and skips as absent.
        a = (
            a
            .with_part(name, stick, location=Pos(x_mid, 0, (z_lo + z_hi) / 2) * Rot(0, 0, yaw))
            .assert_no_interference_between([name], ["bearing_spider", "ring_bearing"])
            .assert_no_interference_between([name], "rotating")
        )
    # The binding yaw, measured: lying along X the stick has half the
    # margin to spare on each side.
    for b in ("rotating.trap.trap", "rotating.column", "rotating.ramp"):
        a = a.assert_distance("stick_000", b, min_mm=FALL_PATH_MARGIN / 2)
    return a


assembly = build_fall_path()
