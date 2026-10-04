"""The base floor's pass-through fall path — a probe fixture, not the floor.

    .venv/bin/khana check cad/m05_diverter/check_base_fall_path.py

`ramp_mechanism/check_fall_path.py` holds the middle floor's pass: flat
`MAX_BRICK_DIM` sticks at 12 yaws down the column past the hanging trap.
The base floor's pass is longer — its column reaches down to the
turntable (`BASE_COL_Z_BOT`) and the brick leaves through the ring gear's
square hole below it, at `base_floor.FALL_EXIT_Z` — and the ring gear is
`base_floor`'s, so the fixture lives here, where both units are visible.

Same sticks, same placement rule (midway between the hanging trap's face
and the nearer of the ramp root and the +X wall), run from the base
column's top down through the ring gear to the exit, held against the
rotating floor (column, trap, ramp), the ring gear, and the bearing
stack. At rest, and that covers every azimuth: the sticks ride with the
column, and the ring gear is bolted to it — it is modeled static in
`drive_train`, at azimuth 0, which is the column's rest azimuth.

Below the exit the column no longer carries the brick, and the static
frame sees it at every azimuth: that is the tower's `base_fall_path`
(the keep-out `drive_train.fall_path`, the swept exit disc down to the
catch, held against the frame and the catch on every tower check), with
`mount_top_clear_of_fall_path`, `drive_clear_of_fall_path` and
`catch_mouth_takes_scatter` in the base_floor builders.
"""

from build123d import Box, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from .base_floor import assembly as _base
from .params import COL_INTERIOR_SIDE, MAX_BRICK_DIM
from .ramp_mechanism import assembly as _ramp

PROBE_W = 8.0                      # a 1×N brick is 7.8 wide
PROBE_YAWS_DEG = tuple(range(0, 180, 15))


def _placed_bbox(a: Assembly, path: str):
    p = a.part(path)
    return (p.location * p.part).bounding_box()


def build_base_fall_path() -> Assembly:
    """The base floor at rest with its ring gear, plus one falling-stick
    probe per yaw from the column top to the exit."""
    floor = (
        _ramp.build_column_side_static("base")
        .with_subassembly("rotating", _ramp.build_column_side_rotating("base"))
        .with_subassembly("drive_train", _base.build_drive_train())
    )
    column = _placed_bbox(floor, "rotating.column")
    x_far = min(_placed_bbox(floor, "rotating.ramp").min.X, COL_INTERIOR_SIDE / 2)
    x_mid = (_placed_bbox(floor, "rotating.trap.trap").max.X + x_far) / 2
    z_lo, z_hi = _base.FALL_EXIT_Z, column.max.Z
    stick = Box(MAX_BRICK_DIM, PROBE_W, z_hi - z_lo)
    a = floor
    for yaw in PROBE_YAWS_DEG:
        name = f"stick_{yaw:03d}"
        a = (
            a
            .with_part(name, stick, location=Pos(x_mid, 0, (z_lo + z_hi) / 2) * Rot(0, 0, yaw))
            .assert_no_interference_between(
                [name],
                ["bearing_spider", "ring_bearing", "rotating", "drive_train.ring_gear"],
            )
        )
    # The binding yaw, measured, as in check_fall_path — plus the ring
    # gear's hole, which is the column's interior carried down.
    for b in (
        "rotating.trap.trap", "rotating.column", "rotating.ramp",
        "drive_train.ring_gear",
    ):
        a = a.assert_distance("stick_000", b, min_mm=_ramp.FALL_PATH_MARGIN / 2)
    return a


assembly = build_base_fall_path()
