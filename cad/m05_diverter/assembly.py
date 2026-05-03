"""
M05 hex-tower — top-level 4-floor diverter assembly.

Composes four floors with roles `[base, middle, middle, top]` stacked
at world z = `[0, FLOOR_H, 2·FLOOR_H, 3·FLOOR_H]`. Bricks enter above
the top floor and fall down through per-floor traps that divert them
into the floor's baskets (or release them through to the floor below).

Each floor reuses the same sub-assemblies as `single_floor.py`:

- frame — through-columns span all floors as one continuous 2020;
  hex rings, brackets, and radial rails repeat per floor (suffixed by
  floor index).
- ramp_mechanism — top-bearing architecture: bearing+spider geometry
  is uniform across roles (short ring at z_local 0..5). Roles differ
  only at the column bottom: base has a bottom collar (motor + ring-
  gear interface); middle/top have a bottom face flange that mates
  with the column below at the joint plane (z_world = N·220 + 273).
  Each column carries a rotating collar at z_local 220..225 that the
  bearing of the floor *above* grips.
- basket — `FLOOR_LAYOUT` (mixed N=6/9/12/18) replicated per floor.
- base_floor — drive train (motor / drive shaft / motor mount) below
  the rail datum on the base floor only. NOTE: drive_shaft is now
  obsolete (Lego falls through the column axis); see m05_diverter/
  NEXT_STEPS.md "Base-floor drive train" for the off-axis-pinion
  rework.

Cross-floor checks (the value the tower adds beyond `single_floor.py`):

- Upper floor's bearing+spider engages lower floor's column rotating
  collar (z_local 220..225 of lower = z_local 0..5 of upper). Grease
  film clearance — `assert_no_interference` guards the radial gap.
- Lower column top (z = 273 in lower local) vs upper column bottom
  (z = 53 in upper local) tangent at the joint plane. The two flanges
  meet face-to-face; assert_no_interference allows that contact.
- Lower floor's open-trap envelope (max-open free end at z ≈ 36 mm
  above the upper floor's bracket) vs upper floor's column-bottom
  stack — the trap-open zone must thread cleanly between the bearing
  (z ≤ 5) and this floor's column-bottom (z ≥ 53).

Intra-floor pairs (basket-vs-frame, ramp-vs-basket, column-side self-
clearance) are validated by `single_floor.py` and the standalone
sub-assemblies; the geometry is z-translated only, so per-floor
enumeration here would be redundant.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from build123d import Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check

from base_floor import assembly as _base
from basket import assembly as _bkt
from frame import assembly as _frame
from params import FLOOR_H, FLOOR_LAYOUT
from ramp_mechanism import assembly as _ramp


# --- Floor plan --------------------------------------------------------

# (idx, role) — base at idx 0 (world z=0, the bottom). Bricks enter
# above the top floor (idx 3, world z=3·FLOOR_H) and fall in -Z.
FLOOR_PLAN = [
    (0, "base"),
    (1, "middle"),
    (2, "middle"),
    (3, "top"),
]

# Through-column top z. Mirrors the single-floor default
# (add_through_columns z_top = FLOOR_H above the floor's rail datum):
# one extra FLOOR_H above the top floor for the brick-entry zone.
TOP_VERT_Z = 4 * FLOOR_H


def _floor_prefix(idx: int, role: str) -> str:
    return f"f{idx}_{role}__"


# --- Build -------------------------------------------------------------

assembly = Assembly()

# Six through-columns: one set, full-tower span. Replaces the per-
# floor verticals from `add_frame`.
assembly = _frame.add_through_columns(
    assembly,
    z_bot=-_frame.VERT_EXT_BELOW,
    z_top=TOP_VERT_Z,
)

# Per-floor parts.
prefixes: list[str] = []
for idx, role in FLOOR_PLAN:
    pre = _floor_prefix(idx, role)
    prefixes.append(pre)
    fz = idx * FLOOR_H
    fz_loc = Pos(0, 0, fz)

    # Frame horizontals at this floor's rail datum.
    assembly = _frame.add_floor_horizontals(assembly, z=fz, prefix=pre)

    # Column-side: spider / ring / column / hinge pin / ramp / trap-closed.
    assembly = (
        assembly
        .add(f"{pre}bearing_spider", _ramp.bearing_spider(role),
             location=fz_loc)
        .add(f"{pre}ring_bearing", _ramp.ring_bearing(role),
             location=fz_loc)
        .add(f"{pre}column", _ramp.column(role),
             location=fz_loc)
        .add(f"{pre}hinge_pin", _ramp.hinge_pin(),
             location=fz_loc * _ramp.hinge_loc)
        .add(f"{pre}ramp", _ramp.ramp(),
             location=fz_loc * _ramp.ramp_loc)
        .add(
            f"{pre}trap_closed", _ramp.trap(),
            location=fz_loc * _ramp.hinge_loc * Rot(0, _ramp.TRAP_CLOSED_DEG, 0),
        )
    )
    # Trap at max-open: only needed on floors that have an upper
    # neighbour (its envelope reaches up into that next floor's
    # column-bottom zone). Top floor's trap-open has nothing above it
    # to test against — skip to keep the check matrix focused.
    if idx < 3:
        assembly = assembly.add(
            f"{pre}trap_open", _ramp.trap(),
            location=fz_loc * _ramp.hinge_loc * Rot(0, _ramp.TRAP_OPEN_DEG, 0),
        )

    # Baskets per the mixed FLOOR_LAYOUT.
    for center_deg, n_size, notches in FLOOR_LAYOUT:
        bname = f"{pre}basket_{int(round(center_deg)):03d}_{n_size:02d}"
        assembly = assembly.add(
            bname,
            _bkt.basket(
                half_angle_deg=180.0 / n_size,
                notch_left="left" in notches,
                notch_right="right" in notches,
                notch_middle="middle" in notches,
            ),
            location=fz_loc * Rot(0, 0, center_deg),
        )

    # Base-floor-only: drive train (motor + drive shaft + mount) below
    # the rail datum. Its parts live at z ∈ [-105, +5] in the floor
    # frame — placed at fz_loc which is (0, 0, 0) for the base floor.
    if role == "base":
        assembly = (
            assembly
            .add(f"{pre}motor", _base.motor(), location=fz_loc)
            .add(f"{pre}drive_shaft", _base.drive_shaft(), location=fz_loc)
            .add(f"{pre}motor_mount", _base.motor_mount(), location=fz_loc)
        )


# --- Cross-floor checks ------------------------------------------------

# Adjacent-floor pairs only. Non-adjacent floors are >1·FLOOR_H apart
# and the trap-open envelope reaches at most ~70 mm above its hinge,
# so floor i+2 is comfortably out of range.
for lower_idx in range(3):
    lower_pre = prefixes[lower_idx]
    upper_pre = prefixes[lower_idx + 1]
    upper_stack = [
        f"{upper_pre}column",
        f"{upper_pre}ring_bearing",
        f"{upper_pre}bearing_spider",
    ]
    for u in upper_stack:
        assembly = (
            assembly
            .assert_no_interference(f"{lower_pre}trap_open", u)
            .assert_no_interference(f"{lower_pre}column", u)
        )


# --- Base-floor drive train cross-checks -------------------------------

# Drive shaft passes coaxially up through the column-bottom stack
# (spider hub ID ≫ shaft D, ring-bearing ID ≫ shaft D, column collar
# square hole ≫ shaft D). Motor sits well below all frame members.
# Motor mount sits flush with the radial-rail bottom face (tangent at
# z = -RAIL_SIZE) and stops 1 mm short of the rail inner ends radially.
base_pre = prefixes[0]
shaft = f"{base_pre}drive_shaft"
mtr = f"{base_pre}motor"
mount = f"{base_pre}motor_mount"

# Drive shaft vs base-floor column-side stack (clearance check inside
# their respective IDs / interior cavities).
assembly = (
    assembly
    .assert_no_interference(shaft, f"{base_pre}column")
    .assert_no_interference(shaft, f"{base_pre}ring_bearing")
    .assert_no_interference(shaft, f"{base_pre}bearing_spider")
)

# Motor body vs frame + ramp_mechanism on the base floor. Motor sits
# at z ∈ [-105, -25], well below every other modeled part.
for other in (
    f"{base_pre}column",
    f"{base_pre}ring_bearing",
    f"{base_pre}bearing_spider",
    f"{base_pre}ramp",
    f"{base_pre}trap_closed",
    f"{base_pre}trap_open",
):
    assembly = assembly.assert_no_interference(mtr, other)

# Motor mount vs base-floor frame radials and inner hex (closest frame
# members radially — outer hex / corner columns are ≥ 270 mm from
# axis, well outside the mount's 89 mm reach).
for k in range(6):
    assembly = (
        assembly
        .assert_no_interference(mount, f"{base_pre}radial_{k}")
        .assert_no_interference(mount, f"{base_pre}inner_horiz_{k}")
    )

# Motor mount vs base-floor column-side stack. All ramp_mechanism base
# parts sit at z ≥ 0; the mount is at z ∈ [-25, -20]. Tangent contact
# nowhere — pure z separation.
for other in (
    f"{base_pre}column",
    f"{base_pre}ring_bearing",
    f"{base_pre}bearing_spider",
):
    assembly = assembly.assert_no_interference(mount, other)


if __name__ == "__main__":
    check(assembly, out="outputs")
