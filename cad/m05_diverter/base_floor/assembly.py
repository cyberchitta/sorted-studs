"""
M05 hex-tower — base-floor drive train.

Stationary structure that drives the rotating column from below.
Base-floor-only: floors 2-4 use only the column-bottom stack from
ramp_mechanism (spider / ring bearing / column).

Realizes the "single motor at the base drives the full 4-floor
column as one rotating shaft" architectural commitment as three
placeholder parts:

- motor — NEMA-23-style 60×60×80 box stub. Final NEMA size, gear
  ratio, and torque budget are open.
- drive_shaft — D14 cylinder along the column axis from motor top
  face up into the column collar's interior cavity. The torque-
  transfer detail (set screws / keyway / bolted hub at the column-
  collar interface) is open.
- motor_mount — printed disc that bolts up into the radial-rail
  bottom faces (T-slot from below). Detailed bolt pattern and
  motor-flange interface are open.

The base bearing (axial+radial+motor-reaction-torque) currently
re-uses the per-floor `ring_bearing` for the base role in
ramp_mechanism — see `m05_diverter/NEXT_STEPS.md` "Ring-bearing
type choice" for the open question on whether the base needs a
heavier thrust+radial unit instead of the radial-only ring.

Coordinate frame (matches ramp_mechanism / params):
    origin = column axis ∩ rail-top datum (z=0)
    +Z     = up
    +X     = radial outward toward the exit opening
    z=0    = rail top face (frame datum)
    z<0    = below the floor — motor + mount + lower drive shaft sit here
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build123d import Box, Cylinder, Part, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

from params import RAIL_INNER_R, RAIL_SIZE


# --- Parameters ---------------------------------------------------------

# Motor stub — NEMA-23 footprint (60 mm body for round numbers; real
# NEMA-23 is 57 mm, swap when the motor is selected).
MOTOR_BODY_W = 60.0
MOTOR_BODY_H = 80.0
MOTOR_TOP_Z = -25.0                   # output face — matches motor_mount underside
MOTOR_BOTTOM_Z = MOTOR_TOP_Z - MOTOR_BODY_H

# Drive shaft — single cylinder from motor top face up into the column
# collar's interior cavity (the collar has a 90×90 square through-hole;
# D14 sits well inside). Represents a coupler-extended drive shaft;
# the actual NEMA-23 8 mm output stub plus a rigid coupler reads
# geometrically as one D14 cylinder for stub purposes.
SHAFT_D = 14.0
SHAFT_Z_BOT = MOTOR_TOP_Z
SHAFT_Z_TOP = 5.0                     # = COLLAR_T (top of column-base collar)

# Motor mount — disc with central shaft pass-through. Top face sits
# flush with the radial-rail bottom face (z = -RAIL_SIZE = -20); bolts
# go up from below into the rail T-slots. OD stops 1 mm short of the
# rail inner ends so the disc fits between the six rail spokes.
MOUNT_T = 5.0
MOUNT_Z_TOP = -RAIL_SIZE                          # -20 — rail bottom face
MOUNT_Z_BOT = MOUNT_Z_TOP - MOUNT_T               # -25 — flush with motor top face
MOUNT_OD = 2 * (RAIL_INNER_R - 1.0)               # 178 — 1 mm clearance off rails
MOUNT_SHAFT_HOLE_D = SHAFT_D + 4.0                # 18 — 2 mm radial clearance

WALL_MIN = 1.5


# --- Part functions -----------------------------------------------------

def motor() -> Part:
    """NEMA-23-style stub: square body, output face at MOTOR_TOP_Z."""
    z = (MOTOR_TOP_Z + MOTOR_BOTTOM_Z) / 2
    return Pos(0, 0, z) * Box(MOTOR_BODY_W, MOTOR_BODY_W, MOTOR_BODY_H)


def drive_shaft() -> Part:
    """Rotating drive shaft from motor top face to column-collar interior."""
    h = SHAFT_Z_TOP - SHAFT_Z_BOT
    z = (SHAFT_Z_TOP + SHAFT_Z_BOT) / 2
    return Pos(0, 0, z) * Cylinder(SHAFT_D / 2, h)


def motor_mount() -> Part:
    """Printed disc that bolts to the radial-rail bottom faces."""
    z = (MOUNT_Z_TOP + MOUNT_Z_BOT) / 2
    body = Pos(0, 0, z) * Cylinder(MOUNT_OD / 2, MOUNT_T)
    bore = Pos(0, 0, z) * Cylinder(MOUNT_SHAFT_HOLE_D / 2, MOUNT_T * 2)
    return body - bore


# --- Assembly -----------------------------------------------------------

def make_assembly() -> Assembly:
    """Standalone base-floor drive-train assembly.

    Three parts meet near `MOTOR_TOP_Z = -25`:
    - motor body top face at z = -25
    - drive shaft bottom face at z = -25
    - motor mount bottom face at z = -25
    All tangent or radially separated; no overlap.
    """
    return (
        Assembly()
        .add("motor", motor())
        .add("drive_shaft", drive_shaft())
        .add("motor_mount", motor_mount())
        .assert_no_interference("motor", "drive_shaft")
        .assert_no_interference("motor", "motor_mount")
        .assert_no_interference("drive_shaft", "motor_mount")
    )


assembly = make_assembly()


if __name__ == "__main__":
    check(assembly, out="outputs")
    method = FDM(wall_min_mm=WALL_MIN, overhang_max_deg=90.0)
    inspect(motor_mount(), method=method, out="outputs", name="motor_mount")
