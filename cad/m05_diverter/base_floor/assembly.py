"""
M05 hex-tower — base-floor drive train (off-axis pinion + ring gear).

Stationary structure that drives the rotating column from below.
Base-floor-only: floors 2-4 use only the column-bottom stack from
ramp_mechanism (spider / ring bearing / column).

Realizes the "single motor at the base, off-axis, gear-coupled to a
ring gear on the base column's bottom collar" architectural commitment.
The brick fall path runs straight down the column axis (90×90 mm
interior), so **no central drive shaft can live there** — the obsolete
coaxial `drive_shaft` from the previous pass is gone. Drive is taken
to the column's periphery instead:

- motor — NEMA-17-style 42×42×40 box stub, mounted **off-axis** on a
  back-side radial-rail azimuth (away from the +X exit/ramp), body
  hanging below the rail datum with its output shaft pointing up.
  Final NEMA size + gearbox choice still open.
- pinion — small gear on the motor output shaft, meshing with the ring
  gear. Modeled cheap as its pitch cylinder (no teeth); the real
  involute profile is a detail-layer concern.
- ring_gear — printed annulus bolted up to the underside of the base
  column's bottom flange (z = BASE_COL_Z_BOT, above the turntable), teeth
  on its OD below the rails; the standoff between drops through the
  turntable's and seat's bores. Modeled cheap as a pitch
  annulus with the same square brick-pass-through hole as the collar.
  The "separate bolted-on ring vs teeth integral to the printed collar
  OD" choice (see `_notes/state/m05-diverter.md`) is resolved here as the
  bolt-on ring: it keeps the gear off the structural column and
  swappable if the ratio changes, and leaves ramp_mechanism's
  `column("base")` untouched.
- motor_mount — printed radial arm at the rail-bottom plane; bolts up
  into one radial rail's T-slot and carries the motor below via the
  NEMA face. Detailed bolt pattern + flange interface still open.

Gear geometry is the cheap two-fidelity-tier model: pitch cylinders,
no teeth. Mesh validity is a min+max `assert_distance` band ("close
but not touching") rather than a volume check, since meshing gears
are meant to touch. The pinion-tip-clears-the-rail-ring margin is the
tight constraint; it needs the rails, so it lives at the tower root
as a tip-circle (`grow_a_mm`) assert_distance.

The base bearing (axial + radial + motor-reaction torque) currently
re-uses the per-floor `ring_bearing` for the base role in
ramp_mechanism — see `_notes/state/m05-diverter.md` for the open question
on whether the base needs a heavier thrust+radial unit (or a
commercial turntable) instead of the radial-only printed ring.

Coordinate frame (matches ramp_mechanism / params):
    origin = column axis ∩ rail-top datum (z=0)
    +Z     = up
    +X     = radial outward toward the exit opening (azimuth 0°)
    z=0    = rail top face (frame datum) = turntable seat bottom
    z=16.5 = BASE_COL_Z_BOT — base flange underside, ring-gear standoff top
    z<0    = below the floor — gear mesh, pinion, mount, motor sit here
"""

import math
from typing import Callable


from build123d import Box, Cylinder, Part, Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from ..params import (
    BASE_COL_Z_BOT, COL_INTERIOR_SIDE, COLLAR_OD, RAIL_INNER_R, RAIL_SIZE,
)


# --- Gear train ---------------------------------------------------------
#
# Spur pair: a small pinion on the off-axis motor shaft drives a large
# ring gear bolted to the base column's bottom collar. The ring's pitch
# diameter tracks the collar OD so the gear stays compact on the collar.
MODULE = 1.5                          # printable spur module (low torque)
PRESSURE_ANGLE_DEG = 20.0             # documentation only — cheap model has no teeth
RING_TEETH = round(COLLAR_OD / MODULE)            # 92 — pitch D ≈ collar OD
# 17 T is the undercut floor for a 20° PA involute — and the tooth count
# m02's rim-drive pinion settled on (`_notes/decisions/m02-rotor-drive.md`),
# so the two modules share one printed pinion. The tip lands at r ≈ 96.2,
# which is what pushed RAIL_INNER_R out to 100 (params.py).
PINION_TEETH = 17
RING_PITCH_D = RING_TEETH * MODULE                 # 138.0
PINION_PITCH_D = PINION_TEETH * MODULE             # 18.0
GEAR_RATIO = RING_TEETH / PINION_TEETH             # ≈ 7.67 : 1
ADDENDUM = MODULE                                  # tip = pitch + module
MESH_BACKLASH = 0.2                   # extra center distance for FDM mesh play

# Center distance = sum of pitch radii + a touch of backlash. The cheap
# pitch cylinders then sit MESH_BACKLASH apart (no overlap) so the
# pinion-vs-ring no-interference is robust; the `gear_mesh_backlash`
# assert_distance in `build_drive_train` measures the placed gap and
# is the authoritative mesh check.
CENTER_DIST = (RING_PITCH_D + PINION_PITCH_D) / 2 + MESH_BACKLASH   # 81.95

# Pitch ≈ collar OD: the ring gear must seat on the collar's bolt face.
# Declared as a `ring_gear_collar_seat` assert_scalar in
# `build_drive_train` — the collar is not a part of this sub-assembly,
# so the claim is a recorded scalar rather than a part-pair distance.

# Tight constraint: the pinion's tip circle must stay inside the radial-
# rail ring (rails start at RAIL_INNER_R). This is the packaging knife-
# edge — the pinion is squeezed between the Ø138 collar and the Ø200
# rail-inner ring. The rails are not parts of this sub-assembly, so
# the claim lives at the tower root (`.._add_drive_train_checks`) as a
# `grow_a_mm=ADDENDUM` assert_distance measuring from the tip circle
# while the modeled body stays a pitch cylinder. If it fails: shrink
# PINION_TEETH or push the rails out.


# --- Z-band layout (top → bottom) ---------------------------------------
#
# The gear mesh is dropped BELOW the ramp_mechanism bearing spider. That
# spider's saddles clamp the radial rails at the same azimuths a
# rail-mounted motor wants, and they dip to z ≈ -7 reaching in to
# r ≈ 83.5 — straight through the pinion's tip path. The ring gear
# itself (r ≤ 69) always clears the saddle radially, but the pinion
# (tip r ≈ 96.2) does not, so the whole mesh plane sits under the saddle
# and the ring gear reaches the collar via a short standoff. The
# top-level `pinion`-vs-`bearing_spider` no-interference assert is the
# guard that catches any future saddle-depth drift.
#
#   z=16.5 base flange underside / ring-gear standoff top (BASE_COL_Z_BOT)
#   z=8    turntable bottom / seat top (ramp_mechanism)
#   z=0    rail top / seat bottom
#   z=-7   spider-saddle bottom (ramp_mechanism) — mesh stays below
#   z=-9   gear-mesh top (pinion top, ring-gear teeth top)
#   z=-17  gear-mesh bottom / ring-gear bottom
#   z=-18  pinion bottom
#   z=-20  rail bottom / motor-mount top (bolts up into rail T-slot)
#   z=-25  motor-mount bottom / motor face (NEMA bolts up into the mount)
#   z=-65  motor body bottom
GEAR_FACE_W = 8.0                                  # mesh face width
MESH_Z_TOP = -9.0                                  # below the spider saddle (≈ -7)
MESH_Z_BOT = MESH_Z_TOP - GEAR_FACE_W              # -17

RING_GEAR_Z_TOP = BASE_COL_Z_BOT                   # 16.5 — standoff bolts to the base flange underside
RING_GEAR_Z_BOT = MESH_Z_BOT                       # -17 — annulus runs collar → teeth

PINION_Z_TOP = MESH_Z_TOP                          # -9
PINION_FACE_W = GEAR_FACE_W + 1.0                  # 9 — overhangs the gear face
PINION_Z_BOT = PINION_Z_TOP - PINION_FACE_W        # -18

MOUNT_T = 5.0
MOUNT_Z_TOP = -RAIL_SIZE                           # -20 — rail bottom face
MOUNT_Z_BOT = MOUNT_Z_TOP - MOUNT_T                # -25

# Motor — NEMA-17 footprint (42.3 mm body), output face flush with the
# mount underside; body hangs below. Real NEMA-17 is 42.3 mm square.
MOTOR_BODY_W = 42.3
MOTOR_BODY_H = 40.0
MOTOR_TOP_Z = MOUNT_Z_BOT                          # -25 — face bolts up to the mount
MOTOR_BOTTOM_Z = MOTOR_TOP_Z - MOTOR_BODY_H        # -65

# Off-axis placement: on a back-side radial-rail azimuth (rails sit at
# (k+0.5)·60°). 210° is two sections back from the +X exit/ramp at 0°,
# so the motor stays clear of the delivery side. The mount arm runs
# along this rail; pinion + motor axis sit at CENTER_DIST on it.
MOTOR_AZIMUTH_DEG = 210.0

# Motor mount — radial arm under the chosen rail. Inner end covers the
# motor footprint (motor spans r ≈ CENTER_DIST ∓ MOTOR_BODY_W/2 ≈ 61..103);
# outer end overlaps the rail (r ≥ RAIL_INNER_R) by 10 mm to bolt.
# Central clearance hole passes the bare output shaft (the pinion sits
# above the mount).
MOUNT_R_INNER = 52.0
MOUNT_R_OUTER = RAIL_INNER_R + 10.0                    # 110
MOUNT_R_CENTER = (MOUNT_R_INNER + MOUNT_R_OUTER) / 2   # 81
MOUNT_L = MOUNT_R_OUTER - MOUNT_R_INNER                # 58
MOUNT_W = MOTOR_BODY_W + 4.0                           # 46.3 — straddles the motor
MOUNT_SHAFT_HOLE_D = 12.0                              # clears the bare output shaft

WALL_MIN = 1.5


# --- Part functions -----------------------------------------------------

def _on_drive_axis(part: Part, z_center: float) -> Part:
    """Place a part centered on the off-axis motor/pinion axis at the
    given z. The drive axis is at radius CENTER_DIST on azimuth
    MOTOR_AZIMUTH_DEG from the column axis."""
    return Rot(0, 0, MOTOR_AZIMUTH_DEG) * Pos(CENTER_DIST, 0, z_center) * part


def motor() -> Part:
    """NEMA-17-style stub: square body, output face up at MOTOR_TOP_Z,
    body hanging below. Placed off-axis on the drive axis."""
    z = (MOTOR_TOP_Z + MOTOR_BOTTOM_Z) / 2
    return _on_drive_axis(Box(MOTOR_BODY_W, MOTOR_BODY_W, MOTOR_BODY_H), z)


def pinion() -> Part:
    """Drive pinion on the motor shaft, meshing with the ring gear.
    Cheap model: pitch cylinder (no teeth). Placed off-axis on the
    drive axis at CENTER_DIST from the column axis."""
    z = (PINION_Z_TOP + PINION_Z_BOT) / 2
    return _on_drive_axis(Cylinder(PINION_PITCH_D / 2, PINION_FACE_W), z)


def ring_gear() -> Part:
    """Ring gear bolted to the base column's bottom flange underside.
    The upper section is a standoff (flange underside z = BASE_COL_Z_BOT
    down through the turntable and seat bores to the saddle-clearing
    mesh plane); the lower section carries the teeth. Cheap
    model: a single pitch-diameter annulus with the same square brick-
    pass hole as the collar (teeth omitted). Centered on the column axis."""
    h = RING_GEAR_Z_TOP - RING_GEAR_Z_BOT
    z = (RING_GEAR_Z_TOP + RING_GEAR_Z_BOT) / 2
    disc = Pos(0, 0, z) * Cylinder(RING_PITCH_D / 2, h)
    hole = Pos(0, 0, z) * Box(COL_INTERIOR_SIDE, COL_INTERIOR_SIDE, h * 2)
    return disc - hole


def motor_mount() -> Part:
    """Printed radial arm at the rail-bottom plane. Bolts up into one
    radial rail's T-slot at its outer end and carries the motor below
    via the NEMA face; a central hole passes the output shaft up to the
    pinion."""
    z = (MOUNT_Z_TOP + MOUNT_Z_BOT) / 2
    arm = Rot(0, 0, MOTOR_AZIMUTH_DEG) * Pos(MOUNT_R_CENTER, 0, z) * Box(
        MOUNT_L, MOUNT_W, MOUNT_T
    )
    bore = _on_drive_axis(Cylinder(MOUNT_SHAFT_HOLE_D / 2, MOUNT_T * 2), z)
    return arm - bore


# --- Assembly -----------------------------------------------------------

def build_drive_train(
    material_fn: Callable[[str], str | None] | None = None,
) -> Assembly:
    """Off-axis drive train — ring gear + pinion + motor + motor mount
    — as a sub-`Assembly` in the base-floor-local frame (rail-top
    datum at z=0; every part hangs below it). The part functions carry
    their own off-axis placement, so all four are placed at identity.
    ``material_fn`` follows the same leaf-name contract as the frame
    builders. Colour is not a parameter — the unit root paints the
    finished tree (``cad/design_systems.py:paint``).
    """
    def _mt(name: str) -> str | None:
        return material_fn(name) if material_fn is not None else None

    a = Assembly()
    for n, part in (
        ("ring_gear", ring_gear()),
        ("pinion", pinion()),
        ("motor", motor()),
        ("motor_mount", motor_mount()),
    ):
        a = a.with_part(n, part, material=_mt(n))
    # Mesh validity — close but not touching: the cheap pitch blanks
    # must sit in the [MESH_BACKLASH, MESH_BACKLASH + 0.1] band (too
    # far = no mesh, too close = jam; real teeth interleave within
    # it). Declared here, where both gears live, so every composed
    # run evaluates the propagated form. Authoritative over the
    # CENTER_DIST derivation above: this measures the placed geometry.
    # The placed gap equals MESH_BACKLASH by construction; bound
    # comparisons carry the library's 1e-6 tolerance, so measured ==
    # bound passes despite solver noise.
    a = a.assert_distance(
        "ring_gear", "pinion",
        min_mm=MESH_BACKLASH, max_mm=MESH_BACKLASH + 0.1,
        name="gear_mesh_backlash",
    )
    # The bolt-on ring must seat on the base collar's face, so its pitch
    # diameter has to track COLLAR_OD to within one tooth height. The
    # collar belongs to another sub-assembly, so this is a recorded
    # scalar rather than a part-pair distance — but recording it still
    # beats a bare assert: the measured offset lands in mechanism.json
    # and `khana diff` sees it drift before it crosses the bound.
    a = a.assert_scalar(
        "ring_gear_collar_seat",
        abs(RING_PITCH_D - COLLAR_OD),
        le=2 * MODULE,
        detail=(
            f"|ring pitch D {RING_PITCH_D:.1f} − base collar OD "
            f"{COLLAR_OD:.1f}| must stay within one tooth height "
            f"({2 * MODULE:.1f} mm); re-pick RING_TEETH if it fails"
        ),
    )
    return a


def make_assembly() -> Assembly:
    """Standalone base-floor drive-train assembly.

    Four parts: ring gear on the column axis (z = -17..0), pinion on the
    off-axis drive axis (z = -18..-9) meshing with it, motor below the
    mount (z = -65..-25), and the mount arm bridging the two (z =
    -25..-20). The pinion ↔ ring *mesh* (the load-bearing relationship)
    is the `gear_mesh_backlash` assert_distance declared in
    `build_drive_train`; the no-interference assert here is a weaker
    guard that the cheap pitch blanks don't gross-overlap.
    """
    return (
        build_drive_train()
        .assert_no_interference("ring_gear", "pinion")
        .assert_no_interference("ring_gear", "motor")
        .assert_no_interference("ring_gear", "motor_mount")
        .assert_no_interference("pinion", "motor")
        .assert_no_interference("pinion", "motor_mount")
        .assert_no_interference("motor", "motor_mount")
    )


assembly = make_assembly()
