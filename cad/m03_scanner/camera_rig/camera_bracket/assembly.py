"""
M03 scanner — rigid camera-bracket (rev 1, non-adjustable).

First-rev mount for the Raspberry Pi HQ Camera (reference capture
device per `docs/m03-multi-view-scanner.md`). A flat rectangular
printed plate bolts to the enclosure front wall behind the camera;
the camera PCB bolts to the plate's front face via four M2.5 screws
into the camera board's built-in threaded corners. The 2.8 mm
C-mount lens extends forward along +Z (the look axis).

This rev locks whatever pose the printed + assembled part delivers —
no pitch/yaw tweak. Calibration will use a sacrificial shim stack at
the enclosure-wall interface if the first build shows the axis is
off. Kinematic refinement is deferred until the whole rig (mirrors +
camera + platform + enclosure) is assembled and we see what actually
drifts; see `../NEXT_STEPS.md`.

Coordinate frame (local to this sub-assembly):
    origin = centre of camera PCB's rear face (the face that contacts
             the bracket's front face)
    +Z     = lens optical axis, outward (toward brick)
    +X, +Y = span the PCB plane (rotation about +Z is free at this
             rev; top-level assembly chooses roll)
    z = 0                                      : bracket front face
                                                 / PCB rear face
    z ∈ [-PLATE_T, 0]                          : printed plate body
    z ∈ [0, PCB_T]                             : camera PCB body
    z ∈ [PCB_T, PCB_T + LENS_L]                : lens + mount column
                                                 (single cylindrical
                                                 stand-in)
    bolt axis = -Z (corner bolts fire through the plate into the
                    enclosure wall sitting at z ≤ -PLATE_T)
    M2.5 axis = +Z (camera screws fire through the plate into the
                    camera PCB's threaded corner holes)

The scanner top-level assembly maps local +Z onto the camera's
world-space look direction (from the camera position (0, +105.7,
−226.6) toward the origin, i.e. 25° below horizontal toward +Z, +Y
down) and places the local origin at the bracket/PCB interface
plane in world space.

The CSI ribbon cable exits the PCB at one of its 38 mm edges parallel
to the PCB plane; the 15 mm plate overhang leaves room to route it
out past the bracket rim without a cutout. The PCB's central tripod-
mount block protrudes ~1–2 mm from the rear face; at rev 1 we do not
pocket for it and rely on the M2.5 fasteners pulling the four PCB
corners firmly against the plate (small central gap is acceptable).
"""

import math

from build123d import Box, Cylinder, Part, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM


# --- Parameters ---------------------------------------------------------

# Raspberry Pi HQ Camera PCB — 38 × 38 mm, ~1.6 mm FR-4.
PCB_W = 38.0
PCB_H = 38.0
PCB_T = 1.6

# Four M2.5 threaded mounting holes at the PCB corners on a 29 × 29 mm
# square pattern (Pi HQ mechanical drawing). Clearance hole for M2.5
# screw shaft in the bracket = 2.8 mm.
CAM_HOLE_SPACING = 29.0
CAM_HOLE_D = 2.8

# Lens + mount column stand-in. Lumps the CS-mount column, C-mount
# adapter, and 2.8 mm C-mount lens barrel into a single cylinder:
# Ø30 × 55 mm is a conservative envelope for the reference 2.8 mm
# lens. Refine once the actual lens is on hand.
LENS_OD = 30.0
LENS_L = 55.0

# Printed plate — oversized around the PCB to host corner enclosure
# bolts. 15 mm flange on every side → plate is 68 × 68.
PLATE_OVERSIZE = 15.0
PLATE_W = PCB_W + 2 * PLATE_OVERSIZE
PLATE_H = PCB_H + 2 * PLATE_OVERSIZE
PLATE_T = 5.0

# M4 corner hardware — clearance hole in the plate; bolt fires -Z
# into the enclosure wall. Same pattern as mirror_mount rev 1.
BOLT_HOLE_D = 4.5
BOLT_INSET = 8.0

# FDM printability floor.
WALL_MIN = 1.5


# --- Parametric sanity checks -------------------------------------------

assert PLATE_OVERSIZE > 0, "plate must be larger than the PCB"

# Corner M4 bolt wall to plate edge.
assert BOLT_INSET - BOLT_HOLE_D / 2 >= WALL_MIN, (
    f"M4-to-plate-edge wall {BOLT_INSET - BOLT_HOLE_D / 2:.2f} mm < {WALL_MIN} mm"
)

# M2.5 hole inset from the plate corner. Camera holes sit at
# ±CAM_HOLE_SPACING/2; distance to the nearer plate edge is
# PLATE_W/2 − CAM_HOLE_SPACING/2. Subtract hole radius for wall.
_m25_to_edge = PLATE_W / 2 - CAM_HOLE_SPACING / 2 - CAM_HOLE_D / 2
assert _m25_to_edge >= WALL_MIN, (
    f"M2.5-to-plate-edge wall {_m25_to_edge:.2f} mm < {WALL_MIN} mm"
)

# M2.5 hole to M4 corner hole — ensure they don't collide or share
# a knife-edge wall. Worst-case diagonal spacing:
_m25_xy = CAM_HOLE_SPACING / 2
_m4_xy = PLATE_W / 2 - BOLT_INSET
_cam_to_corner_spacing = max(_m4_xy - _m25_xy, 0.0)
assert _cam_to_corner_spacing >= CAM_HOLE_D / 2 + BOLT_HOLE_D / 2 + WALL_MIN, (
    f"M2.5↔M4 wall {_cam_to_corner_spacing:.2f} mm too tight for both holes"
)

# Lens body must clear the M2.5 screw heads / bosses on the PCB
# front face. M2.5 corner holes sit at radial distance
# (CAM_HOLE_SPACING / 2) · √2 from the PCB centre; the lens
# cylinder is centred on origin. Check the radial gap leaves room
# for the screw head footprint (~Ø5 mm typical M2.5 pan head).
_m25_radial = (CAM_HOLE_SPACING / 2) * math.sqrt(2)
_lens_to_m25 = _m25_radial - LENS_OD / 2
assert _lens_to_m25 >= 2.5, (
    f"lens body radial gap to M2.5 hole centre {_lens_to_m25:.2f} mm"
    f" leaves no room for a screw head"
)


# --- Part functions -----------------------------------------------------

def _through_hole(plate_t: float, hole_d: float) -> Part:
    """Cylindrical through-hole along Z, long enough to cut cleanly."""
    return Cylinder(hole_d / 2, plate_t * 3)


def bracket(
    plate_w: float = PLATE_W,
    plate_h: float = PLATE_H,
    plate_t: float = PLATE_T,
    bolt_inset: float = BOLT_INSET,
    bolt_hole_d: float = BOLT_HOLE_D,
    cam_hole_spacing: float = CAM_HOLE_SPACING,
    cam_hole_d: float = CAM_HOLE_D,
) -> Part:
    """Flat rectangular mounting plate. Front face at z = 0 (PCB
    contact plane), back face at z = -plate_t (rests against the
    enclosure wall). Four corner through-holes accept M4 bolts fired
    in -Z into the wall; four inner through-holes accept M2.5 screws
    fired in +Z into the camera PCB's threaded corners."""
    plate_cz = -plate_t / 2
    plate = Pos(0, 0, plate_cz) * Box(plate_w, plate_h, plate_t)

    # Corner M4 bolt holes (to enclosure wall).
    dx = plate_w / 2 - bolt_inset
    dy = plate_h / 2 - bolt_inset
    for sx in (-1, 1):
        for sy in (-1, 1):
            plate -= (
                Pos(sx * dx, sy * dy, plate_cz)
                * _through_hole(plate_t, bolt_hole_d)
            )

    # Inner M2.5 camera-PCB holes.
    cx = cam_hole_spacing / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            plate -= (
                Pos(sx * cx, sy * cx, plate_cz)
                * _through_hole(plate_t, cam_hole_d)
            )

    return plate


def camera_pcb_stub(
    w: float = PCB_W, h: float = PCB_H, t: float = PCB_T
) -> Part:
    """Camera PCB stand-in. Rear face at z=0 (against bracket front);
    body at z ∈ [0, t]. Bought, not printed — no inspect() call."""
    return Pos(0, 0, t / 2) * Box(w, h, t)


def lens_stub(
    od: float = LENS_OD,
    length: float = LENS_L,
    pcb_t: float = PCB_T,
) -> Part:
    """Combined lens + mount column stand-in. Cylinder centred on +Z
    axis, starting at the PCB front face (z = PCB_T) and extending
    LENS_L forward. Bought, not printed."""
    z_bot = pcb_t
    z_top = z_bot + length
    cz = (z_bot + z_top) / 2
    return Pos(0, 0, cz) * Cylinder(od / 2, length)


# --- Assembly -----------------------------------------------------------

# PCB sits flush on the bracket's front face (tangent contact at
# z = 0). Lens/mount column extends forward from the PCB top face.
# assert_no_interference treats tangency as zero overlap volume.
assembly = (
    Assembly()
    .add("bracket", bracket())
    .add("pcb", camera_pcb_stub())
    .add("lens", lens_stub())
    .assert_no_interference("bracket", "pcb")
    .assert_no_interference("bracket", "lens")
    .assert_no_interference("pcb", "lens")
)


if __name__ == "__main__":
    check(assembly, out="outputs")
    inspect(
        bracket(),
        method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=45.0),
        out="outputs",
        name="bracket",
    )
