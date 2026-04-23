"""
M03 scanner — rigid mirror-mount bracket (rev 1, non-adjustable).

First-rev mount: a flat rectangular printed plate with the first-surface
mirror glued to its front face and four corner through-holes for bolting
the plate to the enclosure wall behind.

This design locks in whatever alignment the printed + assembled part
delivers — no pitch/yaw calibration headroom. A second rev with a
3-point kinematic mount is planned once the scanner envelope has been
validated with this one; see `../NEXT_STEPS.md`.

Coordinate frame (local to this sub-assembly):
    origin = centre of mirror reflective face
    +Z     = mirror normal (outward, toward camera/brick)
    +X, +Y = span the mirror face (rotation about +Z is free)
    z = 0 plane                          : mirror reflective face
    z ∈ [-MIRROR_T, 0]                   : mirror body
    z ∈ [-MIRROR_T - PLATE_T, -MIRROR_T] : printed plate body
    bolt axis = -Z (bolts fire through the plate into the enclosure
                    wall sitting at z ≤ -MIRROR_T - PLATE_T)

The scanner top-level assembly maps this local +Z onto each mirror's
world-space normal and places the local origin at the mirror's world
centre.
"""

from build123d import Box, Cylinder, Part, Pos

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM


# --- Parameters ---------------------------------------------------------

# Mirror — first-surface, square. 80 × 80 mm per the reference spec;
# 3 mm is a typical first-surface mirror stock thickness.
MIRROR_W = 80.0
MIRROR_H = 80.0
MIRROR_T = 3.0

# Printed plate — oversized around the mirror to host corner bolt
# flanges. 15 mm flange on every side → plate is 110 × 110.
PLATE_OVERSIZE = 15.0
PLATE_W = MIRROR_W + 2 * PLATE_OVERSIZE
PLATE_H = MIRROR_H + 2 * PLATE_OVERSIZE
PLATE_T = 5.0

# M4 hardware — clearance hole in the plate; bolt fires -Z into the
# enclosure wall.
BOLT_HOLE_D = 4.5
BOLT_INSET = 8.0    # bolt-hole centre inset from each plate corner

# FDM printability floor.
WALL_MIN = 1.5


# --- Parametric sanity checks -------------------------------------------

# Plate must be larger than the mirror on every side.
assert PLATE_OVERSIZE > 0, "plate must be larger than the mirror"

# Bolt-hole-to-plate-edge wall thickness (critical corner wall).
assert BOLT_INSET - BOLT_HOLE_D / 2 >= WALL_MIN, (
    f"bolt-to-plate-edge wall {BOLT_INSET - BOLT_HOLE_D / 2:.2f} mm < {WALL_MIN} mm"
)

# Bolt hole must sit outside the mirror's glue footprint so it doesn't
# compromise the bond area; bolt-to-mirror-edge distance in X and Y is
# PLATE_OVERSIZE - BOLT_INSET.
assert PLATE_OVERSIZE - BOLT_INSET >= WALL_MIN, (
    f"bolt-to-mirror-edge margin {PLATE_OVERSIZE - BOLT_INSET:.2f} mm"
    f" < {WALL_MIN} mm"
)


# --- Part functions -----------------------------------------------------

def _bolt_hole(plate_t: float = PLATE_T, hole_d: float = BOLT_HOLE_D) -> Part:
    """Cylindrical through-hole along Z, long enough to cut cleanly."""
    return Cylinder(hole_d / 2, plate_t * 3)


def bracket(
    plate_w: float = PLATE_W,
    plate_h: float = PLATE_H,
    plate_t: float = PLATE_T,
    mirror_t: float = MIRROR_T,
    bolt_inset: float = BOLT_INSET,
    bolt_hole_d: float = BOLT_HOLE_D,
) -> Part:
    """Flat rectangular mounting plate. Front face at z = -mirror_t
    (mirror glues here); back face at z = -mirror_t - plate_t (rests
    against the enclosure wall). Four corner through-holes accept
    bolts fired in -Z into the wall."""
    plate_cz = -mirror_t - plate_t / 2
    plate = Pos(0, 0, plate_cz) * Box(plate_w, plate_h, plate_t)
    dx = plate_w / 2 - bolt_inset
    dy = plate_h / 2 - bolt_inset
    for sx in (-1, 1):
        for sy in (-1, 1):
            plate -= (
                Pos(sx * dx, sy * dy, plate_cz)
                * _bolt_hole(plate_t, bolt_hole_d)
            )
    return plate


def mirror_stub(
    w: float = MIRROR_W, h: float = MIRROR_H, t: float = MIRROR_T
) -> Part:
    """Stand-in for the first-surface mirror. Reflective face at z=0;
    body at z ∈ [-t, 0]. Bought, not printed — no inspect() call."""
    return Pos(0, 0, -t / 2) * Box(w, h, t)


# --- Assembly -----------------------------------------------------------

# Mirror sits flush on the bracket's front face (tangent contact at
# z = -MIRROR_T). assert_no_interference treats tangency as zero
# overlap volume, which is what we want here.
assembly = (
    Assembly()
    .add("bracket", bracket())
    .add("mirror", mirror_stub())
    .assert_no_interference("bracket", "mirror")
)


if __name__ == "__main__":
    check(assembly, out="outputs")
    inspect(
        bracket(),
        method=FDM(wall_min_mm=WALL_MIN, overhang_max_deg=45.0),
        out="outputs",
        name="bracket",
    )
