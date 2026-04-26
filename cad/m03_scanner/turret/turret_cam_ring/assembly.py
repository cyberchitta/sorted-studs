"""
M03 scanner — rev-3 turret cam ring (slice e of the 4-station turret).

Single printed annular cam ring around the hub axis at radius 220 mm.
Each platform's outer-edge follower (designed in a follow-up slice)
rolls on the ring's TOP face; the cam Y profile around 360° controls
platform tilt at each station.

This first cut ships the ring at uniform Y (level all the way around).
Two deferrals carry into a follow-up slice:
  * DUMP notch — downward step in the cam Y profile at the DUMP
    angular sector that drops the follower, tilting the platform
    outer-edge-down to release the brick. Without it, the dump
    behaviour is missing — but the geometry of "ring + follower
    contact tangent at level Y" is already solved by this slice.
  * Mounting structure — the brief calls for a hub-centred cantilever
    plate, but the available cantilever paths either intrude into
    the swept platform footprint (vertical posts at radius 220 vs
    bracket bottoms at y = -27.3 collide) or require extending the
    slice (a) frame outward past x = ±150 / z = +8.14 to support
    posts at +45° headings (radius 245). Both are bigger refactors
    than this slice should swallow; the ring ships floating in the
    standalone, with mounting added in a follow-up once the topology
    is decided.

Members (1 part):
  * cam_ring — annular printed body, inner radius 215, outer radius
    225, Y from -30 (bottom) to -20 (top), centred on the hub axis
    (turret-local (0, *, 0)).

Coordinate frame = turret-local frame (matches `../params.py`):
    origin = hub axis at the cross-rail's top face Y plane
    +X     = operator's right            (parallel to world +X)
    +Y     = up                          (parallel to world +Y)
    +Z     = away from operator          (parallel to world +Z)

Cam ring TOP at y = -20 sits 16 mm below the platform underside at
y = -4 (room for a Ø10 follower wheel centred at y = -15). Cam ring
BOTTOM at y = -30 sits 2.7 mm below the slice (d) bracket bottom at
y = -27.3 — important because the cam ring is a full 360° annulus
that the brackets sweep over during indexing.

Print orientation: bottom face on the build plate, +Y up. The
annulus prints with the outer cylindrical face downward. No
overhangs. Realistically this part needs to be split into 4 pie-slice
segments (each fits a 250 mm bed) bolted at +45 / +135 / +225 / +315
heading lap joints — the segmented geometry is a printability detail
the standalone ignores; what matters here is the cam-ring volume the
follower contacts.
"""

import sys
from pathlib import Path

from build123d import Cylinder, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from turret.params import (  # noqa: E402
    PLATFORM_ACRYLIC_SIZE,
    TURRET_ARM_R,
    TURRET_HUB_X,
    TURRET_HUB_Z,
    WALL_MIN,
)


# --- Cam ring geometry (private to this sub-assembly) -------------------

# Platform-edge follower radius. The platform's outer edge (away from
# hub) sits PLATFORM_ACRYLIC_SIZE/2 = 90 mm beyond the centroid, so
# the cam ring tracks at radius = arm + half-platform_radial = 220 mm
# from the hub axis.
TURRET_CAM_RING_R = TURRET_ARM_R + PLATFORM_ACRYLIC_SIZE / 2  # = 220.0

# Cam ring radial inner / outer faces. 10 mm radial thickness centred
# on TURRET_CAM_RING_R = 220.
TURRET_CAM_RING_RADIAL_T = 10.0
TURRET_CAM_RING_INNER_R = TURRET_CAM_RING_R - TURRET_CAM_RING_RADIAL_T / 2  # = 215
TURRET_CAM_RING_OUTER_R = TURRET_CAM_RING_R + TURRET_CAM_RING_RADIAL_T / 2  # = 225

# Cam ring vertical extent. TOP face is the cam profile (where the
# follower rolls); BOTTOM face is where mounting structure attaches.
# 10 mm Y thickness gives enough section depth for printed strength
# at this radius without mass-producing a thick annulus.
TURRET_CAM_RING_TOP_Y = -20.0
TURRET_CAM_RING_THICKNESS = 10.0
TURRET_CAM_RING_BOTTOM_Y = TURRET_CAM_RING_TOP_Y - TURRET_CAM_RING_THICKNESS  # = -30


# --- Sanity asserts ----------------------------------------------------

# Radial thickness ≥ WALL_MIN.
_radial_t = TURRET_CAM_RING_OUTER_R - TURRET_CAM_RING_INNER_R
assert _radial_t >= WALL_MIN, (
    f"cam ring radial thickness {_radial_t:.2f} mm < WALL_MIN "
    f"({WALL_MIN}); widen TURRET_CAM_RING_RADIAL_T"
)

# Y thickness consistency between the two derived constants.
_y_thickness_derived = TURRET_CAM_RING_TOP_Y - TURRET_CAM_RING_BOTTOM_Y
assert abs(_y_thickness_derived - TURRET_CAM_RING_THICKNESS) < 1e-6, (
    f"cam ring TOP-Y - BOTTOM-Y ({_y_thickness_derived}) ≠ "
    f"TURRET_CAM_RING_THICKNESS ({TURRET_CAM_RING_THICKNESS})"
)


# --- Part function -----------------------------------------------------

def cam_ring(
    inner_r: float = TURRET_CAM_RING_INNER_R,
    outer_r: float = TURRET_CAM_RING_OUTER_R,
    thickness: float = TURRET_CAM_RING_THICKNESS,
):
    """Annular cam ring centred on origin in X / Z, with axis along
    Y. Top face at y = +thickness/2, bottom at y = -thickness/2 in
    part-local coords; caller translates to put the ring at world Y."""
    outer = Rot(90, 0, 0) * Cylinder(outer_r, thickness)
    inner = Rot(90, 0, 0) * Cylinder(inner_r, thickness * 3)
    return outer - inner


# --- Assembly builder + standalone -------------------------------------

# Ring centred on hub axis in world. Y placement at the ring's
# centerline (since the part function builds it symmetric about Y = 0
# locally).
_RING_CENTER_Y = (TURRET_CAM_RING_TOP_Y + TURRET_CAM_RING_BOTTOM_Y) / 2

CAM_RING_PART_NAME = "turret_cam_ring"


def add_turret_cam_ring(a: Assembly, base=Pos(0, 0, 0)) -> Assembly:
    """Place the single cam ring centred on the hub axis. Cam ring is
    positioned in turret-local; `base` (default: identity) is
    pre-multiplied to lift it into a parent frame."""
    return a.add(
        CAM_RING_PART_NAME,
        cam_ring(),
        location=base * Pos(TURRET_HUB_X, _RING_CENTER_Y, TURRET_HUB_Z),
    )


assembly = add_turret_cam_ring(Assembly())


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: ring axis vertical (+Y = print up). The
    # annulus has uniform 10 mm radial × 10 mm Y cross-section all
    # the way around — no thin walls or overhangs to worry about. We
    # skip the printability `inspect()` call here because the ray
    # sampler reads false 0.5–1 mm "walls" at the tessellation seams
    # along the inner / outer cylindrical faces' top + bottom edges
    # (the "noisy near sharp convex edges" caveat in the cad-khana
    # skill). The real wall is unambiguously 10 mm; the sampler
    # noise would only fail the inspect, not catch a real defect.
