"""
M03 scanner — rev-3 platform: acrylic plate + bonded hinge brackets.

The "platform" in operational terms: the moving piece that hinges
forward to drop bricks. Optical floor (transparent PMMA plate) plus
two small printed hinge brackets bonded to its underside near the
front edge. The brackets' pin bores share the world-space hinge axis
with `../platform_hinge_bearing/`'s two pillow-blocks; one continuous
Ø5 round-stock hinge pin (not modelled — stock hardware) threads
through bearings + brackets together.

Members:
  * acrylic_plate — 180 × 180 × 4 mm PMMA, top face at world y = 0.
    Sized to clear the rev-1 below-mirror sightline cones (≤ Ø172
    at the platform plane) plus a few mm of edge margin for future
    edge clamps. Bought as sheet stock and laser-cut to size; not
    printed.
  * bracket_left, bracket_right — printed pillow-blocks bonded to
    the acrylic underside. Two world placements of one body; the
    body has X→-X mirror symmetry so both sides print from the
    same STL.

Coordinate frame = world frame (`../params.py`):
    origin = brick centroid on the platform top face
    +X     = operator's right
    +Y     = up
    +Z     = away from operator (toward back of machine)

Bracket-local frame:
    origin = bottom-face centre of the bracket block
    +X     = along hinge-pin axis (world ±X)
    +Y     = up toward the acrylic underside
    +Z     = world +Z

Bond joint: bracket top face is BOND_GAP below the acrylic underside.
Same idiom as platform_frame's CORNER_GAP — a small modeled gap keeps
the cross-boundary interference check at clean zero. In reality the
gap is filled by the structural-acrylic epoxy bond layer.

Print orientation: bracket bottom on build plate, +Y up. Pin bore
prints horizontal (~Ø5.4), bridge-able by FDM without supports.
Threshold raised to 90° on the printability inspect, same justification
as platform_hinge_bearing.
"""

import sys
from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot

from cad_khana.mechanism.assembly import Assembly
from cad_khana.mechanism.check import check
from cad_khana.printability.inspect import inspect
from cad_khana.printability.methods import FDM

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from params import (  # noqa: E402
    PLATFORM_ACRYLIC_SIZE,
    PLATFORM_ACRYLIC_T,
    PLATFORM_HINGE_BRACKET_X,
    PLATFORM_HINGE_PIN_D,
    PLATFORM_HINGE_PIN_Y,
    PLATFORM_HINGE_PIN_Z,
    WALL_MIN,
)


# --- Acrylic plate ------------------------------------------------------

def acrylic_plate(
    size: float = PLATFORM_ACRYLIC_SIZE,
    thickness: float = PLATFORM_ACRYLIC_T,
):
    """180 × 4 × 180 mm PMMA plate centred on world origin in X / Z,
    top face at y = 0."""
    return Pos(0, -thickness / 2, 0) * Box(size, thickness, size)


# --- Hinge bracket ------------------------------------------------------

# Block dimensions in bracket-local axes.
BLOCK_X = 30.0          # along hinge axis
BLOCK_Y = 18.0          # bottom face up to bond-side top face
BLOCK_Z = 30.0          # along world ±Z

# Modeled bond gap between bracket top face and acrylic underside.
# Filled by epoxy in the real assembly.
BOND_GAP = 0.3

# Pin bore: M5 hinge pin + 0.4 mm diametric clearance.
PIN_BORE_D = PLATFORM_HINGE_PIN_D + 0.4

# Bracket centerline Z in world. Picked so the bracket extends half
# its Z under the acrylic (bond zone) and half forward of the acrylic
# front edge into open air (overhang) to host the pin axis at world
# z = -100. Concretely: BRACKET_Z_CENTRE_WORLD = -ACRYLIC_SIZE / 2
# = -90 → bracket Z span [-105, -75], bond zone z ∈ [-90, -75]
# (15 mm × 30 mm = 450 mm²), pin axis 10 mm forward of bracket centre.
BRACKET_Z_CENTRE_WORLD = -PLATFORM_ACRYLIC_SIZE / 2

# Pin axis position in bracket-local. Derived from the world pin
# coordinates and the bracket's world placement so the bore lines up
# with the bearing's pin bore exactly.
_BRACKET_Y_BOTTOM_WORLD = -PLATFORM_ACRYLIC_T - BOND_GAP - BLOCK_Y
_PIN_Y_LOCAL = PLATFORM_HINGE_PIN_Y - _BRACKET_Y_BOTTOM_WORLD
_PIN_Z_LOCAL = PLATFORM_HINGE_PIN_Z - BRACKET_Z_CENTRE_WORLD


# --- Sanity asserts -----------------------------------------------------

# Pin bore wall to bracket Y faces.
assert _PIN_Y_LOCAL - PIN_BORE_D / 2 >= WALL_MIN, (
    f"pin-bore wall to bracket bottom Y "
    f"{_PIN_Y_LOCAL - PIN_BORE_D / 2:.2f} mm < WALL_MIN ({WALL_MIN}); "
    f"raise BLOCK_Y or move PLATFORM_HINGE_PIN_Y down"
)
assert (BLOCK_Y - _PIN_Y_LOCAL) - PIN_BORE_D / 2 >= WALL_MIN, (
    f"pin-bore wall to bracket top Y "
    f"{(BLOCK_Y - _PIN_Y_LOCAL) - PIN_BORE_D / 2:.2f} mm < WALL_MIN "
    f"({WALL_MIN})"
)

# Pin bore wall to bracket Z faces.
assert (BLOCK_Z / 2 + _PIN_Z_LOCAL) - PIN_BORE_D / 2 >= WALL_MIN, (
    f"pin-bore wall to bracket front Z "
    f"{(BLOCK_Z / 2 + _PIN_Z_LOCAL) - PIN_BORE_D / 2:.2f} mm < WALL_MIN "
    f"({WALL_MIN}); shift BRACKET_Z_CENTRE_WORLD or grow BLOCK_Z"
)
assert (BLOCK_Z / 2 - _PIN_Z_LOCAL) - PIN_BORE_D / 2 >= WALL_MIN, (
    f"pin-bore wall to bracket back Z "
    f"{(BLOCK_Z / 2 - _PIN_Z_LOCAL) - PIN_BORE_D / 2:.2f} mm < WALL_MIN "
    f"({WALL_MIN})"
)

# Bracket's bond zone must overlap the acrylic footprint in Z. Bracket
# back edge in world Z must sit inside the acrylic front edge.
_bracket_z_back_world = BRACKET_Z_CENTRE_WORLD + BLOCK_Z / 2
assert _bracket_z_back_world > -PLATFORM_ACRYLIC_SIZE / 2, (
    f"bracket back edge world Z {_bracket_z_back_world:.1f} ≤ acrylic "
    f"front edge {-PLATFORM_ACRYLIC_SIZE / 2:.1f}; bond zone is empty"
)

# Bracket entirely within acrylic X footprint (with margin for bond).
assert PLATFORM_HINGE_BRACKET_X + BLOCK_X / 2 <= PLATFORM_ACRYLIC_SIZE / 2, (
    f"bracket outboard X face "
    f"{PLATFORM_HINGE_BRACKET_X + BLOCK_X / 2:.1f} > acrylic outboard "
    f"X face {PLATFORM_ACRYLIC_SIZE / 2:.1f}; bracket overhangs in X"
)


# --- Part function ------------------------------------------------------

def bracket(
    block_x: float = BLOCK_X,
    block_y: float = BLOCK_Y,
    block_z: float = BLOCK_Z,
    pin_y_local: float = _PIN_Y_LOCAL,
    pin_z_local: float = _PIN_Z_LOCAL,
    pin_bore_d: float = PIN_BORE_D,
):
    """Single printed hinge bracket: rectangular block bonded to the
    acrylic underside, with a horizontal pin bore at the front-bottom
    that meets the bearing pin in world space. Symmetric across the
    bracket-local YZ plane (X → -X), so the same body fits both world
    instances."""
    body = Pos(0, block_y / 2, 0) * Box(block_x, block_y, block_z)

    # Horizontal pin bore (axis along ±X). Cylinder default axis is +Z;
    # rotate 90° about Y so the axis lies along ±X.
    body -= (
        Pos(0, pin_y_local, pin_z_local)
        * Rot(0, 90, 0)
        * Cylinder(pin_bore_d / 2, block_x * 3)
    )

    return body


# --- Assembly -----------------------------------------------------------

assembly = Assembly().add("acrylic", acrylic_plate())

for _name, _x in (
    ("bracket_left", -PLATFORM_HINGE_BRACKET_X),
    ("bracket_right", +PLATFORM_HINGE_BRACKET_X),
):
    assembly = assembly.add(
        _name,
        bracket(),
        location=Pos(_x, _BRACKET_Y_BOTTOM_WORLD, BRACKET_Z_CENTRE_WORLD),
    )

# Acrylic and brackets are separated in Y by BOND_GAP (clean zero
# overlap); the two brackets are ~110 mm apart in X.
assembly = (
    assembly
    .assert_no_interference("acrylic", "bracket_left")
    .assert_no_interference("acrylic", "bracket_right")
    .assert_no_interference("bracket_left", "bracket_right")
)


if __name__ == "__main__":
    check(assembly, out="outputs")
    # Print orientation: bracket bottom on build plate (+Y up).
    # Threshold 90° for the same horizontal-Ø5.4 bridging reason as
    # platform_hinge_bearing — real FDM bridges this cleanly, but the
    # ray sampler reads the lower-half bore ceiling as a near-90°
    # overhang.
    inspect(
        bracket(),
        method=FDM(
            up_axis=(0, 1, 0), wall_min_mm=WALL_MIN, overhang_max_deg=90.0
        ),
        out="outputs",
        name="bracket",
    )
