"""
M03 scanner — sightline-cone check module (external claims).

A **check module**: it imports the product assembly, composes a
verification fixture on top of it, declares the claims that fixture
exists to test, and exposes the result as the `assembly()` factory.

    .venv/bin/khana check cad/m03_scanner/check_cones.py
    # → cad/m03_scanner/outputs/check_cones/mechanism.json

**Nothing imports this module.** That is the whole reason the probe
problem dissolves: the cones below are ordinary `with_part` solids,
and any exporter handed this assembly would write them into the STL
and STEP beside the real parts. `assembly.py` is the product module —
what `render*.py`, `export_glb*.py`, and `cad/assembly.py` import, and
the only one any exporter ever sees. `khana export cad/assembly.py` is
therefore cone-free by construction, not by a flag.

--- Sightline cones (silhouette assertion, was c7-c) -----------------

One check-only stub solid per below-vertex camera: the forbidden
volume between that camera's lens nose and the imaged region it must
see through the transparent platform. Any opaque part inside it
occludes part of the imaged region, which `khana check` is otherwise
blind to (`cad/CLAUDE.md` §"What a green `khana check` does not
mean", Visibility). This assertion is the gate for any further
turret Z compression (state file, Open work 3).

Geometry: an oblique frustum lofted from a small horizontal disc
just past the lens nose (entrance-pupil stand-in) to the imaged
square at the platform plane z = 0 — `sizing.py`'s
P_imaged = BRICK_MAX_MM + 2·SCATTER_HALF_MM (110 at the settled
3.6 mm lens), axis-aligned and centred on the brick centroid. The
square (not its circumscribed circle) is the real imaged region;
the circle would falsely collide with the platform frame's edge
grip 5 mm outside it.

Pose claim: asserted at the parked REST pose only. Capture happens
only during an IMAGE-station dwell with the rotor indexed, so
transit poses are out of scope; `frame_sweep.py` deliberately does
not run these checks (mid-transit platforms sweep through the cones
by design).

Only acrylic parts are exempt from the assertions — the below
sightlines pass through the transparent platform by design.
"""

from build123d import Circle, Pos, Rectangle, loft

from cad_khana.mechanism.assembly import Assembly

from .params import (
    VERTEX_BELOW_LEFT_POS,
    VERTEX_BELOW_RIGHT_POS,
    lens_axis_unit,
)
from .sizing import BRICK_MAX_MM, SCATTER_HALF_MM
from .camera_mount_esp32cam import assembly as _camera_mount

from ..design_systems import DEFAULT_SCHEMA
from .assembly import build_m03


SIGHTLINE_CONE_SPECS = {
    "sightline_below_left": VERTEX_BELOW_LEFT_POS,
    "sightline_below_right": VERTEX_BELOW_RIGHT_POS,
}

# Imaged-square side at the platform plane (110 at lens 3.6 / brick 80
# / scatter ±15). Live import from sizing.py — no manual mirror; the
# params.py vertex block is reconciled to the same inputs.
_SIGHTLINE_P_IMAGED = BRICK_MAX_MM + 2 * SCATTER_HALF_MM

# Entrance-pupil stand-in disc at the apex (a true M12 pupil at
# f/2.8 is ~Ø1.3; Ø4 adds margin), pushed a further 3 mm past the
# lens nose along the axis so the horizontal disc cannot poke back
# through the tilted lens front face (3 > disc radius ≥ worst-case
# back-reach).
_SIGHTLINE_APEX_D = 4.0
_SIGHTLINE_APEX_SETBACK = 3.0


def sightline_cone(vertex_pos: tuple[float, float, float]):
    """Oblique frustum from the camera's lens nose to the imaged
    square at the platform plane, in m03-local coordinates (place at
    identity). Lens nose = vertex (PCB rear face) + (PCB_T + M12_L)
    along the lens axis, mirroring `m12_lens_stub`'s extent."""
    axis = lens_axis_unit(vertex_pos)
    nose_dist = (
        _camera_mount.PCB_T + _camera_mount.M12_L + _SIGHTLINE_APEX_SETBACK
    )
    apex = tuple(
        vertex_pos[i] + nose_dist * axis[i] for i in range(3)
    )
    return loft(
        [
            Pos(*apex) * Circle(_SIGHTLINE_APEX_D / 2),
            Rectangle(_SIGHTLINE_P_IMAGED, _SIGHTLINE_P_IMAGED),
        ]
    )


# Known cone overlap (2026-07-24, first run of this assertion): the
# acrylic aperture equals the imaged square (ACRYLIC_INSERT_SIZE =
# IMAGED_REGION_SIDE_MM = 110), so each oblique below camera's
# sightline to its own near edge necessarily passes through the IMAGE
# cartridge frame below the platform plane — the perimeter wall band
# (|x| ∈ [55, 60], z ∈ [-4, 0]) plus the two near-side corner lips
# (z ∈ [-7, -4]). ≈958 mm³ per side. Optical cost: bottom-sliver
# vignetting of a brick parked at extreme near-edge scatter, and the
# opposite below camera sees that sliver unobstructed. Kept as a
# regression alarm, not fixed: growing the aperture past the imaged
# square or moving the lips inboard-out is a cartridge redesign the
# vignetting doesn't currently justify — if either lands, this
# assertion self-fails and must be removed.
_SIGHTLINE_KNOWN_OVERLAP_REASON = (
    "aperture == imaged square (110): below-* near-edge sightlines "
    "graze the IMAGE frame wall + corner lips below z=0; bottom-sliver "
    "vignetting covered by the opposite below camera"
)
_SIGHTLINE_KNOWN_OVERLAPS = frozenset({"turret.rotor.platform_image.frame"})


def add_sightline_cone_checks(a: Assembly) -> Assembly:
    """Add both below-vertex sightline cones and assert them clear of
    every m03 part except acrylics (transparent by design) — with the
    documented frame-graze pairs downgraded to `assert_interference`
    regression alarms."""
    # Opaque set fixed at entry — the cones added below must not land
    # in each other's check groups.
    opaque = [p.name for p in a.placed_parts if "acrylic" not in p.name]
    for cone_name, vertex_pos in SIGHTLINE_CONE_SPECS.items():
        a = a.with_part(cone_name, sightline_cone(vertex_pos))
        a = a.assert_no_interference_between(
            (cone_name,),
            opaque,
            known_overlaps=tuple(
                (cone_name, other, _SIGHTLINE_KNOWN_OVERLAP_REASON)
                for other in _SIGHTLINE_KNOWN_OVERLAPS
            ),
        )
    return a


def assembly(schema: str = DEFAULT_SCHEMA) -> Assembly:
    """The m03 assembly with both sightline cones and their clearance
    claims attached — the fixture this module exists to check.

    Builds the product rather than importing a prebuilt object, so the
    fixture is a function of its argument like every other factory
    here. The cone claims are geometric and schema-independent; the
    parameter exists so this fixture can be built alongside a
    particular palette rather than only the default.
    """
    return add_sightline_cone_checks(build_m03(schema))
