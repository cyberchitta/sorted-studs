"""Thirds seams: split a Ø310 ring part into sectors that fit the bed.

The bowl and the stator liner both reach r 155 — past the X1C's 256 mm
bed whole — so each prints as SEAM_COUNT sectors (owner 2026-09-13;
`_notes/state/m02-singulator.md`). A 120° sector of r 155 fits a
195 mm square; with the ears below, ≈ 206.

Joint (decided in the seams slice, 2026-09-17): **bolted radial ears,
outside the ring.** At each seam both sectors carry a flat tab on
their own side of the seam plane, from just inside the wall's outer
face out to r + SEAM_EAR_OUT, so the two tabs meet face-to-face on the
seam plane and a tangential bolt pulls them together. Outside, not
inside: inside the bowl the rotor flange, bearing and pinion fill the
annulus, and inside the liner is the brick surface. Not dovetails: a
dovetail in a 3 mm wall has nothing to bear on. The floor's radial
seam inboard of the skirt is spanned by the turntable bearing's lower
ring, which bolts to the floor on both sides of it (detail layer).
The bolt holes and bolts are detail-layer too — the cheap ear is
solid.

Sector k spans [phase + k·360/n, phase + (k+1)·360/n] in m02-local
degrees (0 = the exit arc's centre). The phase is a per-instance
choice, like the motor azimuth: the seam lines must miss the exit
arc, the motor, and wherever a gantry bar ends under the floor — so
the chain picks it per stage (`cad/m02_chain/params.py`
§Seams).
"""

from __future__ import annotations

import math

from build123d import Box, Cylinder, Part, Pos, Rot


SEAM_COUNT = 3
SEAM_EAR_T = 5.0         # tangential thickness of each tab (a pair is 10)
SEAM_EAR_OUT = 12.0      # radial reach past the ring's outer face
SEAM_EAR_OVERLAP = 0.5   # tab starts this far inside the outer face, so it fuses


def seam_azimuths(phase_deg: float, n: int = SEAM_COUNT) -> tuple[float, ...]:
    """The n seam azimuths, degrees in [-180, 180)."""
    return tuple(_wrap(phase_deg + k * 360.0 / n) for k in range(n))


def sector_index(az_deg: float, phase_deg: float, n: int = SEAM_COUNT) -> int:
    """Which sector contains azimuth ``az_deg``."""
    return int(((az_deg - phase_deg) % 360.0) // (360.0 / n))


def angular_gap_deg(a: float, b: float) -> float:
    """Unsigned angle between two azimuths, degrees in [0, 180)."""
    return abs(_wrap(a - b))


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def split_ring(
    ring: Part,
    *,
    phase_deg: float,
    r_out: float,
    ear_z: tuple[float, float],
    n: int = SEAM_COUNT,
    ear_t: float = SEAM_EAR_T,
    ear_out: float = SEAM_EAR_OUT,
) -> list[Part]:
    """Cut ``ring`` (a solid of revolution about +Z, outer radius
    ``r_out``) into ``n`` sectors at ``phase_deg``, each carrying an
    ear at both of its seams over ``ear_z`` = (z0, z1)."""
    span = 360.0 / n
    assert 0.0 < ear_t and 0.0 < ear_out and ear_z[0] < ear_z[1]
    # The ear must lie inside its own sector's angle at the ring's
    # outer face, or it would cross into the neighbour.
    assert math.degrees(ear_t / r_out) < span / 2.0
    big_r = r_out + ear_out + 50.0
    z_lo, z_hi = -1000.0, 1000.0
    r0, r1 = r_out - SEAM_EAR_OVERLAP, r_out + ear_out
    sectors = []
    for k in range(n):
        a0 = phase_deg + k * span
        wedge = Rot(0, 0, a0) * Pos(0, 0, z_lo) * Cylinder(
            big_r, z_hi - z_lo, arc_size=span, align=None,
        )
        body = ring & wedge
        ear = Box(r1 - r0, ear_t, ear_z[1] - ear_z[0], align=None)
        # Start seam: the tab lies on the +tangential side (inside the
        # sector); end seam: on the −tangential side.
        body += Rot(0, 0, a0) * Pos(r0, 0.0, ear_z[0]) * ear
        body += Rot(0, 0, a0 + span) * Pos(r0, -ear_t, ear_z[0]) * ear
        sectors.append(body)
    return sectors
