"""
M02 C-channel singulator — rotor sub-assembly (hollow shell).

Crown-cut faceted frustum with a matched-taper interior cavity, on
a flange below the base rim that is the rotor's bearing seat and
ring gear (rim-supported, rim-driven — `_notes/decisions/
m02-rotor-drive.md`; the drive itself is ../drive/assembly.py).

Geometry:
    - Outer: lofted shell through three cross-sections —
        z=0                 : circle, r = base_r (rim stays true-round
                              so the rotor-stator gap is constant
                              regardless of rotation angle).
        z=KINK_Z_FRAC·H     : 20-pointed star — 10 "peak" points
                              aligned with top-polygon vertices,
                              10 "valley" points pulled radially
                              inward between them. Creates the
                              crown-cut zigzag ridges.
        z=H                 : 10-sided regular polygon, r = top_r.
    - Shell: ROTOR_SHELL_WALL_T radial wall from z=0 up to z = height
      − top_cap_t (cavity opens here). Cavity is a simple tapered
      10-gon sized to stay inside the narrowest outer section.
    - Top cap: solid ROTOR_TOP_CAP_T thick above the cavity.
    - Flange: annulus z ∈ [-ROTOR_FLANGE_T, 0], r ∈ [ROTOR_FLANGE_R_IN,
      RING_PITCH_D/2]. Its OD is the ring gear's pitch cylinder
      (= the base rim radius, so the rim continues straight down);
      the teeth are a detail-layer concern, as on the m05 base
      floor. Its underside seats the turntable bearing's upper ring
      (r ∈ [BEARING_ID/2, BEARING_OD/2]) and is the bed face when
      printing.

Outside-ride still applies: bricks sit in the V between the rotor
flank and the stator outer wall; nothing about the hollow interior
changes that contact geometry.

Printability note: prints base-down — the flange's flat underside is
the bed face and the ring teeth print in the first layers, which is
where the X1C is most accurate. The cavity stays open at the base
(through the flange's Ø150 bore) so the breakaway support under the
90° cavity ceiling can come out; closing the base would seal that
support in. Ø240 plus the m1.5 addendum is Ø243 at the tips, inside
the 256 mm bed.

Coordinate frame:
    origin = rotor axis ∩ base plane (widest section, at the
             V-bottom level of the stator)
    +Z     = up (top-platform / drop-surface direction)
    z=0    = rotor base plane (= stator V-bottom)
"""

import math

from build123d import (
    Circle,
    Part,
    Plane,
    Polygon,
    Pos,
    RegularPolygon,
    extrude,
    loft,
)

from cad_khana.mechanism.assembly import Assembly

from ..materials import material_of
from ..params import (
    RING_PITCH_D,
    ROTOR_BASE_DIAMETER,
    ROTOR_FACET_COUNT,
    ROTOR_FACET_KINK_Z_FRAC,
    ROTOR_FACET_PEAK_BULGE,
    ROTOR_FACET_VALLEY_DENT,
    ROTOR_FLANGE_R_IN,
    ROTOR_FLANGE_T,
    ROTOR_HEIGHT,
    ROTOR_SHELL_WALL_T,
    ROTOR_TOP_CAP_T,
    ROTOR_TOP_DIAMETER,
    V_HALF_ANGLE_DEG,
)


WALL_MIN = 1.5


# --- Part functions ----------------------------------------------------

def rotor(
    *,
    base_diameter: float = ROTOR_BASE_DIAMETER,
    top_diameter: float = ROTOR_TOP_DIAMETER,
    height: float = ROTOR_HEIGHT,
    n_facets: int = ROTOR_FACET_COUNT,
    flank_angle_deg: float = V_HALF_ANGLE_DEG,
    kink_z_frac: float = ROTOR_FACET_KINK_Z_FRAC,
    peak_bulge: float = ROTOR_FACET_PEAK_BULGE,
    valley_dent: float = ROTOR_FACET_VALLEY_DENT,
    shell_wall_t: float = ROTOR_SHELL_WALL_T,
    top_cap_t: float = ROTOR_TOP_CAP_T,
    flange_t: float = ROTOR_FLANGE_T,
    flange_r_in: float = ROTOR_FLANGE_R_IN,
    flange_r_out: float = RING_PITCH_D / 2.0,
) -> Part:
    """Hollow crown-cut faceted frustum with top cap, on a ring-gear
    flange below the base rim. Base rim at z=0, apex direction +Z,
    flange below z=0.

    Outer shell lofts through three cross-sections (circle → 20-point
    star → 10-gon). The star's "peak" points align with top-polygon
    vertices, producing ridges that extend from top to rim; "valley"
    points between them produce the inset triangular sub-facets.

    Build order:
      outer − cavity  →  shell + solid top cap (annulus below cap,
                         full section above).
      + flange        →  annulus under the rim; fuses to the shell's
                         base edge (shell r ∈ [~117.5, 120] at z=0
                         lies inside the flange's radial span).
    The cavity is cut before the flange is added so its 1 mm
    bottom pad cannot notch the flange's top face.
    """
    # Guards the arguments: the flange must reach the shell's base
    # edge or the two bodies never fuse, and the bearing seat needs
    # the land inside its ID.
    assert flange_r_out >= base_diameter / 2.0 - shell_wall_t, (
        f"flange_r_out ({flange_r_out}) must reach the shell base edge"
    )
    assert 0.0 < flange_r_in < flange_r_out
    base_r = base_diameter / 2.0
    top_r = top_diameter / 2.0
    tan_flank = math.tan(math.radians(flank_angle_deg))

    # Kink cross-section: 20-point star at z = kink_z. Peaks align
    # with top-polygon vertices (ridges visible in the render);
    # valleys sit on the mid-angles between them.
    kink_z = height * kink_z_frac
    nominal_r_at_kink = base_r - kink_z * tan_flank
    r_peak = nominal_r_at_kink + peak_bulge
    r_valley = nominal_r_at_kink - valley_dent

    # Top polygon described as a 20-vertex section (10 real polygon
    # vertices at r=top_r + 10 edge-midpoints at the apothem radius)
    # so its vertex count matches the kink section. The shape is
    # still a perfect regular 10-gon — the extra 10 points lie
    # exactly on its edges — but the 20-to-20 vertex loft avoids
    # degenerate triangle collapses at the valleys.
    top_apothem = top_r * math.cos(math.pi / n_facets)
    step = math.pi / n_facets  # 18° at N=10

    star_pts: list[tuple[float, float]] = []
    top_pts: list[tuple[float, float]] = []
    for i in range(2 * n_facets):
        theta = i * step
        is_peak = (i % 2 == 0)
        r_s = r_peak if is_peak else r_valley
        r_t = top_r if is_peak else top_apothem
        star_pts.append((r_s * math.cos(theta), r_s * math.sin(theta)))
        top_pts.append((r_t * math.cos(theta), r_t * math.sin(theta)))

    outer = loft(
        [
            Plane.XY * Circle(base_r),
            Plane.XY.offset(kink_z) * Polygon(*star_pts, align=None),
            Plane.XY.offset(height) * Polygon(*top_pts, align=None),
        ],
        ruled=True,
    )

    # Simple tapered 10-gon cavity. Its vertices align angularly
    # with outer peaks (both at angles 0, 36°, …), so its apothem
    # faces the outer valleys. Size it so the wall is ≥ shell_wall_t
    # at *both* angular positions, with the kink height being the
    # tightest section:
    #   wall at peak   = r_peak   − cavity_r          ≥ shell_wall_t
    #   wall at valley = r_valley − cavity_r·cos(π/N) ≥ shell_wall_t
    cos_apothem = math.cos(math.pi / n_facets)
    cavity_r_at_kink = min(
        r_peak - shell_wall_t,
        (r_valley - shell_wall_t) / cos_apothem,
    )
    # Tapered cavity shares the flank angle so wall thickness tracks
    # the outer taper; base radius is back-derived from the kink.
    cavity_base_r = cavity_r_at_kink + kink_z * tan_flank
    cavity_bottom_pad = 1.0
    cavity_height = (height - top_cap_t) + cavity_bottom_pad
    cavity = Pos(0, 0, -cavity_bottom_pad) * extrude(
        RegularPolygon(radius=cavity_base_r, side_count=n_facets),
        amount=cavity_height,
        taper=flank_angle_deg,
    )

    # Flange: ring-gear pitch annulus under the rim, top face at z=0.
    flange = Pos(0, 0, -flange_t) * extrude(
        Circle(flange_r_out) - Circle(flange_r_in), amount=flange_t,
    )

    return (outer - cavity) + flange


# --- Assembly ----------------------------------------------------------

def build_rotor() -> Assembly:
    """Rotor sub-assembly in rotor-local frame (= m02-local: origin on
    the rotation axis at the base plane). Single part; the parent
    composes it as the ``rotor`` subtree and owns the spin DOF."""
    return Assembly().with_part(
        "rotor", rotor(), material=material_of("rotor"),
    )


assembly = build_rotor()
