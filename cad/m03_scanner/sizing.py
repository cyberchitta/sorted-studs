"""
M03 scanner — viewpoint sizing.

Single source of truth for the scanner's optical / spatial sizing,
parameterised on a small set of real inputs (lens choice, largest
brick, landing scatter). Everything else — tetrahedral radius,
platform side, turret arm radius, rig-frame envelope — falls out of
those.

Run `.venv/bin/python cad/m03_scanner/sizing.py` to print the proposed
config + a lens-comparison sweep + a platform-sensitivity sweep.

Once a config is committed, the numbers below seed `camera_rig/params.py`
and `turret/params.py`. Manual copy today; possible future direct import.
"""

import math
from dataclasses import dataclass


# ============================================================================
# INPUTS — the only knobs that should normally change
# ============================================================================

LENS_FOCAL_MM       = 3.6     # M12; lens_sweep_table() compares alternatives
LENS_F_NUMBER       = 2.0     # for DoF
LENS_MIN_FOCUS_MM   = 120     # conservative; manual-focus M12s often beat

BRICK_MAX_MM        = 80.0    # largest dim of any sortable brick
BRICK_HEIGHT_MAX_MM = 16.0    # for DoF margin (e.g. 1×1 brick on side)
SCATTER_HALF_MM     = 15.0    # ± landing scatter on platform; placeholder

TILT_DEG            = 65.0    # apex (front_low) angle from platform normal
                              # 65° = rev-5 default (apex 25° elevation, -Y)

PX_PER_MM_TARGET    = 4.0     # 1 mm feature → 4 px on sensor


# ============================================================================
# FIXED HARDWARE — sensor / mount / extrusion
# ============================================================================

# OV2640 1/4" CMOS (AI-Thinker ESP32-CAM)
SENSOR_W_MM   = 3.6
SENSOR_H_MM   = 2.7
SENSOR_PX_W   = 1600
SENSOR_PX_H   = 1200
COC_MM        = 0.005

# ESP32-CAM mount plate (camera_rig/camera_mount_esp32cam)
MOUNT_PLATE_SIDE_MM = 50.0
MOUNT_PLATE_T_MM    = 5.0

# 2020 T-slot extrusion
RAIL_MEMBER_MM = 20.0
CORNER_GAP_MM  = 0.3

# Platform / turret clearances
PLATFORM_EDGE_GRIP_MM  = 5.0   # PMMA margin past imaged region for brackets
TURRET_PLATFORM_GAP_MM = 5.0   # gap between adjacent turret platforms
RAIL_D_FRONT_OVERHANG_MM = 5.0 # rail D extends past front-low cam by this much


# ============================================================================
# TETRAHEDRAL VERTICES (generated from tilt + phase)
# ============================================================================

def tetra_vertices_unit(tilt_deg=TILT_DEG, phase_deg=0.0):
    """4 vertices of a regular tetrahedron, unit distance from origin.

    `tilt_deg` is the angle of the apex (front_low) vertex from the
    platform normal (+Z). 0° → apex straight up. 65° → rev-5 default
    (apex at 25° elevation toward operator side, -Y).

    `phase_deg` rotates the 3 base vertices about the apex axis.
    phase=0° puts back_upper on the YZ plane (X=0), giving the
    operator-facing mirror symmetry across that plane that the rig's
    rail layout exploits. Don't change without reason.
    """
    s89 = math.sqrt(8 / 9)
    t = math.radians(tilt_deg)
    ct, st = math.cos(t), math.sin(t)

    def _tilt(v):
        x, y, z = v
        return (x, y * ct - z * st, y * st + z * ct)

    apex = _tilt((0.0, 0.0, 1.0))
    base = []
    for az_offset in (90.0, 210.0, 330.0):
        a = math.radians(az_offset + phase_deg)
        base.append(_tilt((math.cos(a) * s89, math.sin(a) * s89, -1 / 3)))
    return [
        ("front_low",   apex),
        ("back_upper",  base[0]),
        ("below_left",  base[1]),
        ("below_right", base[2]),
    ]


TETRA_VERTICES_UNIT = tetra_vertices_unit()


def _verify_rev5_reproduced():
    """Sanity: defaults (tilt 65°, phase 0°) approximately reproduce the
    rev-5 hardcoded vertices. Note the rev-5 values themselves are not
    an exact regular tetrahedron — front_low is at r ≈ 250, the others
    at r ≈ 246.6 — so this generator's exact-tetrahedron output diverges
    from rev-5 by up to ~3 mm. The discrepancy is within the spec's
    ±2 mm tolerance and goes away once params.py adopts these values."""
    rev5_r = 248
    expected = [
        (0.0, -226.6, 105.7),
        (0.0, 172.7, 176.0),
        (-201.4, 25.4, -140.1),
        (201.4, 25.4, -140.1),
    ]
    for (_name, unit_pos), exp in zip(TETRA_VERTICES_UNIT, expected):
        scaled = tuple(c * rev5_r for c in unit_pos)
        for s, e in zip(scaled, exp):
            assert abs(s - e) < 4.0, f"defaults far from rev-5: {scaled} vs {exp}"

_verify_rev5_reproduced()


# ============================================================================
# COMPUTATION HELPERS
# ============================================================================

def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def _dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def _mag(a):
    return math.sqrt(_dot(a, a))

def _unit(a):
    m = _mag(a)
    return tuple(c / m for c in a)


def hfov_vfov_deg(focal_mm):
    h = 2 * math.degrees(math.atan(SENSOR_W_MM / (2 * focal_mm)))
    v = 2 * math.degrees(math.atan(SENSOR_H_MM / (2 * focal_mm)))
    return h, v


def _camera_basis(camera_pos):
    axis = _unit(tuple(-c for c in camera_pos))
    for cand in [(0.0, 0.0, 1.0), (0.0, 1.0, 0.0)]:
        right_raw = _cross(cand, axis)
        if _mag(right_raw) > 0.1:
            break
    right = _unit(right_raw)
    up = _cross(axis, right)
    return axis, right, up


def _project_corners(camera_pos, p_imaged):
    axis, right, up = _camera_basis(camera_pos)
    h = p_imaged / 2
    out = []
    for corner in [(-h, -h, 0), (h, -h, 0), (h, h, 0), (-h, h, 0)]:
        v = tuple(corner[i] - camera_pos[i] for i in range(3))
        z = _dot(v, axis)
        if z <= 0:
            return None
        out.append((_dot(v, right) / z, _dot(v, up) / z))
    return out


def _fits_with_roll(projected, hfov_deg, vfov_deg):
    if projected is None:
        return False
    h_lim = math.tan(math.radians(hfov_deg) / 2)
    v_lim = math.tan(math.radians(vfov_deg) / 2)
    for theta_deg in range(0, 91, 2):
        t = math.radians(theta_deg)
        ct, st = math.cos(t), math.sin(t)
        rotated = [(u*ct - v*st, u*st + v*ct) for u, v in projected]
        umax = max(abs(p[0]) for p in rotated)
        vmax = max(abs(p[1]) for p in rotated)
        if (umax <= h_lim and vmax <= v_lim) or (umax <= v_lim and vmax <= h_lim):
            return True
    return False


def all_cameras_fit(r, p_imaged, focal_mm, vertices_unit=None):
    if vertices_unit is None:
        vertices_unit = TETRA_VERTICES_UNIT
    h, v = hfov_vfov_deg(focal_mm)
    for _name, unit_pos in vertices_unit:
        cam = tuple(c * r for c in unit_pos)
        if not _fits_with_roll(_project_corners(cam, p_imaged), h, v):
            return False
    return True


def min_r_for_fit(focal_mm, p_imaged, min_focus_mm, vertices_unit=None, r_max=400):
    if vertices_unit is None:
        vertices_unit = TETRA_VERTICES_UNIT
    start = max(1, int(math.ceil(min_focus_mm)))
    for r in range(start, r_max + 1):
        if all_cameras_fit(r, p_imaged, focal_mm, vertices_unit):
            return r
    return None


def px_per_mm_on_brick(r, focal_mm):
    pitch = SENSOR_W_MM / SENSOR_PX_W
    return focal_mm / r / pitch


def hyperfocal_mm(focal_mm, n=LENS_F_NUMBER, coc=COC_MM):
    return focal_mm * focal_mm / (n * coc)


def dof_half_mm(r, focal_mm, n=LENS_F_NUMBER, coc=COC_MM):
    H = hyperfocal_mm(focal_mm, n, coc)
    if r >= H:
        return (r, float("inf"))
    near = H * r / (H + r)
    far  = H * r / (H - r)
    return (r - near, far - r)


# ============================================================================
# DERIVED GEOMETRY
# ============================================================================

@dataclass
class Geometry:
    # Inputs (echoed)
    lens_focal_mm: float
    brick_max_mm: float
    scatter_half_mm: float
    tilt_deg: float

    # Optical
    hfov_deg: float
    vfov_deg: float
    px_per_mm: float
    dof_near_mm: float
    dof_far_mm: float

    # Spatial
    p_imaged_mm: float
    p_platform_mm: float
    tetra_r_mm: float
    vertices: dict[str, tuple[float, float, float]]

    # Turret
    turret_arm_r_mm: float

    # Rig envelope (rough — conservative mount-plate footprint)
    rail_a_z_mm: float
    rail_a_x_half_mm: float
    rail_c_y_mm: float
    rail_d_z_mm: float
    rail_d_y_front_mm: float


def compute_geometry(
    *,
    lens_focal_mm: float = LENS_FOCAL_MM,
    brick_max_mm: float = BRICK_MAX_MM,
    scatter_half_mm: float = SCATTER_HALF_MM,
    lens_min_focus_mm: float = LENS_MIN_FOCUS_MM,
    tilt_deg: float = TILT_DEG,
) -> Geometry:
    p_imaged = brick_max_mm + 2 * scatter_half_mm
    p_platform = p_imaged + 2 * PLATFORM_EDGE_GRIP_MM

    h, v = hfov_vfov_deg(lens_focal_mm)
    vertices_unit = tetra_vertices_unit(tilt_deg)
    r = min_r_for_fit(lens_focal_mm, p_imaged, lens_min_focus_mm, vertices_unit)
    if r is None:
        raise ValueError(
            f"No viable r for lens={lens_focal_mm} mm, P_imaged={p_imaged} mm, "
            f"tilt={tilt_deg}° (min_focus={lens_min_focus_mm} mm)"
        )

    ppm = px_per_mm_on_brick(r, lens_focal_mm)
    near, far = dof_half_mm(r, lens_focal_mm)

    vertices = {
        name: tuple(c * r for c in unit_pos)
        for name, unit_pos in vertices_unit
    }

    # Two adjacent square platforms (90° apart, edges aligned to local
    # radial frame) overlap iff turret arm radius R ≤ platform side.
    # Add a small gap so the platforms clear.
    arm_r = p_platform + TURRET_PLATFORM_GAP_MM

    # Rig envelope: each rail's constraining face sits one mount-rear-
    # offset + one extrusion-half-thickness + CORNER_GAP outside the
    # vertex it serves. Mount-rear-offset is the worst-case extent of
    # the 50×50×5 mount plate behind its vertex along any direction —
    # plate-thickness + half-side. Conservative; real geometry uses an
    # oriented bounding box and lands ~10–15% tighter.
    mount_offset = MOUNT_PLATE_T_MM + MOUNT_PLATE_SIDE_MM / 2
    rail_clear = mount_offset + RAIL_MEMBER_MM / 2 + CORNER_GAP_MM

    rail_a_z = vertices["below_left"][2] - rail_clear
    rail_a_x_half = abs(vertices["below_left"][0]) + mount_offset
    rail_c_y = vertices["back_upper"][1] + rail_clear
    rail_d_z = vertices["back_upper"][2] + rail_clear
    rail_d_y_front = vertices["front_low"][1] - RAIL_D_FRONT_OVERHANG_MM

    return Geometry(
        lens_focal_mm=lens_focal_mm,
        brick_max_mm=brick_max_mm,
        scatter_half_mm=scatter_half_mm,
        tilt_deg=tilt_deg,
        hfov_deg=h, vfov_deg=v,
        px_per_mm=ppm,
        dof_near_mm=near, dof_far_mm=far,
        p_imaged_mm=p_imaged, p_platform_mm=p_platform,
        tetra_r_mm=r, vertices=vertices,
        turret_arm_r_mm=arm_r,
        rail_a_z_mm=rail_a_z,
        rail_a_x_half_mm=rail_a_x_half,
        rail_c_y_mm=rail_c_y,
        rail_d_z_mm=rail_d_z,
        rail_d_y_front_mm=rail_d_y_front,
    )


# ============================================================================
# REPORTS
# ============================================================================

_REV5_BASELINE_R = 250  # rev-5 historical r, for comparison only


def print_config(g: Geometry):
    print(f"INPUTS")
    print(f"  Lens                   : {g.lens_focal_mm:.1f} mm M12")
    print(f"  Brick max dim          : {g.brick_max_mm:.0f} mm")
    print(f"  Landing scatter        : ±{g.scatter_half_mm:.0f} mm")
    print(f"  Tetrahedral tilt       : {g.tilt_deg:.0f}° from platform normal "
          f"(apex at {90 - g.tilt_deg:.0f}° elevation)")
    print()
    print(f"OPTICAL")
    print(f"  HFOV / VFOV            : {g.hfov_deg:.1f}° / {g.vfov_deg:.1f}°")
    print(f"  Resolution on brick    : {g.px_per_mm:.1f} px/mm "
          f"(target ≥ {PX_PER_MM_TARGET})")
    far = "∞" if g.dof_far_mm == float("inf") else f"+{g.dof_far_mm:.0f}"
    print(f"  DoF (f/{LENS_F_NUMBER:.1f}, c=5μm)   : -{g.dof_near_mm:.0f} / {far} mm")
    if g.dof_near_mm < BRICK_HEIGHT_MAX_MM / 2:
        print(f"  ⚠  near-DoF < brick half-height — re-check at this lens/r")
    print()
    print(f"SPATIAL")
    print(f"  Imaged region          : {g.p_imaged_mm:.0f} × {g.p_imaged_mm:.0f} mm")
    print(f"  Platform (PMMA)        : {g.p_platform_mm:.0f} × {g.p_platform_mm:.0f} mm")
    print(f"  Tetrahedral radius r   : {g.tetra_r_mm:.0f} mm  "
          f"(rev-5 baseline ≈ {_REV5_BASELINE_R})")
    for name, pos in g.vertices.items():
        print(f"    {name:<14}       ({pos[0]:>+7.1f}, {pos[1]:>+7.1f}, "
              f"{pos[2]:>+7.1f})")
    print()
    print(f"TURRET")
    print(f"  Arm radius             : {g.turret_arm_r_mm:.0f} mm  "
          f"(rev-4 = 185)")
    print(f"  IMAGE → LOAD station   : ({g.turret_arm_r_mm:.0f}, 0, "
          f"{-g.turret_arm_r_mm:.0f})")
    print(f"  IMAGE → DUMP station   : ({-g.turret_arm_r_mm:.0f}, 0, "
          f"{-g.turret_arm_r_mm:.0f})")
    print(f"  IMAGE → RETURN station : (0, 0, {-2*g.turret_arm_r_mm:.0f})")
    print()
    print(f"RIG ENVELOPE (rough; conservative mount footprint)")
    print(f"  Rail A (X-bar)         : z = {g.rail_a_z_mm:.0f}, "
          f"|x| ≤ {g.rail_a_x_half_mm:.0f}")
    print(f"  Rail C (Z-column)      : y = {g.rail_c_y_mm:.0f}")
    print(f"  Rail D (Y-cantilever)  : z = {g.rail_d_z_mm:.0f}, "
          f"y ≥ {g.rail_d_y_front_mm:.0f}")
    bb_x = 2 * g.rail_a_x_half_mm
    bb_y = g.rail_c_y_mm - g.rail_d_y_front_mm
    bb_z = g.rail_d_z_mm - g.rail_a_z_mm
    print(f"  Rig bounding box       : {bb_x:.0f} × {bb_y:.0f} × {bb_z:.0f}"
          f" mm (X×Y×Z)")


_LENS_CANDIDATES = [
    (2.1, "2.1mm wide", 100),
    (2.8, "2.8mm",      100),
    (3.6, "3.6mm",      120),
    (6.0, "6mm",        150),
    (8.0, "8mm",        200),
]


def lens_sweep_table():
    print(f"{'lens':<12} {'f':>4} {'HFOV':>6} {'VFOV':>6} "
          f"{'r':>4} {'P_pmma':>7} {'arm_R':>6} {'px/mm':>6} "
          f"{'rig BB X×Y×Z':>15}")
    print("-" * 78)
    for focal, label, min_focus in _LENS_CANDIDATES:
        try:
            g = compute_geometry(lens_focal_mm=focal, lens_min_focus_mm=min_focus)
        except ValueError:
            print(f"{label:<12} {focal:>4.1f}  NO FIT at default brick + scatter")
            continue
        bb_x = 2 * g.rail_a_x_half_mm
        bb_y = g.rail_c_y_mm - g.rail_d_y_front_mm
        bb_z = g.rail_d_z_mm - g.rail_a_z_mm
        print(f"{label:<12} {focal:>4.1f} {g.hfov_deg:>5.1f}° {g.vfov_deg:>5.1f}° "
              f"{g.tetra_r_mm:>4.0f} {g.p_platform_mm:>7.0f} "
              f"{g.turret_arm_r_mm:>6.0f} {g.px_per_mm:>6.1f} "
              f"{bb_x:>4.0f}×{bb_y:>3.0f}×{bb_z:>3.0f}")


def platform_sensitivity_table():
    print(f"How min r and turret arm R vary with brick_max + scatter "
          f"(P_imaged):")
    print()
    header = f"{'P_imaged':>9} {'arm_R':>6}"
    for _, label, _ in _LENS_CANDIDATES:
        header += f"  {label:>10}"
    print(header)
    print(f"{'(mm)':>9} {'(mm)':>6}", end="")
    for _ in _LENS_CANDIDATES:
        print(f"  {'r (mm)':>10}", end="")
    print()
    print("-" * len(header))
    for p in [60, 80, 100, 110, 120, 140, 160, 180]:
        platform_side = p + 2 * PLATFORM_EDGE_GRIP_MM
        arm_r = platform_side + TURRET_PLATFORM_GAP_MM
        row = f"{p:>9} {arm_r:>6.0f}"
        for focal, _, min_focus in _LENS_CANDIDATES:
            r = min_r_for_fit(focal, p, min_focus)
            row += f"  {(str(r) if r else '—'):>10}"
        print(row)


def tilt_sensitivity_table():
    print(f"How tilt affects r and rig BB at lens = {LENS_FOCAL_MM} mm, "
          f"P_imaged = {BRICK_MAX_MM + 2*SCATTER_HALF_MM:.0f} mm:")
    print()
    print(f"{'tilt':>5} {'apex_el':>8} {'r':>5} {'rig BB X×Y×Z':>16}")
    print("-" * 38)
    for tilt in [30, 40, 50, 55, 60, 65, 70, 75, 80]:
        try:
            g = compute_geometry(tilt_deg=tilt)
        except ValueError:
            print(f"{tilt:>4}°  NO FIT")
            continue
        bb_x = 2 * g.rail_a_x_half_mm
        bb_y = g.rail_c_y_mm - g.rail_d_y_front_mm
        bb_z = g.rail_d_z_mm - g.rail_a_z_mm
        elev = 90 - tilt
        print(f"{tilt:>4}° {elev:>6}° {g.tetra_r_mm:>5.0f} "
              f"{bb_x:>4.0f}×{bb_y:>3.0f}×{bb_z:>3.0f}")


def report():
    print("=" * 78)
    print(f"M03 SCANNER SIZING — proposed config (lens = {LENS_FOCAL_MM} mm, "
          f"tilt = {TILT_DEG:.0f}°)")
    print("=" * 78)
    print()
    print_config(compute_geometry())
    print()
    print("=" * 78)
    print("Lens comparison (each at its smallest viable r)")
    print("=" * 78)
    print()
    lens_sweep_table()
    print()
    print("=" * 78)
    print("Platform sensitivity (varying brick_max + scatter)")
    print("=" * 78)
    print()
    platform_sensitivity_table()
    print()
    print("=" * 78)
    print("Tilt sensitivity")
    print("=" * 78)
    print()
    tilt_sensitivity_table()


if __name__ == "__main__":
    report()
