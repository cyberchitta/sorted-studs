"""
M03 scanner — rig-intrusion visibility analysis.

Quantifies how much of each camera's frame is occupied by the
T-frame extrusion rails. Goal: minimise — not eliminate —
structural intrusion as we shrink the rig (smaller tetra radius,
shorter focal lengths). Output supports comparing (lens × tilt)
configs by intrusion cost.

Run:  .venv/bin/python cad/m03_scanner/visibility.py

Strategy
  1. `build_rig(g)` returns 3D line segments + 20 mm cross-section
     for rails A/B/C/D and brace_bc. Anchors come from `Geometry`
     (sizing.py); the only T-frame literal not exposed by Geometry —
     rail-A's Y position — is imported from
     `camera_rig.params.T_FRAME_RAIL_A_Y`. Other derived endpoints
     mirror the formulas in `skeleton_t_frame/assembly.py`.
  2. For each (camera, member): project both endpoints via
     `_camera_basis()`, near-clip at depth ≥ 5 mm (handles rails
     straddling the principal plane), normalise to FOV units, clip
     against the frame rectangle (expanded by projected half-width
     so edge-grazing rails register).
  3. Score: uniform area frac, centre-weighted area frac (Gaussian
     σ=0.4 → corner weight ≈ 1.9%; brick lives near frame centre,
     so edge intrusion is mostly cosmetic), centroid distance.
  4. Sweeps: 5 lens candidates @ default tilt, 5 tilts @ default
     lens, full 5×5 grid.

Coordinate frame: world Z-up, +Y away from operator.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))

from sizing import (  # noqa: E402
    _LENS_CANDIDATES,
    Geometry,
    _camera_basis,
    _dot,
    compute_geometry,
)
from camera_rig.params import T_FRAME_RAIL_A_Y  # noqa: E402


# ============================================================================
# RIG MODEL
# ============================================================================

MEMBER_MM = 20.0       # 2020 extrusion cross-section
CORNER_GAP_MM = 0.3
_BRACE_LEG_MM = 60.0
_RAIL_B_BACK_PAST_C_MM = 40.0


@dataclass
class RigMember:
    name: str
    p_start: tuple[float, float, float]
    p_end: tuple[float, float, float]
    cross_section_mm: float = MEMBER_MM


def build_rig(g: Geometry) -> list[RigMember]:
    """Reconstruct the T-frame as 3D segments. Mirrors the formulas
    in `skeleton_t_frame/assembly.py`; if those constants drift,
    update both."""
    a_y = T_FRAME_RAIL_A_Y
    a_z = g.rail_a_z_mm
    c_y = g.rail_c_y_mm
    d_z = g.rail_d_z_mm
    d_y_front = g.rail_d_y_front_mm

    b_y_front = a_y + MEMBER_MM / 2 + CORNER_GAP_MM
    b_y_back = c_y + _RAIL_B_BACK_PAST_C_MM
    c_z_bottom = a_z + MEMBER_MM / 2 + CORNER_GAP_MM
    c_z_top = d_z + MEMBER_MM / 2          # rail C top end-face flush with rail D top side face
    d_y_back = c_y - MEMBER_MM / 2 - CORNER_GAP_MM

    # brace_bc: rail B +Z face → rail C -Y face (60 mm legs).
    brace_p1 = (0.0, c_y - MEMBER_MM / 2 - _BRACE_LEG_MM, a_z + MEMBER_MM / 2)
    brace_p2 = (0.0, c_y - MEMBER_MM / 2, a_z + MEMBER_MM / 2 + _BRACE_LEG_MM)

    return [
        RigMember("rail_a",
                  (-g.rail_a_x_half_mm, a_y, a_z),
                  (+g.rail_a_x_half_mm, a_y, a_z)),
        RigMember("rail_b", (0.0, b_y_front, a_z), (0.0, b_y_back, a_z)),
        RigMember("rail_c", (0.0, c_y, c_z_bottom), (0.0, c_y, c_z_top)),
        RigMember("rail_d", (0.0, d_y_front, d_z), (0.0, d_y_back, d_z)),
        RigMember("brace_bc", brace_p1, brace_p2),
    ]


# ============================================================================
# PROJECTION + INTRUSION SCORING
# ============================================================================

_NEAR_MM = 5.0   # near-plane clip depth (segments straddling depth=0)
_SIGMA = 0.4     # Gaussian centre-weight σ (corner weight ≈ 1.9%)


def _project_point(p_world, cam_pos, axis, right, up):
    v = (p_world[0] - cam_pos[0],
         p_world[1] - cam_pos[1],
         p_world[2] - cam_pos[2])
    return _dot(v, axis), _dot(v, right), _dot(v, up)


def _clip_segment_to_near(z1, x1, y1, z2, x2, y2, near):
    """Clip 3D segment to depth ≥ near. Returns ((z,x,y), (z,x,y)) or
    (None, None) if entirely behind."""
    if z1 < near and z2 < near:
        return None, None
    if z1 >= near and z2 >= near:
        return (z1, x1, y1), (z2, x2, y2)
    t = (near - z1) / (z2 - z1)
    xc = x1 + t * (x2 - x1)
    yc = y1 + t * (y2 - y1)
    if z1 < near:
        return (near, xc, yc), (z2, x2, y2)
    return (z1, x1, y1), (near, xc, yc)


def _liang_barsky_clip(u1, v1, u2, v2, umin, umax, vmin, vmax):
    """Clip 2D segment to rectangle. Returns (u1,v1,u2,v2) or None."""
    dx, dy = u2 - u1, v2 - v1
    p = [-dx, dx, -dy, dy]
    q = [u1 - umin, umax - u1, v1 - vmin, vmax - v1]
    t0, t1 = 0.0, 1.0
    for pi, qi in zip(p, q):
        if pi == 0:
            if qi < 0:
                return None
            continue
        t = qi / pi
        if pi < 0:
            if t > t1:
                return None
            t0 = max(t0, t)
        else:
            if t < t0:
                return None
            t1 = min(t1, t)
    return u1 + t0 * dx, v1 + t0 * dy, u1 + t1 * dx, v1 + t1 * dy


def _project_member(member: RigMember, cam_pos, axis, right, up,
                    h_lim, v_lim):
    """Project a 3D segment to clipped image-plane coords.

    Returns a dict {u1,v1,u2,v2,w1,w2} in normalised-FOV units (frame
    is [-1,+1]²) where w_i is the projected half-width at endpoint i,
    or None if the member is entirely behind / outside the frame
    (including its projected width margin)."""
    z1, x1u, y1u = _project_point(member.p_start, cam_pos, axis, right, up)
    z2, x2u, y2u = _project_point(member.p_end,   cam_pos, axis, right, up)

    p1, p2 = _clip_segment_to_near(z1, x1u, y1u, z2, x2u, y2u, _NEAR_MM)
    if p1 is None:
        return None
    z1, x1u, y1u = p1
    z2, x2u, y2u = p2

    half_cs = member.cross_section_mm / 2
    # Endpoints in normalised-FOV coords (1.0 = frame edge).
    u1n, v1n = (x1u / z1) / h_lim, (y1u / z1) / v_lim
    u2n, v2n = (x2u / z2) / h_lim, (y2u / z2) / v_lim
    # Projected half-widths in normalised-FOV (use h_lim — we
    # aggregate area, so direction-agnostic is fine).
    w1 = half_cs / z1 / h_lim
    w2 = half_cs / z2 / h_lim
    wmax = max(w1, w2)

    # Bbox reject.
    if (max(u1n, u2n) < -1 - wmax or min(u1n, u2n) > 1 + wmax
            or max(v1n, v2n) < -1 - wmax or min(v1n, v2n) > 1 + wmax):
        return None

    # Clip centerline against frame *expanded by* projected width so
    # edge-grazing rails register; the per-sample integrator below
    # clamps each sample's effective area to the actual frame.
    clipped = _liang_barsky_clip(u1n, v1n, u2n, v2n,
                                 -1 - wmax, 1 + wmax,
                                 -1 - wmax, 1 + wmax)
    if clipped is None:
        return None
    cu1, cv1, cu2, cv2 = clipped

    # Recover depth at the clipped endpoints by parametrising t on
    # the post-near-clip segment (whichever axis has more spread).
    du, dv = u2n - u1n, v2n - v1n
    if abs(du) >= abs(dv):
        t_a = (cu1 - u1n) / du if du != 0 else 0.0
        t_b = (cu2 - u1n) / du if du != 0 else 0.0
    else:
        t_a = (cv1 - v1n) / dv if dv != 0 else 0.0
        t_b = (cv2 - v1n) / dv if dv != 0 else 0.0
    t_a = max(0.0, min(1.0, t_a))
    t_b = max(0.0, min(1.0, t_b))
    z_a = z1 + t_a * (z2 - z1)
    z_b = z1 + t_b * (z2 - z1)
    return dict(u1=cu1, v1=cv1, u2=cu2, v2=cv2,
                w1=half_cs / z_a / h_lim,
                w2=half_cs / z_b / h_lim)


def _intrusion_metrics(proj):
    """Sample-integrate occupancy along the clipped segment.

    Each of N samples represents a chunk of length L/N and width
    2·ww; we clip its bbox to [-1, 1]² so a sample whose centre is
    just outside frame still contributes the in-frame fraction of
    its width. Returns (uniform_frac, weighted_frac, centroid_dist)
    — uniform/weighted as fractions of the 2×2 frame area."""
    u1, v1, u2, v2 = proj["u1"], proj["v1"], proj["u2"], proj["v2"]
    w1, w2 = proj["w1"], proj["w2"]
    L = math.hypot(u2 - u1, v2 - v1)

    N = 32
    inv_2sigma2 = 1.0 / (2 * _SIGMA * _SIGMA)
    uniform_acc = 0.0
    weighted_acc = 0.0
    cx = cy = 0.0
    in_frame_count = 0

    for i in range(N):
        t = (i + 0.5) / N
        uu = u1 + t * (u2 - u1)
        vv = v1 + t * (v2 - v1)
        ww = w1 + t * (w2 - w1)
        half_along = L / N / 2
        half_box = max(half_along, ww)
        clip_u_lo = max(-1.0, uu - half_box)
        clip_u_hi = min(+1.0, uu + half_box)
        clip_v_lo = max(-1.0, vv - half_box)
        clip_v_hi = min(+1.0, vv + half_box)
        if clip_u_hi <= clip_u_lo or clip_v_hi <= clip_v_lo:
            continue
        chunk_area = (L / N) * (2 * ww)
        bbox_area = (2 * half_box) ** 2
        in_frame_frac = ((clip_u_hi - clip_u_lo)
                         * (clip_v_hi - clip_v_lo)) / bbox_area
        eff_area = chunk_area * in_frame_frac
        weight = math.exp(-(uu * uu + vv * vv) * inv_2sigma2)
        uniform_acc += eff_area
        weighted_acc += weight * eff_area
        if -1.0 <= uu <= 1.0 and -1.0 <= vv <= 1.0:
            cx += uu
            cy += vv
            in_frame_count += 1

    uniform_frac = uniform_acc / 4.0
    weighted_frac = weighted_acc / 4.0
    if in_frame_count:
        centroid_dist = math.hypot(cx / in_frame_count, cy / in_frame_count)
    else:
        centroid_dist = math.inf
    return uniform_frac, weighted_frac, centroid_dist


def per_camera_intrusion(g: Geometry):
    """{cam_name: {member_name: (uniform, weighted, centroid_dist)}}."""
    rig = build_rig(g)
    h_lim = math.tan(math.radians(g.hfov_deg) / 2)
    v_lim = math.tan(math.radians(g.vfov_deg) / 2)
    out = {}
    for cam_name, cam_pos in g.vertices.items():
        axis, right, up = _camera_basis(cam_pos)
        per_member = {}
        for m in rig:
            proj = _project_member(m, cam_pos, axis, right, up, h_lim, v_lim)
            if proj is None:
                per_member[m.name] = (0.0, 0.0, math.inf)
            else:
                per_member[m.name] = _intrusion_metrics(proj)
        out[cam_name] = per_member
    return out


def aggregate(per_cam):
    out = {}
    for cam, members in per_cam.items():
        tot_u = sum(u for (u, _, _) in members.values())
        tot_w = sum(w for (_, w, _) in members.values())
        worst_name, worst_w = "", 0.0
        for name, (_, w, _d) in members.items():
            if w > worst_w:
                worst_name, worst_w = name, w
        out[cam] = dict(total_uniform=tot_u, total_weighted=tot_w,
                        worst_name=worst_name, worst_weighted=worst_w)
    return out


# ============================================================================
# REPORTS
# ============================================================================

CAM_ORDER = ["front_low", "back_upper", "below_left", "below_right"]
TILTS = (30, 40, 50, 65, 80)


def print_default_table(g: Geometry):
    agg = aggregate(per_camera_intrusion(g))
    print(f"Default config: lens={g.lens_focal_mm} mm, tilt={g.tilt_deg:.0f}°, "
          f"r={g.tetra_r_mm:.0f} mm, HFOV={g.hfov_deg:.1f}°")
    print()
    print(f"{'camera':<13} {'uniform%':>9} {'weighted%':>10} "
          f"{'worst member':>14} {'worst w%':>10}")
    print("-" * 62)
    for cam in CAM_ORDER:
        a = agg[cam]
        print(f"{cam:<13} {100*a['total_uniform']:>9.2f} "
              f"{100*a['total_weighted']:>10.2f} "
              f"{a['worst_name']:>14} "
              f"{100*a['worst_weighted']:>10.2f}")


def _sweep_table(rows, label_col_w, first_col_header):
    """rows: iterable of (label, agg-or-None). Prints per-camera
    weighted % and rig-wide max."""
    head = f"{first_col_header:<{label_col_w}}"
    for cam in CAM_ORDER:
        head += f" {cam:>11}"
    head += f" {'rig max':>9}"
    print(head)
    print("-" * len(head))
    for label, agg in rows:
        if agg is None:
            print(f"{label:<{label_col_w}}  no fit")
            continue
        row = f"{label:<{label_col_w}}"
        rig_max = 0.0
        for cam in CAM_ORDER:
            w = agg[cam]['total_weighted']
            row += f" {100*w:>10.2f}%"
            rig_max = max(rig_max, w)
        row += f" {100*rig_max:>8.2f}%"
        print(row)


def _try_geom(focal_mm, min_focus_mm, tilt_deg):
    try:
        return compute_geometry(lens_focal_mm=focal_mm,
                                lens_min_focus_mm=min_focus_mm,
                                tilt_deg=tilt_deg)
    except ValueError:
        return None


def lens_sweep_table(tilt_deg):
    print(f"Lens sweep at tilt={tilt_deg:.0f}° "
          f"(per-camera weighted intrusion %, then rig-wide max):")
    print()
    rows = []
    for focal, label, min_focus in _LENS_CANDIDATES:
        g = _try_geom(focal, min_focus, tilt_deg)
        rows.append((label, aggregate(per_camera_intrusion(g)) if g else None))
    _sweep_table(rows, 10, "lens")


def tilt_sweep_table(focal_mm, min_focus_mm):
    print(f"Tilt sweep at lens={focal_mm} mm "
          f"(per-camera weighted intrusion %, then rig-wide max):")
    print()
    rows = []
    for tilt in TILTS:
        g = _try_geom(focal_mm, min_focus_mm, tilt)
        rows.append((f"{tilt}°",
                     aggregate(per_camera_intrusion(g)) if g else None))
    _sweep_table(rows, 6, "tilt")


def lens_tilt_grid():
    print("(lens × tilt) grid of rig-wide max weighted intrusion %:")
    print("(lower is better; '*' marks the minimum cell)")
    print()
    head = f"{'lens \\ tilt':<11}"
    for tilt in TILTS:
        head += f" {tilt:>6}°"
    print(head)
    print("-" * len(head))

    grid: dict[tuple[float, int], float | None] = {}
    for focal, _label, min_focus in _LENS_CANDIDATES:
        for tilt in TILTS:
            try:
                g = compute_geometry(lens_focal_mm=focal,
                                     lens_min_focus_mm=min_focus,
                                     tilt_deg=tilt)
            except ValueError:
                grid[(focal, tilt)] = None
                continue
            agg = aggregate(per_camera_intrusion(g))
            grid[(focal, tilt)] = max(a['total_weighted']
                                      for a in agg.values())
    valid = [(k, v) for k, v in grid.items() if v is not None]
    min_key = min(valid, key=lambda kv: kv[1])[0] if valid else None

    for focal, label, _ in _LENS_CANDIDATES:
        row = f"{label:<11}"
        for tilt in TILTS:
            v = grid[(focal, tilt)]
            if v is None:
                row += f" {'  —  ':>7}"
            else:
                marker = "*" if (focal, tilt) == min_key else " "
                row += f" {100*v:>5.2f}%{marker}"
        print(row)
    if min_key:
        f, t = min_key
        print()
        print(f"Minimum cell: lens={f} mm, tilt={t}° → "
              f"{100*grid[min_key]:.2f}% rig-wide max weighted intrusion.")
    return min_key, grid


def interpretation(default_g: Geometry, grid_min_key, grid):
    per_cam = per_camera_intrusion(default_g)
    rig_max = max(a['total_weighted']
                  for a in aggregate(per_cam).values())
    score = {}
    for members in per_cam.values():
        for name, (_, w, _d) in members.items():
            score[name] = score.get(name, 0.0) + w
    consistent = max(score.items(), key=lambda kv: kv[1])

    print()
    print("INTERPRETATION")
    if grid_min_key:
        f, t = grid_min_key
        v = grid[grid_min_key]
        delta_str = ("comparable" if abs(v - rig_max) < 0.005
                     else f"{100*(rig_max - v):+.2f}% worse")
        print(f"  Cleanest grid cell: lens={f} mm @ tilt={t}° "
              f"({100*v:.2f}% rig-wide max). Default "
              f"(lens={default_g.lens_focal_mm} mm @ {default_g.tilt_deg:.0f}°) "
              f"= {100*rig_max:.2f}% — {delta_str}.")
    print(f"  Most consistent offender (sum of weighted % across cameras): "
          f"'{consistent[0]}' = {100*consistent[1]:.2f}%. "
          f"Redesign target if intrusion budget bites.")
    if 0.05 <= rig_max <= 0.25:
        verdict = "reasonable"
    elif rig_max < 0.05:
        verdict = "very clean — frames near-empty of rig structure"
    else:
        verdict = "high — re-evaluate rail layout or lens choice"
    print(f"  Default rig-wide max = {100*rig_max:.2f}%: {verdict}.")


# ============================================================================
# MAIN
# ============================================================================

def report():
    from sizing import LENS_FOCAL_MM, LENS_MIN_FOCUS_MM, TILT_DEG
    print("=" * 78)
    print("M03 SCANNER VISIBILITY — rig intrusion in camera frames")
    print("=" * 78)
    print()
    g = compute_geometry()
    print_default_table(g)
    print()
    print("=" * 78)
    print("Lens sweep (default tilt)")
    print("=" * 78)
    print()
    lens_sweep_table(TILT_DEG)
    print()
    print("=" * 78)
    print("Tilt sweep (default lens)")
    print("=" * 78)
    print()
    tilt_sweep_table(LENS_FOCAL_MM, LENS_MIN_FOCUS_MM)
    print()
    print("=" * 78)
    print("Lens × tilt grid")
    print("=" * 78)
    print()
    min_key, grid = lens_tilt_grid()
    print()
    print("=" * 78)
    interpretation(g, min_key, grid)


if __name__ == "__main__":
    report()
