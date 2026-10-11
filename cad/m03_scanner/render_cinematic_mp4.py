"""M03 scanner — cinematic mp4 of the animated rig.

Three named camera moves (pick via ``CINE_CAMERA``) drive
cad_khana.render.animation's two entry points. Each move stays closer to
the rig than the abandoned wide-orbital first pass — the rig is
cubical, so wide framings leave dead background space.

All moves are framed around the DUMP zone (world heading 270°)
because the platform-tip is the storytelling beat. Tip peaks at
``t ∈ {0.0875, 0.3375, 0.5875, 0.8375}`` (4 dumps over 8 seconds).

Camera azimuth ≈ 240° (3/4 view from the -X side) sees the DUMP
platform in profile and the bench-mounted servo arm edge-on. The
earlier straight-on -Y view (az=275°) put the servo arm directly
between camera and platform, occluding the tip.

All moves keep the camera at elevation ≈ 0° (horizontal) so the
studio HDRI's dark backdrop sits behind the rig in frame instead
of the bright upper softbox region. An earlier set of looking-down
poses (EL=22°–35°) framed the floor of the HDRI dome above the
subject's horizon and washed the background out.

Camera moves:
  * ``dump_static``   — Camera at az≈240°, EL=0°, D=350 targeting
    DUMP zone, with ±10° azimuth drift. All 4 dumps play out in
    profile in front of a near-static camera.
  * ``dump_approach`` — Same DUMP-centered framing but pushes from
    D=500 → D=250 across the clip. Final dump lands at the closest
    framing — punchline-style.
  * ``dump_intro``    — Opens looking edge-on at the rig from
    az≈135° (back-left), then sweeps horizontally around to settle
    into the dump_static pose at az≈240°. Single smooth move
    interpolating camera location and target at fixed elevation.

Target medium: Twitter/X mobile feed. 720×720 square at 24 fps —
the cubical rig wastes pixels in 9:16 vertical at wide framings;
square fills consistently across any move.

Env knobs:
  CINE_MODE      — "both" (default) | "photoreal" | "fromglb"
  CINE_CAMERA    — "dump_intro" (default) | "dump_static" | "dump_approach" | "all"
  CINE_RES       — "720x720" (default) | "480x480" (draft)
  CINE_DURATION  — 8.0 (default, matches GLB native length)
  CINE_FPS       — 24 (default)
  CINE_SAMPLES   — 32 (default Cycles sample budget)
  CINE_GLB       — path override; defaults to subsite light-palette GLB
  CINE_HDRI      — "assets/studio_small_08_1k.exr" (default) | any of
                   the bundled names ("studio" | "interior" |
                   "courtyard" | "city" | "forest" | "sunrise" |
                   "sunset" | "night") | "none" (flat-grey, no HDRI)
                   | a path (absolute or relative to this script) to
                   any .exr / .hdr. The Polyhaven studio_small_08
                   multi-softbox HDRI cached under assets/ is the
                   default — it's been the best-looking so far on
                   the anodized/extrusion materials and stays
                   uniform under a moving camera.
  CINE_HDRI_ROT  — HDRI yaw rotation in degrees, or "auto" (default).
                   "auto" scans the HDRI's equator-band luminance via
                   ``cad_khana.render.lighting.scan_hdri`` and yaws so the
                   *backdrop* (widest uniformly-bright region — the
                   photographer's intended white/grey wall) lands at
                   the look-through-the-rig azimuth (60°, opposite
                   the az=240° camera). Earlier iterations aligned
                   the darkest column instead, which turned out to
                   be a shadow corner between the wall and the
                   softbox rather than a usable backdrop.

Outputs land in ``outputs/cinematic/scanner-<camera>-<mode>.mp4``.
"""

from __future__ import annotations

import dataclasses
import math
import os
from collections.abc import Callable
from pathlib import Path

HERE = Path(__file__).resolve().parent

from .animated_assembly import build_m03_at  # noqa: E402
from .detail_variations import apply_default_detail  # noqa: E402
from .hdri_framing import lookup as hdri_override_lookup  # noqa: E402
from .material_variations import DEFAULT_OVERRIDES  # noqa: E402

from cad_khana.render import lighting  # noqa: E402
from cad_khana.render.animation import (  # noqa: E402
    render_animation,
    render_animation_from_glb,
)
from cad_khana.render.framing import FixedPose  # noqa: E402
from cad_khana.render.scene import Scene  # noqa: E402


DEFAULT_GLB = (
    HERE.parent.parent
    / "subsite"
    / "assets"
    / "exhibits"
    / "scanner-260523-light.glb"
)
GLB = Path(os.environ.get("CINE_GLB", str(DEFAULT_GLB)))
OUT_DIR = HERE / "outputs" / "cinematic"

MODE = os.environ.get("CINE_MODE", "both")
CAMERA_NAME = os.environ.get("CINE_CAMERA", "dump_intro")
NAME_SUFFIX = os.environ.get("CINE_NAME_SUFFIX", "")
RESOLUTION = tuple(int(x) for x in os.environ.get("CINE_RES", "720x720").split("x"))
DURATION_S = float(os.environ.get("CINE_DURATION", "8.0"))
FPS = int(os.environ.get("CINE_FPS", "24"))
SAMPLES = int(os.environ.get("CINE_SAMPLES", "32"))
WORLD_UP = (0.0, 0.0, 1.0)
HDRI_RAW = os.environ.get("CINE_HDRI", "assets/studio_small_08_1k.exr")
HDRI_ROT_RAW = os.environ.get("CINE_HDRI_ROT", "auto")

TARGET = (0.0, 0.0, 30.0)         # rig center — for any non-dump camera
# DUMP platform sits at world (0, -200, +50) at rest, (0, -252, -34) at
# peak tilt (queried from build_m03_at(0.0875).placed_parts). Aiming at
# the mean of the tilt arc keeps the action centred across all 4 dumps.
DUMP_TARGET = (0.0, -225.0, 10.0)

# The dump_static / dump_approach cameras orbit at azimuth ≈ 240°
# around DUMP_TARGET, so the look-through-the-rig direction (where the
# studio backdrop should sit for a clean shot) is the opposite world
# azimuth, 60°. ``dump_intro`` ends here too, so a single yaw aligned
# to this azimuth works for all three moves.
DUMP_LOOK_THROUGH_AZ_DEG = 60.0

# Preferred framing — what we'd shoot if the HDRI's backdrop were
# infinitely wide. ``_recommend_framing`` narrows these to fit the
# actual backdrop's angular width, per HDRI.
VFOV_DEG_PREF = 42.0
DRIFT_DEG_PREF = 10.0


def _resolve_hdri() -> str | Path:
    name = HDRI_RAW.lower()
    if name == "none":
        return None  # type: ignore[return-value]
    if name in lighting.BUNDLED_HDRI_NAMES:
        return name
    if HDRI_RAW.endswith((".exr", ".hdr")):
        candidate = Path(HDRI_RAW)
        if not candidate.is_absolute():
            candidate = HERE / candidate
        return candidate
    raise SystemExit(
        f"unknown CINE_HDRI: {HDRI_RAW!r} "
        f"(have {sorted(lighting.BUNDLED_HDRI_NAMES)} + 'none' "
        f"+ any .exr/.hdr path)"
    )


def _build_rig_and_framing() -> tuple[lighting.StudioRig, float, float]:
    """Pick cad_khana.render's HDRI rig and the (vfov, drift) framing that
    fit inside this HDRI's backdrop wall.

    ``CINE_HDRI=none`` falls back to the flat-grey ``DEFAULT`` rig
    (no HDRI, no backdrop constraint — preferred framing returned
    unchanged). Otherwise the HDRI is scanned for its backdrop
    centre + angular width; the yaw rotates the backdrop to the
    look-through-rig azimuth, and ``HdriScan.recommend_framing``
    narrows VFOV (and, if needed, drift) so the wall fully covers
    the swept frame across all drift positions. Near-uniform HDRIs
    (studio_small_08) get the preferred framing unchanged; narrow-
    wall HDRIs (studio_small_01 / _03) get tighter framing.

    Mirrors ``render.py``'s ``_pick_rig`` for the rig half so the
    still-image and cinematic outputs share an environment.
    """
    hdri = _resolve_hdri()
    if hdri is None:
        return (lighting.DEFAULT, VFOV_DEG_PREF, DRIFT_DEG_PREF)
    if HDRI_ROT_RAW.lower() == "auto":
        path = lighting.resolve_hdri_path(hdri)
        scan = lighting.scan_hdri(path)
        auto_yaw = scan.yaw_to_align(backdrop_at=DUMP_LOOK_THROUGH_AZ_DEG)
        auto_vfov, auto_drift = scan.recommend_framing(
            VFOV_DEG_PREF, DRIFT_DEG_PREF
        )
        override = hdri_override_lookup(path.name) or hdri_override_lookup(
            str(hdri)
        )
        yaw = float(override.get("yaw_deg", auto_yaw))
        vfov = float(override.get("vfov_deg", auto_vfov))
        drift = float(override.get("drift_deg", auto_drift))
        tag = " (override)" if override else ""
        print(
            f"[hdri] {path.name}{tag}: "
            f"backdrop az={scan.backdrop_azimuth_deg:.1f}°, "
            f"width={scan.backdrop_width_deg:.1f}°, "
            f"contrast {scan.contrast_ratio:.0f}× → "
            f"yaw {yaw:.1f}° (auto {auto_yaw:.1f}°), "
            f"vfov={vfov:.1f}° (auto {auto_vfov:.1f}°, pref {VFOV_DEG_PREF:.0f}°), "
            f"drift=±{drift:.1f}° (auto ±{auto_drift:.1f}°, pref ±{DRIFT_DEG_PREF:.0f}°)"
        )
    else:
        yaw = float(HDRI_ROT_RAW)
        vfov, drift = VFOV_DEG_PREF, DRIFT_DEG_PREF
    rig = dataclasses.replace(
        lighting.DEFAULT_HDRI,
        world_hdri=hdri,
        world_hdri_rotation_deg=yaw,
    )
    return (rig, vfov, drift)


RIG, VFOV_DEG, DRIFT_DEG = _build_rig_and_framing()

# Compensate camera distance so the rig keeps the same on-screen size
# when ``recommend_framing`` narrows the VFOV. For small-angle FOVs
# ``subject_radius / distance ≈ tan(vfov/2)``, so distance scales
# linearly with ``vfov_pref / vfov``. Side effect — and a feature —
# is mild telephoto compression on the directional-HDRI shots, which
# reads as more product-photo-like.
DISTANCE_SCALE = VFOV_DEG_PREF / VFOV_DEG
if DISTANCE_SCALE != 1.0:
    print(
        f"[framing] distance scale {DISTANCE_SCALE:.2f}× (vfov "
        f"{VFOV_DEG_PREF:.0f}°→{VFOV_DEG:.1f}°) to keep rig the same "
        f"size in frame"
    )


# --- Camera move math -------------------------------------------------
#
# Each function takes t ∈ [0, 1] and returns a FixedPose. The
# spherical-coords helper keeps the per-move code focused on the
# distance / elevation / azimuth schedule, not the trig.


def _smoothstep(s: float) -> float:
    """C¹ ease-in-out. 0→0, 1→1, derivative zero at endpoints."""
    s = max(0.0, min(1.0, s))
    return s * s * (3.0 - 2.0 * s)


def _pose(
    distance_mm: float,
    elevation_deg: float,
    azimuth_deg: float,
    target: tuple[float, float, float] = TARGET,
) -> FixedPose:
    el = math.radians(elevation_deg)
    az = math.radians(azimuth_deg)
    cx, cy, cz = target
    location = (
        cx + distance_mm * math.cos(el) * math.cos(az),
        cy + distance_mm * math.cos(el) * math.sin(az),
        cz + distance_mm * math.sin(el),
    )
    return FixedPose(
        location=location,
        target=target,
        vertical_fov_deg=VFOV_DEG,
        clip_start=1.0,
        clip_end=10000.0,
    )


def camera_dump_static(t: float) -> FixedPose:
    """Watch the DUMP zone near-stationary. Camera at azimuth ≈ 240°
    (between -X and -Y, 3/4 angle relative to DUMP) frames the
    platform tip with the rotor hub visible upper-left for context.
    Elevation 0° (horizontal) keeps the HDRI's backdrop behind the
    rig. Sin-cycle azimuth drift amplitude is ``DRIFT_DEG`` —
    auto-sized to fit the HDRI's backdrop width. Distance scales
    with ``DISTANCE_SCALE`` so the rig stays the same size in frame
    when the auto-framing narrows VFOV."""
    az = 240.0 + DRIFT_DEG * math.sin(2 * math.pi * t)
    return _pose(
        distance_mm=350.0 * DISTANCE_SCALE, elevation_deg=0.0,
        azimuth_deg=az, target=DUMP_TARGET,
    )


def camera_dump_approach(t: float) -> FixedPose:
    """Same DUMP-centered horizontal framing as ``dump_static`` but
    pushes closer across the clip: D=500 → D=250 (× ``DISTANCE_SCALE``)
    at fixed EL=0°. The fourth dump (t≈0.84) lands at the closest
    framing — punchline-style. Drift amplitude is 80% of
    ``DRIFT_DEG`` so the push-in motion stays dominant."""
    s = _smoothstep(t)
    distance = (500.0 + (250.0 - 500.0) * s) * DISTANCE_SCALE
    az = 240.0 + 0.8 * DRIFT_DEG * math.sin(2 * math.pi * t)
    return _pose(
        distance_mm=distance, elevation_deg=0.0, azimuth_deg=az,
        target=DUMP_TARGET,
    )


def camera_dump_intro(t: float) -> FixedPose:
    """Open looking edge-on at the rig from az≈135° (back-left,
    rig-centred), then sweep horizontally around to settle into
    the ``dump_static`` profile pose at az≈240°. Fixed elevation
    0° throughout so the HDRI's dark backdrop stays behind the rig
    across the whole sweep. Distances scale with ``DISTANCE_SCALE``."""
    s = _smoothstep(t)

    # Opening: back-left horizontal view, rig-centred. D=420 keeps
    # the camera just past the rig's outer extent so the V-slot
    # frame and the 4-camera tetrahedron read cleanly.
    start_target = (0.0, 0.0, 30.0)
    start_distance = 420.0 * DISTANCE_SCALE
    start_elevation = 0.0
    start_azimuth = 135.0

    # Settle: DUMP-centred profile, same as dump_static.
    end_target = DUMP_TARGET
    end_distance = 350.0 * DISTANCE_SCALE
    end_elevation = 0.0
    end_azimuth = 240.0

    target = tuple(
        start_target[i] + (end_target[i] - start_target[i]) * s
        for i in range(3)
    )
    distance = start_distance + (end_distance - start_distance) * s
    elevation = start_elevation + (end_elevation - start_elevation) * s
    azimuth = start_azimuth + (end_azimuth - start_azimuth) * s

    return _pose(
        distance_mm=distance,
        elevation_deg=elevation,
        azimuth_deg=azimuth,
        target=target,
    )


CAMERAS: dict[str, Callable[[float], FixedPose]] = {
    "dump_intro": camera_dump_intro,
    "dump_static": camera_dump_static,
    "dump_approach": camera_dump_approach,
}


def scene_factory(t: float) -> Scene:
    """Per-frame Scene for the photo-real path. Mirrors ``render.py``'s
    overlay chain: detail-variation + material overrides."""
    detailed = apply_default_detail(build_m03_at(t))
    return Scene.from_assembly(detailed).with_materials(DEFAULT_OVERRIDES)


def _ts(n_frames: int) -> list[float]:
    """Closed-loop sampling: ``t = i / n_frames`` so the final frame
    doesn't duplicate the first."""
    return [i / n_frames for i in range(n_frames)]


def _render_one(cam_name: str) -> None:
    if cam_name not in CAMERAS:
        raise SystemExit(f"unknown CINE_CAMERA: {cam_name!r} (have {list(CAMERAS)})")
    camera = CAMERAS[cam_name]
    n_frames = int(DURATION_S * FPS)

    if MODE in ("both", "photoreal"):
        print(f"\n--- {cam_name} photo-real ({n_frames} frames @ {FPS}fps, "
              f"{RESOLUTION[0]}×{RESOLUTION[1]}, Cycles {SAMPLES} samples, "
              f"hdri={getattr(RIG.world_hdri, 'name', RIG.world_hdri) or 'none'}) ---")
        mp4 = render_animation(
            scene_factory,
            ts=_ts(n_frames),
            out=str(OUT_DIR),
            name=f"scanner-{cam_name}-photoreal{NAME_SUFFIX}.mp4",
            duration_s=DURATION_S,
            camera=camera,
            resolution=RESOLUTION,
            samples=SAMPLES,
            world_up=WORLD_UP,
            rig=RIG,
        )
        print(f"wrote {mp4}")

    if MODE in ("both", "fromglb"):
        print(f"\n--- {cam_name} from-GLB ({n_frames} frames @ {FPS}fps, "
              f"{RESOLUTION[0]}×{RESOLUTION[1]}, Cycles {SAMPLES} samples, "
              f"hdri={getattr(RIG.world_hdri, 'name', RIG.world_hdri) or 'none'}) ---")
        mp4 = render_animation_from_glb(
            GLB,
            camera=camera,
            out=str(OUT_DIR),
            name=f"scanner-{cam_name}-fromglb{NAME_SUFFIX}.mp4",
            duration_s=DURATION_S,
            fps=FPS,
            resolution=RESOLUTION,
            samples=SAMPLES,
            world_up=WORLD_UP,
            rig=RIG,
        )
        print(f"wrote {mp4}")


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cams_to_render = list(CAMERAS) if CAMERA_NAME == "all" else [CAMERA_NAME]
    for cam_name in cams_to_render:
        _render_one(cam_name)


if __name__ == "__main__":
    main()
