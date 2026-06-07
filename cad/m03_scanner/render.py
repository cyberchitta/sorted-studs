"""M03 scanner — photo-real render via chitra-cad.

Imports the composed top-level assembly (already module-level in
``assembly.py``) and applies two override layers before handing it to
chitra-cad:

  * ``detail_variations.default_detail`` — swap each cheap 20×20×L
    Box stub for a real V-slot profile from ``bd_warehouse``. Same
    pose, same material/color; only the underlying shape changes.
  * ``material_variations.DEFAULT_OVERRIDES`` — the render-only
    material overrides for parts whose real-world material isn't yet
    representable in chitra-cad's table. Settled materials live on
    the assembly itself; see ``material_variations.py`` for the
    promotion convention.

Lighting and camera framing come from chitra-cad's defaults; do not
author them here.
"""

import dataclasses
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from assembly import assembly  # noqa: E402
from detail_variations import apply_default_detail  # noqa: E402
from material_variations import DEFAULT_OVERRIDES  # noqa: E402

from chitra_cad import lighting  # noqa: E402
from chitra_cad.framing import HERO_3Q  # noqa: E402
from chitra_cad.render import render  # noqa: E402
from chitra_cad.scene import Scene  # noqa: E402


detailed = apply_default_detail(assembly)
scene = Scene.from_assembly(detailed).with_materials(DEFAULT_OVERRIDES)


def _pick_rig() -> lighting.StudioRig:
    """``SORTED_STUDS_HDRI=studio`` (default) selects chitra-cad's HDRI
    rig; ``none`` falls back to the flat-grey DEFAULT for diagnostic
    comparison; any other bundled name (``interior`` / ``courtyard`` /
    ``forest`` / …) swaps the environment; an absolute or
    relative-to-this-script path to an ``.exr`` / ``.hdr`` uses a
    custom HDRI (e.g. ``assets/studio_small_03_1k.exr`` for a
    multi-softbox setup that stays uniform under a moving camera).
    ``SORTED_STUDS_HDRI_ROT`` overrides the HDRI yaw rotation in
    degrees (default 0)."""
    raw = os.environ.get("SORTED_STUDS_HDRI", "studio")
    name = raw.lower()
    rot_deg = float(os.environ.get("SORTED_STUDS_HDRI_ROT", "0"))
    if name == "none":
        return lighting.DEFAULT
    if name in lighting.BUNDLED_HDRI_NAMES:
        hdri: str | Path = name
    elif raw.endswith((".exr", ".hdr")):
        candidate = Path(raw)
        if not candidate.is_absolute():
            candidate = Path(__file__).resolve().parent / candidate
        hdri = candidate
    else:
        raise SystemExit(
            f"unknown SORTED_STUDS_HDRI: {raw!r} "
            f"(have {sorted(lighting.BUNDLED_HDRI_NAMES)} + 'none' "
            f"+ any .exr/.hdr path)"
        )
    rig = dataclasses.replace(
        lighting.DEFAULT_HDRI,
        world_hdri=hdri,
        world_hdri_rotation_deg=rot_deg,
    )
    if os.environ.get("SORTED_STUDS_NO_KEY") == "1":
        rig = dataclasses.replace(rig, key=None)
    if os.environ.get("SORTED_STUDS_NO_FILL") == "1":
        rig = dataclasses.replace(rig, fill=None)
    return rig


if __name__ == "__main__":
    out_dir = os.environ.get("SORTED_STUDS_RENDERS_OUT", "outputs/renders")
    transparent = os.environ.get("SORTED_STUDS_TRANSPARENT", "0") == "1"
    render(
        scene,
        out=out_dir,
        resolution=(1920, 1080),
        world_up=(0.0, 0.0, 1.0),
        framing=HERO_3Q,
        transparent_background=transparent,
        rig=_pick_rig(),
    )
