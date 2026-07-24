"""M03 scanner — animated GLB export for the subsite scanner pane.

Sweeps ``build_m03_at(t)`` over one full turret revolution and emits a
glTF animation block on top of the static geometry path
(``export_glb.py``). Per-part TRS samplers target node-by-name
matches; static rig parts get no channel and carry through unchanged.

Consumed by ``<glb-pane>`` in the subsite scanner pane, which loops the
sweep in ``DURATION_S`` seconds. Filename embeds the active palette
so the subsite's theme-aware src swapper can pick ``-light`` /
``-dark`` per visitor's ``prefers-color-scheme``.
"""

from datetime import date
from pathlib import Path

HERE = Path(__file__).resolve().parent

from .animated_assembly import build_m03_at  # noqa: E402
from .detail_variations import apply_default_detail  # noqa: E402

from cad_khana.export import export_animated_glb  # noqa: E402

from ..design_systems import Role, fill_palette_neutrals, resolve_name  # noqa: E402

from .design_systems import DESIGN_SYSTEM  # noqa: E402

OUT_DIR = HERE.parent.parent / "subsite" / "assets" / "exhibits"

# 61 samples = one revolution at 6° steps. Closes the loop (t=1 equals
# t=0), so a glTF LOOP plays seamlessly.
N_FRAMES = 61
DURATION_S = 8.0


def factory(t: float):
    detailed = apply_default_detail(build_m03_at(t))
    return fill_palette_neutrals(detailed, DESIGN_SYSTEM.palette[Role.SECONDARY_STRUCTURE])


def main() -> None:
    palette = resolve_name("M03")
    tag = palette.removeprefix("cyberchitta-")
    ts = [i / (N_FRAMES - 1) for i in range(N_FRAMES)]
    path = export_animated_glb(
        factory,
        ts=ts,
        out=OUT_DIR,
        name=f"scanner-{date.today():%y%m%d}-{tag}.glb",
        duration_s=DURATION_S,
    )
    print(f"wrote {path} ({path.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
