"""M03 scanner — animated GLB export for the subsite scanner pane.

Sweeps ``build_m03_at(t)`` over one full turret revolution and emits a
glTF animation block on top of the static geometry path
(``export_glb_shape.py``). Per-part TRS samplers target node-by-name
matches; static rig parts get no channel and carry through unchanged.

Consumed by ``<model-viewer autoplay>`` in the subsite scanner pane,
which loops the sweep in ``DURATION_S`` seconds.
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from animated_assembly import build_m03_at  # noqa: E402
from detail_variations import apply_default_detail  # noqa: E402

from cad_khana.export import export_animated_glb  # noqa: E402

OUT_DIR = HERE.parent.parent / "subsite" / "assets" / "exhibits" / "scanner"

# 61 samples = one revolution at 6° steps. Closes the loop (t=1 equals
# t=0), so a glTF LOOP plays seamlessly.
N_FRAMES = 61
DURATION_S = 8.0


def factory(t: float):
    return apply_default_detail(build_m03_at(t))


def main() -> None:
    ts = [i / (N_FRAMES - 1) for i in range(N_FRAMES)]
    path = export_animated_glb(
        factory,
        ts=ts,
        out=OUT_DIR,
        name="scanner-animated.glb",
        duration_s=DURATION_S,
    )
    print(f"wrote {path} ({path.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
