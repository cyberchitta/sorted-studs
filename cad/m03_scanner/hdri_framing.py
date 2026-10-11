"""Per-HDRI manual framing overrides for ``render_cinematic_mp4.py``.

For each HDRI, ``cad_khana.render.lighting.scan_hdri`` computes a backdrop
azimuth + width + a recommended (vfov, drift) that fits the wall.
The auto-computed values are usually fine. When they aren't —
because the wall edge is diffuse, or the rig should sit slightly
off-centre against the backdrop, or you want a specific drift
amplitude for cinematic reasons — add an entry here.

Keys are the HDRI filename's basename (e.g. ``studio_small_03_1k.exr``)
or the bundled-name string (e.g. ``studio``). Lookups are
case-insensitive. Every field is optional — omit it to use the
auto-computed value.

Values land in ``render_cinematic_mp4.py``'s ``_build_rig_and_framing``
as overrides applied AFTER the scan. The script logs both the
auto-computed and the final values so you can compare.
"""

from typing import TypedDict


class Override(TypedDict, total=False):
    yaw_deg: float
    """HDRI yaw in degrees. Overrides
    ``scan.yaw_to_align(backdrop_at=DUMP_LOOK_THROUGH_AZ_DEG)``."""
    vfov_deg: float
    """Vertical FOV in degrees. Overrides
    ``scan.recommend_framing(...)[0]``. Narrow values zoom in."""
    drift_deg: float
    """Azimuth-drift amplitude in degrees. Overrides
    ``scan.recommend_framing(...)[1]``. Set to 0 for a fully static
    camera."""
    note: str
    """Free-text human note. Ignored by the script."""


OVERRIDES: dict[str, Override] = {
    # Examples to fill in once we settle on visually-confirmed numbers:
    #
    # "studio_small_03_1k.exr": {
    #     "vfov_deg": 32.0,
    #     "drift_deg": 5.0,
    #     "note": "60° wall — narrow framing to keep wall behind rig",
    # },
    # "studio": {  # = bundled studio_small_01
    #     "yaw_deg": 26.0,
    #     "note": "fine-tuned by hand: 86° + 4° to land wall centre",
    # },
}


def lookup(name: str) -> Override:
    """Return the override entry for ``name`` (filename basename or
    bundled-name string), or an empty dict if none. Case-insensitive."""
    key = name.lower()
    for k, v in OVERRIDES.items():
        if k.lower() == key:
            return v
    return {}
