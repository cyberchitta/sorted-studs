"""
sorted-studs — top-level photo-real render.

Composes the same three-module stack as ``cad/assembly.py`` and
renders it under both design schemas (``workshop`` and ``bauhaus``)
via chitra-cad. Output lands in
``cad/outputs/renders/<schema>/`` — under the existing
``cad/**/outputs/`` gitignore prefix, so renders don't bloat the
repo.

Layers applied before rendering (per module):

  * ``detail_variations.apply_default_detail`` — swap each cheap
    20×20×L Box stub for a real V-slot profile from ``bd_warehouse``.
    m03 and m05 expose this; m02 has none yet (its NEXT_STEPS lists
    the slice).
  * ``material_variations.DEFAULT_OVERRIDES`` — render-only material
    overrides for parts whose real-world material isn't yet in
    chitra-cad's table (m03 only). Names are prefixed at the
    splice site to match the composed assembly's part names.

Schema switch is a *mutation* of ``cad.design_systems.DESIGN_SYSTEM_NAME``
between renders. Each module's per-module ``design_systems.py`` reads
the cad-level switch at import time, so we evict module-local names
between schema passes and re-load each module via ``runpy``. The
shared ``cad.design_systems`` module stays in ``sys.modules`` with
its mutated attribute, so the next reload sees the new schema.

Lighting and camera framing come from chitra-cad's defaults.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

from build123d import Location, Pos

from cad_khana.mechanism.assembly import Assembly
from chitra_cad.framing import HERO_3Q
from chitra_cad.render import render
from chitra_cad.scene import Scene


THIS_DIR = Path(__file__).resolve().parent

# Make ``cad`` importable so we can mutate the schema switch.
sys.path.insert(0, str(THIS_DIR.parent))
import cad.design_systems as _ds  # noqa: E402


# --- Placement (mirrors cad/assembly.py) ------------------------------
#
# Top-level placement constants are duplicated from cad/assembly.py
# rather than imported, because importing cad.assembly executes its
# own module load + cross-module assertion graph at the wrong schema.
# A drift here surfaces visually as a misaligned render — easy to
# spot.

M05_TOP_COLUMN_TOP_Z = 933.0
SCANNER_BENCH_LIFT_OVER_COLUMN = 20.0
_M03_RAIL_HALF_BELOW_BENCH = 10.0
_M03_SHARED_BENCH_Z = -220.0
M03_TURRET_ARM_R = 185.0
M02_EXIT_RADIUS = 90.0
SINGULATOR_DROP_OVER_LOAD = 100.0

_M03_PLATFORM_TOP_Z = (
    M05_TOP_COLUMN_TOP_Z
    + SCANNER_BENCH_LIFT_OVER_COLUMN
    + _M03_RAIL_HALF_BELOW_BENCH
    - _M03_SHARED_BENCH_Z
)   # 1183

_M05_POSE = Location()
_M03_POSE = Pos(M03_TURRET_ARM_R, 0, _M03_PLATFORM_TOP_Z)
_M02_POSE = Pos(
    2 * M03_TURRET_ARM_R - M02_EXIT_RADIUS,
    0,
    _M03_PLATFORM_TOP_Z + SINGULATOR_DROP_OVER_LOAD,
)

_PREFIX = {
    "m02": "m02__",
    "m03": "m03__",
    "m05": "m05__",
}


# --- Module-load helpers ---------------------------------------------

_MODULE_LOCAL_HEADS = frozenset(
    {
        "params",
        "design_systems",
        "materials",
        "detail_variations",
        "material_variations",
        "drive",
        "rotor",
        "stator",
        "camera_rig",
        "turret",
        "frame",
        "basket",
        "ramp_mechanism",
        "base_floor",
        "inner_vertex_bracket",
        "outer_vertex_bracket",
    }
)


def _evict_module_locals() -> None:
    for name in list(sys.modules):
        head = name.split(".", 1)[0]
        if head in _MODULE_LOCAL_HEADS:
            del sys.modules[name]


def _runpy_isolated(path: Path, run_name: str) -> dict:
    saved_path = sys.path.copy()
    _evict_module_locals()
    try:
        return runpy.run_path(str(path), run_name=run_name)
    finally:
        sys.path[:] = saved_path
        _evict_module_locals()


def _load_module_detailed(module_dirname: str) -> tuple[Assembly, dict[str, str]]:
    """Load a module's assembly + apply its detail variations.

    Returns ``(detailed_assembly, material_overrides)``. Material
    overrides are render-time-only and are kept un-prefixed here;
    the splice site adds the module prefix before they hit the
    chitra-cad scene.
    """
    mod_dir = THIS_DIR / module_dirname
    ns_assy = _runpy_isolated(mod_dir / "assembly.py", f"<{module_dirname}>")
    assembly: Assembly = ns_assy["assembly"]

    detail_path = mod_dir / "detail_variations.py"
    if detail_path.exists():
        ns_det = _runpy_isolated(detail_path, f"<{module_dirname}.detail>")
        if "apply_default_detail" in ns_det:
            assembly = ns_det["apply_default_detail"](assembly)

    material_overrides: dict[str, str] = {}
    mat_path = mod_dir / "material_variations.py"
    if mat_path.exists():
        ns_mat = _runpy_isolated(mat_path, f"<{module_dirname}.material>")
        material_overrides = dict(ns_mat.get("DEFAULT_OVERRIDES", {}))

    return assembly, material_overrides


def _splice(
    top: Assembly, mod: Assembly, base: Location, prefix: str
) -> Assembly:
    for p in mod.parts:
        top = top.with_part(
            f"{prefix}{p.name}",
            p.part,
            location=base * p.location,
            color=p.color,
            material=p.material,
        )
    return top


def _build_top_level() -> tuple[Assembly, dict[str, str]]:
    """Compose the three-module top-level assembly with detail
    variations applied. Returns ``(assembly, prefixed_material_overrides)``.
    """
    assy_m05, mat_m05 = _load_module_detailed("m05_diverter")
    assy_m03, mat_m03 = _load_module_detailed("m03_scanner")
    assy_m02, mat_m02 = _load_module_detailed("m02_c_channel_singulator")

    top = Assembly()
    top = _splice(top, assy_m05, _M05_POSE, _PREFIX["m05"])
    top = _splice(top, assy_m03, _M03_POSE, _PREFIX["m03"])
    top = _splice(top, assy_m02, _M02_POSE, _PREFIX["m02"])

    prefixed_mat: dict[str, str] = {}
    for prefix, mat in (
        (_PREFIX["m02"], mat_m02),
        (_PREFIX["m03"], mat_m03),
        (_PREFIX["m05"], mat_m05),
    ):
        for name, token in mat.items():
            prefixed_mat[f"{prefix}{name}"] = token
    return top, prefixed_mat


# --- Render driver ---------------------------------------------------

SCHEMAS = ("workshop", "bauhaus")


def render_schema(schema: str) -> None:
    _ds.DESIGN_SYSTEM_NAME = schema
    top, mat = _build_top_level()
    scene = Scene.from_assembly(top).with_materials(mat)
    render(
        scene,
        out=f"outputs/renders/{schema}",
        resolution=(1920, 1080),
        world_up=(0.0, 0.0, 1.0),
        framing=HERO_3Q,
    )


if __name__ == "__main__":
    for schema in SCHEMAS:
        render_schema(schema)
