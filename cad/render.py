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
    chitra-cad's table (m03 only). Keys are module-local tree paths;
    composition qualifies them under the unit's subtree name
    (``rig.cam_front_low_pcb`` → ``m03.rig.cam_front_low_pcb``).

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

from build123d import Pos

from cad_khana.mechanism.assembly import Assembly
from chitra_cad.framing import HERO_3Q
from chitra_cad.render import render
from chitra_cad.scene import Scene


THIS_DIR = Path(__file__).resolve().parent

# Make ``cad`` importable so we can mutate the schema switch.
sys.path.insert(0, str(THIS_DIR.parent))
import cad.design_systems as _ds  # noqa: E402


# --- Placement --------------------------------------------------------
#
# m05 sits at world identity; the m03 pose derives from the units'
# exported anchors inside ``_build_top_level`` — the same derivation
# as ``cad/assembly.py`` (DUMP centroid over the m05 hub axis, turret
# floor bottom on the deck top face), so the render can no longer
# drift from the check assembly's layout. The m02 singulator here is
# a render-only vignette (the m02_chain unit owns the real S1
# placement): it hangs over the m03 LOAD platform so its exit arc
# drops bricks onto LOAD, offset by two render-local constants.

M02_EXIT_RADIUS = 90.0
SINGULATOR_DROP_OVER_LOAD = 100.0

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
    overrides are render-time-only and stay module-local here;
    ``_build_top_level`` qualifies them under the unit's subtree
    name before they hit the chitra-cad scene.
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


def _build_top_level() -> tuple[Assembly, dict[str, str]]:
    """Compose the three-module top-level assembly with detail
    variations applied. Returns ``(assembly, material_overrides)``
    with the override keys qualified as full tree paths.
    """
    assy_m05, mat_m05 = _load_module_detailed("m05_diverter")
    assy_m03, mat_m03 = _load_module_detailed("m03_scanner")
    assy_m02, mat_m02 = _load_module_detailed("m02_c_channel_singulator")

    deck = assy_m05.anchor("deck_top").position
    dump = assy_m03.anchor("dump_centroid").position
    floor_bottom = assy_m03.anchor("floor_bottom").position
    m03_pose = Pos(deck.X - dump.X, deck.Y - dump.Y, deck.Z - floor_bottom.Z)
    load = (m03_pose * assy_m03.anchor("load_top")).position
    m02_pose = Pos(
        load.X - M02_EXIT_RADIUS, load.Y, load.Z + SINGULATOR_DROP_OVER_LOAD
    )

    top = (
        Assembly()
        .with_subassembly("m05", assy_m05)
        .with_subassembly("m03", assy_m03, location=m03_pose)
        .with_subassembly("m02", assy_m02, location=m02_pose)
    )

    qualified_mat: dict[str, str] = {}
    for unit, mat in (("m02", mat_m02), ("m03", mat_m03), ("m05", mat_m05)):
        for path, token in mat.items():
            qualified_mat[f"{unit}.{path}"] = token
    return top, qualified_mat


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
