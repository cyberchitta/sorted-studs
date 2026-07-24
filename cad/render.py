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
between renders. Each unit's per-module ``design_systems.py`` reads
the cad-level switch at import time, so between schema passes we
evict the unit subtrees from ``sys.modules`` and re-import them. The
shared ``cad.design_systems`` module is never evicted — it keeps its
mutated attribute, so the re-import sees the new schema.

Lighting and camera framing come from chitra-cad's defaults.
"""

from __future__ import annotations

import importlib
import importlib.util
import sys

from build123d import Pos

from cad_khana.mechanism.assembly import Assembly
from chitra_cad.framing import HERO_3Q
from chitra_cad.render import render
from chitra_cad.scene import Scene

from . import design_systems as _ds


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

# Unit subtrees evicted between schema passes so their import-time
# palette bindings re-read the mutated switch. ``cad.design_systems``
# itself deliberately doesn't match — it carries the switch.
_UNIT_MODULE_PREFIXES = ("cad.m02_", "cad.m03_", "cad.m05_")


def _evict_unit_modules() -> None:
    for name in list(sys.modules):
        if name.startswith(_UNIT_MODULE_PREFIXES):
            del sys.modules[name]


def _optional_import(dotted: str):
    if importlib.util.find_spec(dotted) is None:
        return None
    return importlib.import_module(dotted)


def _load_module_detailed(module_dirname: str) -> tuple[Assembly, dict[str, str]]:
    """Load a module's assembly + apply its detail variations.

    Returns ``(detailed_assembly, material_overrides)``. Material
    overrides are render-time-only and stay module-local here;
    ``_build_top_level`` qualifies them under the unit's subtree
    name before they hit the chitra-cad scene.
    """
    unit = f"cad.{module_dirname}"
    assembly: Assembly = importlib.import_module(f"{unit}.assembly").assembly

    detail = _optional_import(f"{unit}.detail_variations")
    if detail is not None and hasattr(detail, "apply_default_detail"):
        assembly = detail.apply_default_detail(assembly)

    material_overrides: dict[str, str] = {}
    mat = _optional_import(f"{unit}.material_variations")
    if mat is not None:
        material_overrides = dict(getattr(mat, "DEFAULT_OVERRIDES", {}))

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
    _evict_unit_modules()
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
