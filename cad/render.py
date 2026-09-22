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

Schema switch is a **parameter**: ``render_schema(schema)`` passes the
name to each unit builder, which binds its own tag dicts and paints
its own subtree. Every schema builds in this one process. Until
2026-07-26 the schema was a module global mutated between renders, so
this file had to evict ``cad.m02_*`` / ``cad.m03_*`` / ``cad.m05_*``
from ``sys.modules`` and re-import them to make the units re-read it —
the last surviving eviction site in the tree, and now gone.

Lighting and camera framing come from chitra-cad's defaults.
"""

from __future__ import annotations

import importlib
import importlib.util

from build123d import Pos

from cad_khana.mechanism.assembly import Assembly
from chitra_cad.framing import HERO_3Q
from chitra_cad.render import render
from chitra_cad.scene import Scene

from .assembly import m03_pose
from .m02_c_channel_singulator.assembly import build_m02
from .m03_scanner.assembly import build_m03
from .m05_diverter.assembly import build_m05


# --- Placement --------------------------------------------------------
#
# m05 sits at world identity; the m03 pose is ``cad/assembly.py``'s
# own ``m03_pose`` (DUMP centroid over the m05 hub axis, turret floor
# bottom on the deck top face, turned by ψ) — one function, so the
# render cannot drift from the check assembly's layout. (It did once:
# a copied derivation here left out ψ.) The m02 singulator here is
# a render-only vignette (the m02_chain unit owns the real S1
# placement): it hangs over the m03 LOAD platform so its exit arc
# drops bricks onto LOAD, offset by two render-local constants.

M02_EXIT_RADIUS = 90.0
SINGULATOR_DROP_OVER_LOAD = 100.0


# --- Override layers --------------------------------------------------

def _optional_import(dotted: str):
    if importlib.util.find_spec(dotted) is None:
        return None
    return importlib.import_module(dotted)


def _with_overrides(
    unit_dirname: str, assembly: Assembly
) -> tuple[Assembly, dict[str, str]]:
    """Apply a unit's detail variations and collect its material
    overrides.

    Returns ``(detailed_assembly, material_overrides)``. Material
    overrides are render-time-only and stay module-local here;
    ``_build_top_level`` qualifies them under the unit's subtree name
    before they hit the chitra-cad scene. Both layers are optional —
    only m03 and m05 carry detail variations, only m03 carries
    material overrides — hence the spec probe rather than a plain
    import.
    """
    unit = f"cad.{unit_dirname}"

    detail = _optional_import(f"{unit}.detail_variations")
    if detail is not None and hasattr(detail, "apply_default_detail"):
        assembly = detail.apply_default_detail(assembly)

    material_overrides: dict[str, str] = {}
    mat = _optional_import(f"{unit}.material_variations")
    if mat is not None:
        material_overrides = dict(getattr(mat, "DEFAULT_OVERRIDES", {}))

    return assembly, material_overrides


def _build_top_level(schema: str) -> tuple[Assembly, dict[str, str]]:
    """Compose the three-module top-level assembly under ``schema``,
    with detail variations applied. Returns ``(assembly,
    material_overrides)`` with the override keys qualified as full tree
    paths.
    """
    assy_m05, mat_m05 = _with_overrides("m05_diverter", build_m05(schema))
    assy_m03, mat_m03 = _with_overrides("m03_scanner", build_m03(schema))
    assy_m02, mat_m02 = _with_overrides(
        "m02_c_channel_singulator", build_m02(schema)
    )

    m03_at = m03_pose(assy_m05, assy_m03)
    load = (m03_at * assy_m03.anchor("load_top")).position
    m02_pose = Pos(
        load.X - M02_EXIT_RADIUS, load.Y, load.Z + SINGULATOR_DROP_OVER_LOAD
    )

    top = (
        Assembly()
        .with_subassembly("m05", assy_m05)
        .with_subassembly("m03", assy_m03, location=m03_at)
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
    top, mat = _build_top_level(schema)
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
