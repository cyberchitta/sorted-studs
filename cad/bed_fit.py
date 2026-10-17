"""Bed fit — every printed part against the printer's build volume.

    .venv/bin/khana run cad/bed_fit.py

`khana check` on `cad/assembly.py` covers nothing in this file, and
nothing here re-runs printability: each unit's `printability.py` /
`role_sweep.py` is imported with `inspect()` and `check()` swapped for
a recorder, so the body and its `up_axis` are exactly the ones that
script declares — orientation stays declared in one place.

Per recorded body: the height along `up_axis` and the smallest bed
footprint over in-plane rotations (a slicer may turn the part on the
bed). A body fits when its height ≤ Z and some rotation fits X × Y.
Read from a 0.02 mm tessellation, so a reading may undershoot by that.

Coverage: every placed part of `cad/assembly.py` made of a printed
material is matched to a recorded body by volume and area (placement
moves neither). A printed part no script inspects cannot be checked
here, and goes red as `uninspected`. Bought plastic parts — the hobby
servos, which only render as PLA — are listed in `BOUGHT_PLASTIC`.

Writes `outputs/bed_fit/bed_fit.json`; exits 1 on any misfit or
uninspected printed part.
"""

from __future__ import annotations

import importlib
import json
import math
import sys
from pathlib import Path

import numpy as np

from .printer import BUILD_VOLUME_MM

CAD = Path(__file__).parent
IMPORT_LINE = "from cad_khana.printability.inspect import inspect"


def _scripts() -> list[str]:
    """Every script under cad/ that imports `inspect` — found by the
    import, not the filename (m05's role×part inspections live in
    `role_sweep.py`, and a `printability.py` glob missed them once)."""
    found = []
    for p in sorted(CAD.rglob("*.py")):
        if p == Path(__file__) or "outputs" in p.parts:
            continue
        if IMPORT_LINE in p.read_text():
            found.append(".".join(p.relative_to(CAD).with_suffix("").parts))
    return found

PRINTED_MATERIALS = frozenset({"pla_matte"})
BOUGHT_PLASTIC = frozenset({"servo_body", "servo_horn", "trap_servo", "lower_trap_servo"})

TESS_TOL = 0.02
ANGLE_STEP_DEG = 0.5
MATCH_REL = 1e-4


def _record() -> list[dict]:
    """Import every script with inspect/check swapped for a recorder."""
    seen: list[dict] = []

    def recorder(part, *, method, name="part", out="outputs", **_):
        # A family member is told apart by its out dir (role_sweep's
        # `outputs/<role>`), so the dir joins the name.
        sub = str(out).removeprefix("outputs").strip("/")
        label = f"{sub}/{name}" if sub else name
        seen.append({"name": label, "part": part, "up": tuple(method.up_axis)})

    insp = importlib.import_module("cad_khana.printability.inspect")
    chk = importlib.import_module("cad_khana.mechanism.check")
    real = (insp.inspect, chk.check)
    insp.inspect, chk.check = recorder, (lambda *a, **k: None)
    try:
        for mod in _scripts():
            before = len(seen)
            importlib.import_module(f"{__package__}.{mod}")
            for r in seen[before:]:
                r["script"] = mod
    finally:
        insp.inspect, chk.check = real
    return seen


def _frame(up: tuple[float, float, float]) -> np.ndarray:
    """Rows: two in-bed axes, then up (unit)."""
    u = np.asarray(up, float)
    u /= np.linalg.norm(u)
    a = np.array([1.0, 0, 0]) if abs(u[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = np.cross(u, a)
    e1 /= np.linalg.norm(e1)
    return np.stack([e1, np.cross(u, e1), u])


def _measure(part, up) -> dict:
    verts, _ = part.tessellate(TESS_TOL)
    p = np.array([(v.X, v.Y, v.Z) for v in verts]) @ _frame(up).T
    height = float(p[:, 2].max() - p[:, 2].min())
    xy = p[:, :2]
    bx, by, bz = BUILD_VOLUME_MM
    best = None
    for k in range(int(180 / ANGLE_STEP_DEG)):
        t = math.radians(k * ANGLE_STEP_DEG)
        c, s = math.cos(t), math.sin(t)
        q = xy @ np.array([[c, -s], [s, c]])
        w = float(np.ptp(q[:, 0]))
        d = float(np.ptp(q[:, 1]))
        w, d = max(w, d), min(w, d)  # the bed is square-agnostic below
        slack = min(max(bx, by) - w, min(bx, by) - d)
        if best is None or slack > best[0]:
            best = (slack, w, d, k * ANGLE_STEP_DEG)
    slack, w, d, ang = best
    fits = height <= bz and slack >= 0
    return {
        "height_mm": round(height, 2),
        "footprint_mm": [round(w, 2), round(d, 2)],
        "rotation_deg": ang,
        "height_margin_mm": round(bz - height, 2),
        "footprint_margin_mm": round(slack, 2),
        "fits": fits,
    }


def _key(part) -> tuple[float, float]:
    return (part.volume, part.area)


def _same(a, b) -> bool:
    return all(abs(x - y) <= MATCH_REL * max(abs(x), abs(y), 1.0) for x, y in zip(a, b))


def main() -> int:
    recorded = _record()
    bodies: dict[str, dict] = {}
    for r in recorded:
        # One script may reuse a name across a family (role_sweep's three
        # roles), so a body is identified by its geometry, not its name:
        # same volume, area, up axis and fit readings.
        key = _key(r["part"])
        m = _measure(r["part"], r["up"])
        twin = next(
            (b for b in bodies.values()
             if _same(key, b["key"]) and b["up_axis"] == list(r["up"])
             and b["height_mm"] == m["height_mm"]
             and b["footprint_mm"] == m["footprint_mm"]),
            None,
        )
        if twin is not None:
            alias = f"{r['script']}:{r['name']}"
            if alias != f"{twin['script']}:{twin['name']}" and alias not in twin["also"]:
                twin["also"].append(alias)
            continue
        tag = f"{r['script']}:{r['name']}"
        n = 2
        while tag in bodies:
            tag = f"{r['script']}:{r['name']}#{n}"
            n += 1
        bodies[tag] = {
            "script": r["script"],
            "name": r["name"],
            "up_axis": list(r["up"]),
            "key": key,
            "also": [],
            **m,
        }

    from .assembly import assembly as top

    uninspected: dict[str, list[str]] = {}
    for pp in top.placed_parts:
        leaf = pp.name.rsplit(".", 1)[-1]
        if pp.material not in PRINTED_MATERIALS:
            continue
        if leaf.rstrip("0123456789_") in BOUGHT_PLASTIC or leaf in BOUGHT_PLASTIC:
            continue
        k = _key(pp.part)
        if not any(_same(k, b["key"]) for b in bodies.values()):
            uninspected.setdefault(leaf.rstrip("0123456789_"), []).append(pp.name)

    misfits = {t: b for t, b in bodies.items() if not b["fits"]}
    for b in bodies.values():
        del b["key"]
    report = {
        "build_volume_mm": list(BUILD_VOLUME_MM),
        "bodies": dict(sorted(bodies.items(), key=lambda kv: kv[1]["height_margin_mm"])),
        "misfits": sorted(misfits),
        "uninspected": uninspected,
        "status": "ok" if not misfits and not uninspected else "failed",
    }
    # `khana run` anchors only check()/inspect() outs to the script's dir.
    out = CAD / "outputs" / "bed_fit"
    out.mkdir(parents=True, exist_ok=True)
    (out / "bed_fit.json").write_text(json.dumps(report, indent=2) + "\n")

    print(f"build volume {BUILD_VOLUME_MM}; {len(bodies)} bodies")
    for t, b in report["bodies"].items():
        flag = "  " if b["fits"] else "RED"
        print(
            f"{flag} {t:70s} h {b['height_mm']:7.1f} (margin {b['height_margin_mm']:6.1f})"
            f"  fp {b['footprint_mm'][0]:6.1f}×{b['footprint_mm'][1]:6.1f}"
            f" @{b['rotation_deg']:5.1f}° (margin {b['footprint_margin_mm']:6.1f})"
            + (f"  = {', '.join(a.split(':')[1] for a in b['also'])}" if b["also"] else "")
        )
    for leaf, paths in sorted(uninspected.items()):
        print(f"RED uninspected {leaf}: {len(paths)} placed, e.g. {paths[0]}")
    print(report["status"])
    return 0 if report["status"] == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
