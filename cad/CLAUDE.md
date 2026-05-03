# CAD decomposition

Guidance for organizing Build123d / cad-khana assemblies in this repo.

## Principles

1. **One concern per sub-assembly.** Each `assembly.py` validates one
   coherent physical sub-system. Don't mix unrelated parts or checks
   in a single assembly just because they share a module.

2. **Sub-assemblies run standalone.** `python <dir>/assembly.py` must
   work without any sibling sub-assembly. Shared inputs live in a
   module-local `params.py`; sibling-to-sibling imports are not
   allowed.

3. **Top-level assemblies hold cross-sub-assembly checks.** When an
   interference check spans two sub-systems (e.g. ramp tip vs basket
   lip), put it in a top-level `assembly.py` that sits alongside its
   sibling sub-assembly directories and imports them. Don't wrap the
   top-level in its own subdirectory — the enclosing module directory
   is already named for the whole assembly. Per-part printability
   stays in the sub-assembly that owns the part; the top-level runs
   `check()` only.

4. **One directory per FDM part family or mechanical sub-system.**
   Non-FDM hardware (aluminum extrusions, bearings, fasteners) appears
   as a stub inside the sub-assembly that constrains it, not as its
   own directory.

5. **Parameters live at their owning scope.** A `params.py` at any
   level holds only that frame's anchors and the explicit interface
   mirrors shared with peers (the `MIRROR_MOUNT_PLATE_T`-style cross-
   boundary duplicates with assertions). Sub-assembly internals —
   housing dimensions, bearing types, bolt patterns — stay inside the
   sub-assembly that owns them. Constants leaking upward force future
   readers to disentangle genuinely-shared values from private details
   that happened to be visible.

6. **Group sub-assemblies that share a natural local frame.** When a
   set of sub-assemblies forms a relocatable unit with its own local
   frame (e.g. a turret about a vertical hub axis, a camera rig about
   an optical origin), give them a parent directory whose
   `assembly.py` composes them in unit-local coordinates and whose
   `params.py` owns the unit's anchors. The top-level then places the
   unit in world via one transform — so "where does this unit go?"
   is a single-number decision instead of a constant-by-constant
   rewrite. The parent itself runs standalone (principle 2 applies
   recursively); cross-unit checks live in the enclosing top-level.

## World axes: Z is up

Every module in this tree uses the same world frame:

    +X  = operator's right
    +Y  = away from operator (toward the back of the machine)
    +Z  = up (gravity is -Z)

This matches build123d's defaults (`Plane.XY` is the default work
plane, extrusions rise in +Z) and the FDM print convention (the
printer's nozzle moves in +Z). Every sub-assembly's local frame
either matches world or differs by a documented `Rot(...)`; never
adopts a different "up" axis.

When importing geometry from external sources that use a different
convention (graphics/OpenGL viewpoint synthesis is typically Y-up;
some CAD packages use Y-up), **convert at the boundary**: rewrite
the constants into Z-up before they enter `params.py`. Don't carry
foreign-frame values verbatim and document the convention away —
mixed conventions in the same repo make every cross-module
assertion suspect, and the rotation helpers (`rotation_for_normal`
and friends) silently produce wrong placements when fed normals
from the wrong frame.

## Directory layout example

```
cad/m05_diverter/
    assembly.py            # top-level, cross-sub-assembly checks
    params.py              # shared inputs + derived interface constants
    basket/
        assembly.py        # basket sub-assembly (standalone)
    ramp_mechanism/
        assembly.py        # column + trap + ramp (standalone)
```

Use underscores in directory names so Python can import them
normally; hyphens block package imports and force `importlib`
workarounds.

## Running checks

`cad-khana` and `chitra-cad` are installed (editable) in the repo's
`uv`-managed venv at `.venv/`. Invoke the CLI directly:

```
.venv/bin/khana check  <path/to/assembly.py>   # diagnostics only
.venv/bin/khana build  <path/to/assembly.py>   # also write STL/STEP
.venv/bin/khana view   <path/to/assembly.py>   # push to the OCP viewer
.venv/bin/khana render <path/to/assembly.py>   # ortho + iso PNGs
```

`uv run khana …` works too. After any geometry change, run
`khana check` on the affected sub-assembly's `assembly.py` and the
top-level `assembly.py` that includes it before claiming the slice is
done — exit 0 means assertions and printability inspects all passed.

## Commit message draft

Whenever a CAD slice updates a module's `NEXT_STEPS.md` (or
equivalent live progress doc), also save the corresponding commit
message draft to `.commit-msg` at the project root in the same edit
batch. The two updates pair: NEXT_STEPS records what landed in the
sub-assembly tree, `.commit-msg` records the commit that will carry
it. Don't leave one updated and the other stale.

Follow the global commit-message convention from `~/.claude/CLAUDE.md`
(single-line conventional subject, co-author trailer, body only when
it earns its place).

## Feeding observations back to cad-khana

This repo is a heavy user of `cad-khana`; real-use friction is a
primary signal for what the skill/guidelines should cover. Treat
every session here as an opportunity to improve `cad-khana` itself.

- When an idiom, gotcha, or convention surfaces that isn't in
  cad-khana's `SKILL.md` — CWD-relative `outputs/`, coordinate-frame
  docstrings, where constants live, helper naming, assertion
  placement, etc. — log a one-liner observation in
  `../cad-khana/field-notes.md` (uncommitted; see that file's
  header). Include the concrete example (part name, symptom) so
  future agents can judge whether the pattern repeats.
- **Once 2–3 observations point at the same pattern, promote it
  into cad-khana** (SKILL.md description, SKILL.md body, or
  `NOTES.md` as appropriate) and remove the matched field-notes
  entries. Don't promote earlier — one-off accidents aren't
  patterns.
- The threshold exists because SKILL.md is part of every future
  agent's system prompt; every line there costs context, and
  speculative guidance ages badly.
