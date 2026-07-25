# CAD decomposition

Guidance for organizing Build123d / cad-khana assemblies in this repo.

## Principles

1. **One concern per sub-assembly.** Each `assembly.py` validates one
   coherent physical sub-system. Don't mix unrelated parts or checks
   in a single assembly just because they share a module.

2. **Sub-assemblies run standalone.** `khana check <dir>/assembly.py`
   (or `python -m cad.<unit>.<sub>.assembly` from the repo root) must
   work without any sibling sub-assembly. `cad/` is a package —
   `khana` runs a package-member script with `python -m` semantics, so
   its relative imports resolve; plain `python <path>` does not.
   Shared inputs live in a module-local `params.py`;
   sibling-to-sibling imports are not allowed.

   **A standalone green is a weaker claim than a top-level green.** The
   turret's c5/c6 slices both reported `khana check` passing while the
   kicker lever overlapped the cross rail by 216 mm³ and the kicker
   mount foot by 1904 mm³ — the standalones were reading stale local
   copies of constants their neighbours had already moved. Local
   copies of a peer's numbers are now the exception (principle 5:
   interfaces are anchors), and a unit that must keep one to build
   standalone exports a belief anchor from it so the parent's
   `check()` catches the drift — but cross-unit assertions still run
   only at the top. Re-run the enclosing top-level before calling a
   slice done.

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

5. **Parameters live at their owning scope; interfaces are anchors.**
   A `params.py` at any level holds only that frame's anchors.
   Sub-assembly internals — housing dimensions, bearing types, bolt
   patterns — stay inside the sub-assembly that owns them. Constants
   leaking upward force future readers to disentangle genuinely-shared
   values from private details that happened to be visible.

   A value a *peer* needs is not shared by copying it: the unit
   exports the datum as a named anchor (`with_anchor("deck_top", …)`
   in its own frame) and the composing parent derives placements from
   `anchor(path)` readings and asserts the interface with
   `assert_anchors_coincident`. A unit that must mirror a peer's
   constant to build standalone (the m02 chain) keeps the local copy
   but exports a *belief anchor* computed from it, so drift fails
   loudly at the parent's `check()` instead of silently desyncing the
   machine. (This retired the `MIRROR_MOUNT_PLATE_T`-style
   cross-boundary duplicate + hand-written drift-assert idiom.)

6. **The assembly is a tree; boundaries are rigid units and DOFs.**
   When a set of sub-assemblies forms a relocatable unit with its own
   local frame (e.g. a turret about a vertical hub axis, a camera rig
   about an optical origin), give them a parent directory whose
   `assembly.py` composes them in unit-local coordinates and whose
   `params.py` owns the unit's anchors — and give them the same
   boundary in the in-memory assembly: every relocatable unit, and
   every joint, is a `with_subassembly` node, declared from the start
   (a joint at angle 0 is fine), never bolted on later when animation
   needs it. Sub-modules expose `build_<name>() -> Assembly` in their
   own canonical local frame; the parent places the subtree via one
   `location=` — so "where does this unit go?" is a single-number
   decision instead of a constant-by-constant rewrite. Part identity
   is the dotted tree path (`m03.turret.rotor.arm.spider`) — no
   name-prefix conventions, no hand-maintained part-name tuples;
   attribute maps (color, material, detail) key on the leaf, because a
   builder can't know its placement path. The top level composes
   units the same way (`with_subassembly("m05", …)`); it never
   re-adds parts flat, so every unit's joints survive to the top.
   Assertions address subtrees with the group forms
   (`assert_no_interference_between("m03", "m05")`). Sub-assembly
   assertions **propagate**: the root that runs `check()` evaluates
   every nested assertion with paths, names, and datum-plane targets
   qualified through each level's placement + joint. So declare each
   claim once, at the level where the knowledge lives — a claim
   internal to one builder's parts goes in that builder (the ramp's
   rim-plane sweep clearance); a claim spanning sibling subtrees goes
   at the level that composes both (the pulley's Z corridor at the
   turret; cross-unit checks in the enclosing top-level). Don't
   mirror a claim at two levels — it just evaluates twice under two
   names. Assertions referencing parts absent from a run (detail-only
   fasteners, an optional trap_open) skip as `passed: null`; an
   assertion that is *always* skipped usually means a typo'd path.
   Group-form expansion is still a macro over current contents, so
   group asserts come after composition. The parent itself runs
   standalone (principle 2 applies recursively).

   Two corollaries for scalar claims. (1) Don't hand-derive from
   constants what the geometry already knows: `assert_distance` /
   `assert_scalar` record a named, diffable measured value, where a
   bare `assert` crashes the import (`status: "error"`) and measures
   nothing. Bare asserts stay only for parameter sanity with no
   geometric content (a deliberately loose upper bound, a cut
   feature's extent). (2) When geometry is *constructed from* the
   same constant as the bound (gear blanks placed at exactly
   backlash, a bracket sized off the same 1 mm air gap), measured ==
   bound, and the strict comparison flips on solver noise (~1e-14
   distance, ~1e-7 bbox extents) — set the bound a small explicit
   measurement tolerance below the constructed value and say so in a
   comment.

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

A wrong frame is not the only way to get a silently wrong placement.
**build123d's `Rot(a, b, c)` composes as `Rx·Ry·Rz`, not `Rz·Ry·Rx`.**
An early `rotation_for_normal` assumed the latter and placed *every*
rev-1 mirror mount with an incorrect outward normal — with no
assertion failure anywhere, because a wrong normal is not an overlap.
See "What a green `khana check` does not mean" below.

## Directory layout example

```
cad/
    __init__.py            # cad/ is a package — marker at every level
    assembly.py            # whole machine: with_subassembly per unit,
                           #   cross-unit group asserts + anchor checks
    m05_diverter/
        __init__.py
        assembly.py        # unit root: composes basket/ramp/… subtrees,
                           #   exports interface anchors, within-unit checks
        params.py          # unit-shared inputs + derived constants
        basket/
            __init__.py
            assembly.py    # build_basket_ring() -> Assembly (+ standalone check)
        ramp_mechanism/
            __init__.py
            assembly.py    # build_* builders (+ standalone check)
```

Imports are relative (`.params` at unit level, `..params` in
sub-modules); no `sys.path` manipulation anywhere. Use underscores in
directory names so Python can import them normally; hyphens block
package imports and force `importlib` workarounds.

## Running checks

`cad-khana` and `chitra-cad` are installed (editable) in the repo's
`uv`-managed venv at `.venv/`. Invoke the CLI directly:

```
.venv/bin/khana check  <path/to/assembly.py>   # diagnostics only
.venv/bin/khana build  <path/to/assembly.py>   # also write STL/STEP
.venv/bin/khana view   <path/to/assembly.py>   # push to the OCP viewer
.venv/bin/khana draw   <path/to/assembly.py>   # ortho + iso engineering drawings
```

`uv run khana …` works too. After any geometry change, run
`khana check` on the affected sub-assembly's `assembly.py` and the
top-level `assembly.py` that includes it before claiming the slice is
done — exit 0 means assertions and printability inspects all passed.

**Use the CLI, not bare `python`, for any script with more than one
`check()`/`inspect()`.** The CLI defers failures to its own boundary so
every call runs and every JSON on disk is fresh, then exits nonzero once;
a bare interpreter keeps raise-on-first-failure and aborts the run,
leaving the rest of the diagnostics stale at their previous values while
reading as current. This is a correctness difference, not a convenience:
a multi-role or multi-frame sweep under bare `python` reports only its
first red.

The same deferral means **`check()` and `inspect()` return normally on
failure under the CLI** — there is no `SystemExit` to catch. Read
failures off the result (`result.diagnostics.assertions`, `passed is
False` — `None` is a skipped assertion, not a failure); never wrap the
call in `try/except SystemExit` and never re-`evaluate()` the assembly to
find out. `animated_assembly.py`'s 9-frame sweep had exactly that
`except`, and after the deferral shipped it printed *"all 9 frames pass"*
on runs the CLI was failing.

## What a green `khana check` does not mean

Exit 0 means *no interference and printability passed, at the poses you
built*. Five classes of real defect have shipped past a green check in
this repo, each costing a redesign slice. When a slice depends on one of
these, the check is not your evidence — find the one that is.

- **Orientation.** A wrong outward normal is not an overlap. Every rev-1
  mirror mount was placed facing the wrong way with all assertions
  passing; it surfaced only when a floor bracket tried to tangent-contact
  the mount plate's back face. When a part's *facing* carries function,
  assert a tangent contact against the surface it must face.
- **Direction of motion.** Static REST geometry says nothing about which
  way a mechanism travels. The dump kicker pushed the platform
  outer-**up** — brick slides inward, backwards — for several slices,
  while its docstring asserted the opposite throughout. Check the sign at
  a non-REST pose; a docstring claiming kinematics is not evidence.
- **Visibility.** Nothing in `check()` sees a camera frustum. The kicker
  tab cleared every part geometrically and sat plainly visible in the
  scan frame; the photoreal render is what caught it. Render before
  trusting any optical or dataset-quality claim.
- **Physical reality across units.** Two independently floor-standing
  assemblies at different local floor heights pass cross-unit checks
  while being unbuildable — they share one bench. A shared datum must be
  asserted; the check cannot infer it. Export it as an anchor and
  `assert_anchors_coincident` at the top level (principle 5).
- **The poses you did not sample.** A single-pose assertion carries an
  implicit worst-case claim. `lower_trap_open` is asserted at 60° alone
  because 60° bounds the upward-intrusion envelope *at the current*
  `TRAP_L` / `TRAP_OPEN_DEG`. Write that claim beside the assert, or it
  expires silently when the parameter moves.

Two habits that keep the check honest:

- **Turn every fixed interference into a named regression assert**, and
  write a known-bad overlap as `assert_interference(reason=…)` rather
  than skipping it. A positive assertion self-fails the moment the
  redesign lands, so it cannot rot into folklore.
- **Verification expires when its substrate changes.** The ramp sweep
  results were derived on pre-rail basket geometry and were quietly
  invalid afterwards. Re-run it; don't cite it.

## Right-size the plan to the change

A multi-slice plan is a commitment device. Writing one before the
topology is verified is what led the dump-mechanism work to commit to a
cam topology across five slices and then roll back — the difficulty was
topology-specific, not intrinsic, and one page of first-principles
re-derivation found the servo answer that made the whole cascade moot.

- When interference volumes get *large* (~16 600 mm³ of forced overlaps,
  in that case), read it as a signal the **topology** is wrong, not that
  the numbers need tuning. Swapping one conflicted topology for another
  trades a kinematic conflict for a geometric one.
- **Re-price a prior rejection rather than inheriting it.** A
  one-paragraph dismissal in an earlier handoff is not evidence; the
  rejected path was re-opened and the objection was wrong on three
  counts.
- **Question the constraint before optimising inside it.** "Is 'no
  electronics for dumping' actually hard?" collapsed an entire design
  document to "use a servo."

## Landing a CAD slice

A slice lands in **two repos**, and neither is optional:

- the geometry, committed to `sorted-studs`;
- the module's `_notes/state/<module>.md` (or equivalent live
  progress doc), which records what landed — a working note in the
  private `working-notes` repo.

Update both in the same edit batch. Don't leave one current and the
other stale. They cannot land atomically (different repos), and that
drift is accepted — but only when both sides are actually written.

Commit message convention, pathspec discipline and the notes commit
cadence all come from the global `~/.claude/CLAUDE.md` and the
`working-notes` skill; this repo has no local exception. In
particular the message goes **straight into git**, never through a
`.commit-msg` file.

## Feeding observations back to cad-khana

This repo is a heavy user of `cad-khana`; real-use friction is a
primary signal for what the skill/guidelines should cover. Treat
every session here as an opportunity to improve `cad-khana` itself.

- When an idiom, gotcha, or convention surfaces that isn't in
  cad-khana's `SKILL.md` — CWD-relative `outputs/`, coordinate-frame
  docstrings, where constants live, helper naming, assertion
  placement, etc. — log a one-liner observation in
  `../cad-khana/_notes/field-notes.md` (a symlink into the private
  `working-notes` repo; see that file's header). Include the concrete
  example (part name, symptom) so
  future agents can judge whether the pattern repeats.
- **Once 2–3 observations point at the same pattern, promote it
  into cad-khana** (SKILL.md description, SKILL.md body, or
  `../cad-khana/_notes/NOTES.md` as appropriate) and remove the
  matched field-notes entries. Don't promote earlier — one-off accidents aren't
  patterns.
- The threshold exists because SKILL.md is part of every future
  agent's system prompt; every line there costs context, and
  speculative guidance ages badly.
