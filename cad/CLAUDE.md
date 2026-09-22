# CAD decomposition

Guidance for organizing Build123d / cad-khana assemblies in this repo.

## Principles

1. **One concern per sub-assembly.** Each `assembly.py` validates one
   coherent physical sub-system. Don't mix unrelated parts or checks
   in a single assembly just because they share a module.

2. **Sub-assemblies run standalone.** `khana check <dir>/assembly.py`
   must work without any sibling sub-assembly. `cad/` is a package —
   `khana` imports a package member with `python -m` semantics, so its
   relative imports resolve; plain `python <path>` does not.
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
   fasteners, a standalone-only stand-in) skip as `passed: null`; an
   assertion that is *always* skipped usually means a typo'd path.
   **Read `skipped_counts` on every green run** — a nonzero count is a
   claim that did not look, and it should fall to zero at the root
   that supplies what was missing. A **detail addition is keyed by the
   path of the unit it belongs to**, not bare: a bare key lands the
   part at whatever root the override was applied to, while the unit's
   own claims qualify to `<unit path>.<part>` at every composed root,
   so the two never meet and the claim skips forever. Key it by the
   unit, give it that unit's local `location`, and the same bundle
   works at the unit standalone, at its module root and at the whole
   machine.
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
   bound. Bound it with that same constant — cad-khana's bound
   comparisons carry a 1e-6 tolerance, which covers solver noise
   (~1e-14 distance, ~1e-7 bbox extents). Don't pad the bound
   (`- 0.01`): the pad is a real 0.01 mm the claim stops seeing.

7. **A bare `assert` lives inside the thing it constrains, never at
   module level.** Importing a module must not run a claim. Put each
   surviving bare assert where its knowledge is:

   - a wall, bore or cut-feature bound → the **part function** that
     cuts it, written over the *arguments* so an override is guarded
     too, not just the module default;
   - a span, datum or mirror-drift bound → the **builder** that
     consumes it;
   - a claim owned by a `params.py` → the **consumer's** builder.
     A pure-parameter module has no builder, and an import-time assert
     there fires on anyone reading an unrelated constant out of the
     file.

   Pick the builder the *parent* calls, not the standalone fixture: a
   claim parked in `build_x_standalone()` never runs in a composed
   check. When a unit splits static/rotor, that means the static (or
   rotor) builder, not the standalone.

   Caveat worth knowing: the degenerate memo (`assembly = build_x()`)
   still calls its builder at import, so a builder-hosted assert does
   fire during import — via the memo, not as a module-level statement.
   That import-time effect is the memo's, and is the accepted cost of
   the CLI's degenerate-target contract.

   **Bare stays bare when there is nothing to measure**: bolt-hole and
   wall arithmetic over subtracted negative space, deliberately loose
   parameter bounds, and swept-envelope claims. Convert to
   `assert_distance` / `assert_scalar` / `assert_tangent_contact` only
   where the claim is about parts that actually coexist in one
   assembly — and see "What a green `khana check` does not mean" below
   for the pose trap that catches the rest.

## Design systems: builders never set colour

**This overrides cad-khana's SKILL.md**, which says colours are set at
the `with_part` placement. In this repo they are not — that guidance is
for a consumer with one palette, and we have seven.

A design system is a **value threaded as a parameter**, never module
state:

- **Sub-assembly builders take no colour argument and set no
  `color=`.** They are palette-free: same geometry whatever the schema.
- **Unit roots take `schema: str = DEFAULT_SCHEMA`** and paint the
  finished tree once on the way out —
  `paint(a, design_system(schema))` from `cad/design_systems.py`. The
  unit root is where the module's tag dict lives, so it is the only
  level that can resolve a leaf name to a colour.
- **Entry points own env resolution.** `schema_from_env("M03")` reads
  `SORTED_STUDS_DS_<MODULE>` → `SORTED_STUDS_DS` → default, and is for
  render/export scripts only. A declaration module that reads the
  environment is not a function of its arguments.
- **`paint` carries the coverage check.** Every leaf must be tagged;
  untagged parts are collected and reported in one `KeyError` naming
  all of them. Don't hand-roll a `placed_parts` walk asserting tag
  coverage — that is what the four retired module-level asserts did.
- **A part tagged `None` keeps the colour its builder set.** `None`
  means "outside the design system", so paint leaves it alone rather
  than overwriting with `None`. Two parts rely on this: m03's `acrylic`
  and m05's `top_acrylic` carry translucent baseColors with a real
  alpha channel that the shape-truth GLB renders through.
- **Colour that isn't a plain tag read goes in the module's
  `color_for` hook**, not into the builder. m05 ramps basket luminance
  off `n_size`, recovered from the leaf name (`basket_090_12` → 12).

Consequence to know: a **sub-assembly's standalone `assembly` is
uncoloured**, because paint happens at the unit root. Nothing regresses
— `mechanism.json` records no colour and every export goes through a
unit root — but `khana view <sub>/assembly.py` shows default-coloured
parts. View the unit root to see the palette.

Why a walk and not a parameter threaded down every builder: the tag
dicts key on **leaf part name**, and inside the tree
`PlacedPart.name` is already the leaf (`placed_parts` is what
qualifies it into a dotted path), so the walk has its key at every
level with no plumbing in between.

**`@cache` on a builder needs the schema passed explicitly at every
call site, including the degenerate `assembly = build_x(DEFAULT_SCHEMA)`
memo.** `functools.cache` keys on the call signature, so `build_x()`
and `build_x(DEFAULT_SCHEMA)` are two entries building identical
geometry twice. The symptom is invisible — same output, double the
work; `cache_info().misses` is the only tell.

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
            assembly.py      # build_basket_ring() -> Assembly — declarations
            printability.py  # command script: the unit's inspect() batch
        ramp_mechanism/
            __init__.py
            assembly.py      # build_* builders — declarations
            role_sweep.py    # command script: the three-role check + inspect suite
    m03_scanner/
        assembly.py          # product declarations
        check_cones.py       # CHECK module: probe fixture + external claims
        frame_sweep.py       # command script: 12-frame regression sweep
```

**Three file kinds, and the boundary is what the file does.**

- **Declaration modules** (`assembly.py`, `animated_assembly.py`) —
  pure. Parts, builders, joints, claims. They call nothing effectful:
  no `check()`, no `inspect()`, no export, no `__main__` block. The
  module's public surface is its named factories returning `Assembly`.

  **A module binds `assembly` exactly once, and only as the degenerate
  memo** — `assembly = build_x()`, last line. Everything above it is a
  factory. A sub-assembly's standalone fixture (its subtrees plus the
  stubs and claims that only make sense outside a parent) is a factory
  too: name it `build_<unit>_standalone()`. Nothing imports it — the
  parent calls the product builders directly — but making it a
  function is what keeps the module import-time-effect-free, and it
  gives `khana check <file>:build_x_standalone` a name to address.
  Watch for the two shapes that aren't a literal `x = x.with_…`
  rebind: sequential module-level bindings (compose, then a long
  assert chain) and a module-level `for` loop appending asserts. Binds
  *once* is the invariant; "no `x = x.`" is only a proxy for it.
- **Check modules** (`check_*.py`) — declaration modules whose purpose
  is verification. They import product factories, compose a fixture
  (probe geometry, a merged assembly), declare the claims that fixture
  exists to test, and expose the result as `assembly()`. **Nothing ever
  imports a check module** — that rule is what keeps probe solids out of
  every export. A claim internal to one unit stays in that unit; only a
  claim about the *interaction* of a unit with something it doesn't own
  earns a check module.
- **Command scripts** (`printability.py`, `role_sweep.py`,
  `frame_sweep.py`, `render*.py`, `export_glb*.py`) — orchestration,
  run for effect, never imported. Reach for one only when the CLI
  genuinely can't address the work: an `inspect()` batch, a family of
  members the CLI can't name (a required factory argument, a float),
  a multi-frame sweep. Orchestration only — no logic that matters.

Imports are relative (`.params` at unit level, `..params` in
sub-modules); no `sys.path` manipulation anywhere. Use underscores in
directory names so Python can import them normally; hyphens block
package imports and force `importlib` workarounds.

## Running checks

`cad-khana` and `chitra-cad` are installed (editable) in the repo's
`uv`-managed venv at `.venv/`. Invoke the CLI directly:

```
# import-model verbs — take <module-path>[:<factory>], import it,
# resolve one member, run one verb. Nothing in the module executes.
.venv/bin/khana check  <path/to/assembly.py>   # diagnostics + assertions
.venv/bin/khana export <path/to/assembly.py>   # write STL + STEP
.venv/bin/khana view   <path/to/assembly.py>   # push to the OCP viewer
.venv/bin/khana draw   <path/to/assembly.py>   # ortho + iso engineering drawings

# exec-model verb — runs a command script for effect
.venv/bin/khana run    <path/to/printability.py>
```

`uv run khana …` works too. `khana build` is **deprecated** and
retires in the next cad-khana release — it is `run` plus exports, and
nothing in this tree needs it any more.

**Member resolution.** With no `:factory`, `check`/`export`/`view`/
`draw` resolve the name `assembly` — a callable is called, a bare
`Assembly` value is accepted as the degenerate (but still supported)
form, which is what most `assembly.py` files here expose. Naming a
factory (`assembly.py:build_lid`) calls it with its defaults. A target
that resolves nothing exits **2** and lists the module's public
`-> Assembly` factories on stderr — read that error rather than
grepping for candidates.

**Where output lands.** The target owns its default out:
`<module-dir>/outputs` for a unit's `assembly.py`, and
`<module-dir>/outputs/<slug>` for anything else, where the slug is the
module stem (`check_cones.py` → `outputs/check_cones/`,
`single_floor.py` → `outputs/single_floor/`). So co-located targets
never overwrite each other's `mechanism.json`, and no `--out` flag is
needed. An explicit `--out` still wins and is taken cwd-relative.

**A green `khana check` no longer covers printability.** Since the
Phase B decomposition (2026-07-26) the `inspect()` calls live in a
per-unit `printability.py` run separately. After a geometry change,
for the affected unit run **both**:

```
.venv/bin/khana check <unit>/assembly.py
.venv/bin/khana run   <unit>/printability.py     # if the unit has one
```

then re-run `khana check` on the enclosing top-level `assembly.py`
before claiming the slice is done. Three units carry a parametrized
suite whose members the CLI cannot address, and there `khana check`
on `assembly.py` evaluates **one** member only — run the command
script for the rest:

| unit | `khana check assembly.py` covers | command script covers |
|---|---|---|
| `m05_diverter/ramp_mechanism` | the `middle` role | `role_sweep.py` — all three roles + 11 inspects; `khana check check_fall_path.py` — flat 80 mm stick probes at 12 yaws down the middle floor's column at rest — the pass-through fall path (~10 s) |
| `m05_diverter/single_floor.py` | the **base floor's** basket layout, held over the declared `stack_turn` (181 poses, ~30 s) | `khana run floor_layout_sweep.py` — all four floors' layouts against the same turning column side (~1 m 45 s). The tower declares no basket-vs-rotating pair, so without this three layouts are checked at no azimuth |
| `m05_diverter` (stack turn) | the tower at azimuth 0 only — `build_m05` declares no motion on purpose, because `cad/assembly.py` composes it and a motion in the factory lands on every whole-machine check | `khana check check_stack_turn.py` — the same tower with `stack_turn` declared on top; 28 claims move (drive train vs the turning column/ramp/trap, each floor's column + trap servo vs the bearing above, `top_acrylic` vs the top column), ~1 m 30 s |
| `m05_diverter` | every floor's baskets at rest, each in its own layout | `khana check check_basket_removal.py` — every distinct basket of every floor layout drawn out along its own axis, stepped past the columns and vertex brackets (~17 min; run it in the background) |
| `m03_scanner` (animated) | frame `t = 0` | `frame_sweep.py` — all 12 sampled frames |
| `m03_scanner` (cones) | the cone-free product assembly | `khana check check_cones.py` — the sightline claims |
| `m03_scanner` (detail) | the cheap bodies only — the six drive-foot bolt claims skip as `absent_part` | `khana check check_detail.py` — the same tree with the detail bundle applied, so those six evaluate against the hollow V-slot rail |
| `cad/` (top level) | cross-unit interference, anchor coincidence, the gantry's rests on m05 | `khana check check_turret_envelope.py` — the chain vs the turret's loaded-platform sweep (a probe solid); `khana check check_service_access.py` — a hand-wide reach corridor to IMAGE from ±X, clear of every unit; `khana check check_detail.py` — the whole machine with m03's detail bundle applied, the only run in which every foot-bolt claim evaluates in machine coordinates (slow; run it in the background) |
| `m02_c_channel_singulator` | every rotor claim at spin angle 0 only — the `rotor` joint is declared and no motion drives it, because the chain composes the unit three times | `khana check check_rotor_turn.py` — one facet period (36°, 1° steps); 15 claims move, ~25 s |
| `m02_chain` | the product: stages, ramp, gantry and its joints — no posts, no deck | `khana check assembly.py:build_chain_standalone` — the same plus stand-ins for m05's two posts and deck, the rests and sockets asserted against them; the target to `view` when you want the gantry on its posts. `khana run printability.py` — the five printed joint bodies (`joints.py`); the LOAD ramp is inspected nowhere |

**Use the CLI, not bare `python`, for any command script.** The CLI
defers failures to its own boundary so every call runs and every JSON
on disk is fresh, then exits nonzero once; a bare interpreter keeps
raise-on-first-failure and aborts the run, leaving the rest of the
diagnostics stale at their previous values while reading as current.
This is a correctness difference, not a convenience: a multi-role or
multi-frame sweep under bare `python` reports only its first red.

The same deferral means **`check()` and `inspect()` return normally on
failure under the CLI** — there is no `SystemExit` to catch. Read
failures off the result (`result.diagnostics.assertions`, `passed is
False` — `None` is a skipped assertion, not a failure); never wrap the
call in `try/except SystemExit` and never re-`evaluate()` the assembly to
find out. The m03 9-frame sweep (now `frame_sweep.py`) had exactly that
`except`, and after the deferral shipped it printed *"all 9 frames pass"*
on runs the CLI was failing.

A command script does not need to read failures off the result at all:
the CLI's roll-up already names every one and sets the exit code. Only
reach into `result.diagnostics.assertions` when the script has to
*branch* on a failure — which, in this tree, nothing does.

## What a green `khana check` does not mean

cad-khana's `SKILL.md` §"What a green check does not mean" carries the
tool-level form of this list. Below is what each class cost *here* —
keep the stories, and when a class stops being a blind spot in the tool,
retire it there first.

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
  implicit worst-case claim. The m05 trap's upward-intrusion check was
  asserted at its 60° open pose alone, because 60° bounded the envelope
  *at the then-current* trap length and angle. Write that claim beside
  the assert, or it expires silently when the parameter moves.

  The converse bites when *promoting* a bare assert to an assertion. A
  bare assert over constants is frozen at whatever pose its numbers
  came from; an `assert_distance` is re-evaluated at **every** pose the
  sweeps build. Promoting a rest-pose claim therefore reddens frames
  that were always fine — the homing parts' drop-block clearance
  measured −22.6 mm at `t=0.0875` when first promoted, as the dumping platform tips
  through the same Z band without ever touching. Nothing is wrong
  there; the claim was only ever true at rest tilt. Before converting,
  ask which poses the claim was ever true at, scope it with `during=`
  to exactly those (`build_turret`'s `*_drop_block_gap_*`, 2026-09-21),
  and run the sweep — not just the unit's own `khana check`.

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
`working-notes` skill; this repo has no local exception, including on
the draft-first default.

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
