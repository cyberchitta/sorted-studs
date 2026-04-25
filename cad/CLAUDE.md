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
