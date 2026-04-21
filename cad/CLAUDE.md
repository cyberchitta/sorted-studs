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
