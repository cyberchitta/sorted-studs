# Module 8: Packet Generator

**Status:** To build
**Dependencies:** [Module 7 — Set Coverage](module-07-set-coverage-planner.md), [Module 5 — Diverter](module-05-diverter.md) (for physical bag-filling)
**Feeds into:** [Module 10 — UI](module-10-user-interface.md) (pick lists)

---

## Purpose

For a chosen build (set or MOC), group the required bricks into "packets" — bagged
groups corresponding to stages of the build instructions — so the builder can work
through instructions without searching for parts.

## Requirements

- Map parts to instruction step ranges
- Generate printable pick lists per packet
- Optionally integrate with diverter for physical bag-filling mode

## Functionality

- Step-to-part mapping: determine which parts appear at which instruction steps.
  For official sets, Rebrickable provides part lists but not step-level granularity;
  instruction files or manual annotation may be needed.
- Packet optimization: minimize bags while keeping each to a manageable size
- Pick list generation: part images, quantities, bin locations per packet
- Physical mode: diverter routes bricks directly into labeled bags

## Open Questions

- Where do we get step-level part data? Official LEGO instruction PDFs lack structured
  per-step data. Options: community annotation, CV-based parsing of instruction pages,
  or manual entry.
- For MOCs, instruction formats vary wildly. What's the minimum standard we support?
- Two-pass approach (sort to inventory first, then re-scan for packets) vs. direct
  packet routing in a single pass?
