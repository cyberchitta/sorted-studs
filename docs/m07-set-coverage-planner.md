# Module 7: Set Coverage & Build Planner

**Status:** To build
**Dependencies:** [Module 6 — Inventory](module-06-inventory-rebrickable.md)
**Feeds into:** [Module 8 — Packet Generator](module-08-packet-generator.md), [Module 10 — UI](module-10-user-interface.md)

---

## Purpose

Compare the current brick inventory against official LEGO sets and community MOCs to
surface what's buildable or nearly buildable.

## Requirements

- Ranked list of buildable sets/MOCs by percentage completeness
- Missing-parts manifest for each candidate build, with quantities
- Color-substitution mode ("right shapes, some in different colors")
- Link missing parts to BrickLink/BrickOwl for sourcing

## Functionality

- Pull part lists for any set/MOC from Rebrickable API
- Coverage algorithm: compare required parts against inventory, compute percentage
  match, generate missing-parts list
- Filtering: by theme, year, piece count, completeness threshold
- "Build now" mode: only 100%-complete builds
- "One order away" mode: ≥90% complete with BrickLink shopping list for remainder

## Data Model (simplified)

```
SetParts:   { set_id, part_id, color_id, quantity_needed }
Coverage:   { set_id, pct_complete, missing_parts[], substitutions[] }
```
