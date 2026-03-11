# Module 5: Diverter & Bin Output

**Status:** Design TBD — modular and scalable
**Dependencies:** [Module 3 — Scanner](module-03-multi-view-scanner.md) (physical brick input), [Module 4 — Classification](module-04-classification-engine.md) (routing command)
**Feeds into:** Output bins / [Module 8 — Packet Generator](module-08-packet-generator.md) (bag-filling mode)

---

## Purpose

Route each classified brick to the correct output bin based on the classification
result.

## Requirements

- Minimum 6 output bins for v1 prototype
- Target: 18+ bins for mature system; stretch goal 30+ for fine-grained sorting
- Diversion must complete before the next brick arrives
- Gentle handling — bricks shouldn't be damaged or launched out of bins
- "Uncertain" bin always present as one output
- **Modular and replaceable** — the diverter interfaces with the system via a fixed
  entry point and a simple software command. Different diverter designs can be swapped
  without affecting other modules.

## Interface Specification

Regardless of which diverter approach is used, the interface is:

- **Mechanical input:** single entry point where bricks arrive by gravity from the
  scanning station
- **Software input:** `route_to_bin(bin_number)` command from control system
- **Software output:** `diverter_ready` signal indicating the diverter is positioned
  and the next brick can be released

This abstraction allows different diverter designs to be developed independently and
for builders to choose what fits their space and bin-count needs.

## Design Approaches

### Approach A: Rotary Chute (small bin count)

The base Hackaday.io design uses a stepper-driven rotating chute that swings to aim at
bins arranged in an arc. Uses a NEMA 17 stepper with A4988 driver.

- **Pros:** Simple, proven, low part count, STL files available
- **Cons:** Becomes slow or physically large beyond ~10 bins
- **Best for:** v1 prototype, proof of concept

### Approach B: Cascade of Binary Gates

Series of left/right flaps in a binary-tree topology. N levels = 2^N bins.

- **Pros:** Fast switching (each gate only moves a short distance), scales well, each
  gate is a simple servo
- **Cons:** Requires vertical space, more servos and wiring
- **Best for:** Medium bin count (8–32) in a gravity-fed design

### Approach C: Linear Slide

Chute on a linear rail (stepper + belt or lead screw) that positions over target bin
in a row. BrickSortingMachine's "robotic sorting slide" uses this approach.

- **Pros:** Scales linearly to many bins, mechanically straightforward
- **Cons:** Requires horizontal space, potentially slower for distant bins
- **Best for:** Large bin count (18+) when horizontal space is available

### Approach D: Air Jets

Compressed air nozzles blow bricks off a surface into side bins. Used by Daniel West
and the Shape Sifter.

- **Pros:** Very fast, no moving mechanical parts in the divert path
- **Cons:** Noisy, requires compressor, hard to control for variable brick sizes
- **Best for:** High-speed industrial setups; not recommended for home 3D-printed build

### Approach E: Other / Community-Sourced

The diverter is an active area of innovation. We expect to find or develop better
designs as the project progresses. Any design that conforms to the interface
specification above is compatible.

## Open Questions

- Which approach best balances bin count, speed, and build complexity for v1?
- Can we design a modular bin rack where bins are removable/swappable (for packet-mode
  bagging)?
- Is a re-circulation loop practical (uncertain bricks loop back for a second pass)?
- Are there existing 3D-printable diverter designs from other sorting domains
  (electronics components, beads, etc.) that could be adapted?
