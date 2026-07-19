# Module 5: Diverter & Bin Output

**Status:** CAD in progress (`cad/m05_diverter/`)
**Dependencies:** [Module 3 — Scanner](m03-multi-view-scanner.md) (physical brick input), [Module 4 — Classification](m04-classification-engine.md) (routing command)
**Feeds into:** Output bins / [Module 8 — Packet Generator](m08-packet-generator.md) (bag-filling mode)

---

## Purpose

Route each classified brick to the correct output bin based on the classification
result.

## Requirements

- Bin count: 18 (3-floor build) at v1; 24 (4-floor) committed for the
  current CAD pass; stretch up to 30+ for fine-grained sorting
- Diversion must complete before the next brick arrives
- Gentle handling — bricks shouldn't be damaged or launched out of bins
- "Uncertain" bin always present as one output
- Modular and replaceable — the diverter interfaces with the system via a
  fixed entry point and a simple software command

## Interface Specification

- **Mechanical input:** single drop point at the top of the tower, into the
  central column of the top floor. The scanner platform (Module 3) sits
  directly on the diverter's clear-acrylic top deck and feeds bricks into
  the central aperture.
- **Software input:** `route_to_bin(bin_number)` command from the control
  system (see [Module 9](m09-control-orchestration.md))
- **Software output:** `diverter_ready` signal indicating the diverter is
  positioned and the next brick can be released

## Committed architecture — Hex Tower Cascade

The current build is a hexagonal tower of stacked floors with a central
rotating column that delivers bricks to one of six baskets per floor. Bin
count = floors × baskets/floor.

### Tower structure

- **4-floor stack** at world z = 0 / 220 / 440 / 660 mm (base at the
  bottom). Each floor carries a hex ring of 6 baskets around a central
  rotating column. Bricks enter above the top floor and fall in −Z;
  per-floor trap doors decide whether the brick drops at this floor or
  falls through to the next.
- **6 through-columns** at every inner and outer hex vertex, 2020
  aluminum extrusion, full tower height. Splits each radial rail into
  inboard and outboard segments terminating at vertex brackets; both
  segments load the columns in compression rather than forcing the
  inner-hex ring to work as a bending beam under the ~20 kg/floor basket
  live load.
- **Clear-acrylic top deck** — OD 660 mm, 10 mm cast acrylic, central
  Ø150 mm aperture for the brick drop. Rests on the through-column tops.
  The scanner platform (and singulator) sit on this disc as the shared
  M2 → M3 → M5 mounting surface.

### Central rotating column

- 4 stacked column sub-assemblies (one per floor) coupled end-to-end via
  a print-only **bas-relief interlock** at each joint (cylindrical bumps
  at cardinal azimuths on the lower flange face fit into matching
  depressions on the upper flange — positive rotational engagement under
  gravity preload, no torque-bearing bolts).
- Each floor's column carries a +X exit opening at a fixed azimuth in its
  own frame. Default install: all openings aligned (single rotation
  delivers to whichever floor releases its trap). Spiral and alternating
  install rotations are reserved as options if structural or operational
  reasons emerge.

### Bearings & drive

- **Top-mounted bearings, partial-ring grip.** Each floor's
  spider+bearing sits at its own bracket level and engages the rotating
  collar of the *lower* column through a ~270° partial ring (the +X exit
  opening cuts through the engagement surface). Mechanically sound at
  low RPM / low load: the bearing contact patch reshuffles as the collar
  rotates, never losing radial constraint.
- **5 bearing slots total**: motor-driven turntable + ring gear at the
  base, three passive sleeve bearings between floors, one ceiling sleeve
  under the hopper / feed interface.
- **Single off-axis motor at the base**, gear-coupled to a ring gear on
  the base column's bottom collar. Two implementation candidates: an
  integrated slewing-ring (combined bearing + ring gear) or a cheap
  commercial lazy-susan turntable + bolt-on printed ring gear. No central
  drive shaft — bricks fall through the column axis.

### Per-floor ramp + trap

The ramp is inherently a **rotating trap door**: rotation selects *which*
basket on the floor; up/down selects *whether* this floor catches the
brick at all. Both degrees of freedom are required — a ramp that only
rotates would catch every brick on the top floor; a trap that only
raises would need one per bin.

Trap actuator is deferred. The bounded options are (a) one servo per
floor mounted on the rotating column, fed via a slip ring at the base,
or (b) stationary cam / solenoid that engages the trap only at a release
azimuth. Both are compatible with the rest of the architecture.

### Why this geometry

- **Compact footprint.** Vertical scaling lets us hit 18+ bins in
  workshop-scale floor area, where horizontal arrangements (rotary
  chute, linear slide) would not fit.
- **Modular floors.** Floors are identical (modulo base / top variants),
  so the bin count is a build-time choice without redesign.
- **Fails safe.** A stuck ramp leaves the trap closed → brick passes
  through to the next floor → ultimately the bottom catches it.
- **Symmetric column loading.** Every radial gets both ends backed by a
  through-column; no bending-beam regimes.

The live design state, open mechanical detail (column-to-column joint
sub-options, base drive-train interface choice, trap actuator), and
per-sub-assembly progress are tracked in
`../_notes/cad/m05_diverter/NEXT_STEPS.md` — a working note, not tracked in
this repo.
Full earlier-design narrative is in
[`../cad/m05_diverter/hex-cascade.md`](../cad/m05_diverter/hex-cascade.md).

## Alternatives considered

These were evaluated before settling on the hex tower cascade. Kept here
so the choice is visible and traceable.

### A. Rotary Chute (small bin count)

The base Hackaday.io design uses a stepper-driven rotating chute that
swings to aim at bins arranged in an arc.

- **Pros:** simple, proven, low part count, STL files available.
- **Cons:** becomes slow or physically large beyond ~10 bins; horizontal
  footprint grows with bin count.

### B. Cascade of Binary Gates

Series of left/right flaps in a binary-tree topology. N levels = 2^N bins.

- **Pros:** fast switching, scales well, each gate is a simple servo.
- **Cons:** requires vertical space; more servos and wiring than the hex
  tower at equivalent bin count.

### C. Linear Slide

Chute on a linear rail (stepper + belt or lead screw) that positions over
a target bin in a row. BrickSortingMachine's "robotic sorting slide" uses
this approach.

- **Pros:** scales linearly to many bins, mechanically straightforward.
- **Cons:** requires horizontal bench space — the constraint that pushed
  sorted-studs toward vertical stacking in the first place.

### D. Air Jets

Compressed air nozzles blow bricks off a surface into side bins. Used by
Daniel West and the Shape Sifter.

- **Pros:** very fast, no moving mechanical parts in the divert path.
- **Cons:** noisy, requires compressor, hard to control for variable
  brick sizes; not a fit for a home / workshop build.

### E. Other / Community-Sourced

The diverter is an active area of innovation in the hobbyist sorter
community. Any design that conforms to the interface specification above
remains compatible if a better approach surfaces.

## Open Questions

- Can we design a modular bin rack where baskets are removable/swappable
  (for packet-mode bagging)?
- Is a re-circulation loop practical (uncertain bricks loop back for a
  second pass)?
- Trap-actuator pick: per-floor servo + slip ring vs stationary
  cam/solenoid at a release azimuth?
- Column-to-column joint: bas-relief alone, or add outboard castellation
  / bolts as a belt-and-braces layer?
