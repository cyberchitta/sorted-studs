# Module 1: Drum Feeder & Bulk Storage

**Status:** Build from base design
**Alternative:** [Module 1 — Vibrating Hopper](m01-vibrating-hopper.md)
**Dependencies:** None (first stage in physical flow)
**Feeds into:** [Module 2 — Vibratory Singulation](m02-vibratory-singulation.md)

---

## Purpose

Hold unsorted bricks and meter them into the singulation stage at a controlled rate.

## Base Design Reference

From the [Hackaday.io Colour Sorter](https://hackaday.io/project/194170-colour-sorting-machine-for-lego):
a hopper with a chute feeds into a rotating drum. The drum has 3D-printed scoops that
pick up a small number of bricks per revolution and release them onto the vibratory
feeder. The scoop geometry is curved to release bricks gradually rather than dumping
them all at once. A DC motor drives the drum at adjustable speed.

STL files for the drum scoops are available in the project's file downloads (20 files
total including `drum_feeder_scoop_v3.stl`).

## Requirements

- Hold at least 2–3 liters of bricks (~500–1000 pieces)
- Feed rate controllable to avoid overwhelming the singulation stage
- Handle the full range of common brick sizes without jamming

## Key Components

| Component                             | Source                           | Est. Cost    |
| ------------------------------------- | -------------------------------- | ------------ |
| 3D-printed drum + scoops              | Base design STL files            | ~$5 filament |
| DC motor ("280" size)                 | Off-the-shelf                    | ~$3          |
| Motor driver (e.g., L298N or DRV8871) | Off-the-shelf                    | ~$3          |
| Hopper bin                            | 3D-printed or modified container | ~$5          |

## Planned Modifications

- May need larger scoops or wider drum for bigger pieces (Technic beams, large plates)
- Feed rate should be dynamically adjustable via software feedback from the singulation
  stage (e.g., slow down if bricks are piling up downstream)

## Open Questions

- What's the optimal hopper geometry to prevent bridging and jams with mixed brick sizes?
- Should the drum speed be fixed or dynamically adjusted based on singulation performance?
- What's the maximum brick size the current scoop design can handle?
