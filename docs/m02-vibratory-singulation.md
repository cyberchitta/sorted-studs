# Module 2: Vibratory Singulation Stage

**Status:** Build from base design
**Dependencies:** [Module 1 — Drum Feeder](module-01-drum-feeder.md) (input)
**Feeds into:** [Module 3 — Multi-View Scanner](module-03-multi-view-scanner.md)

---

## Purpose

Separate the stream of bricks from the drum feeder so that exactly one brick at a time
arrives at the scanning station. Widely regarded as the hardest mechanical problem in
any sorting machine.

## Base Design Reference

From the [Hackaday.io Colour Sorter](https://hackaday.io/project/194170-colour-sorting-machine-for-lego):
the current integrated (v1) design uses a V-channel vibratory feeder driven by a small
DC motor. The vibration makes bricks "dance" apart so they travel roughly single-file.
A "slippery slope" gravity slide section after the vibratory channel stabilizes brick
orientation before reaching the sensor.

Key design insights from the build logs:

- Vibration amplitude increases along the channel length so bricks accelerate and
  spread apart naturally
- The channel is V-shaped so bricks self-center regardless of size
- The slippery slope section uses gravity to bring bricks to a consistent speed and
  orientation

**Note on the v2 feeder:** The base project's author (RobG) is separately developing
an improved v2 vibratory feeder using a voice-coil actuator (solenoid driven at ~140Hz
via DRV8871 H-bridge). This design achieves higher frequency, better stop-start
control, and faster transport (~60mm/sec). As of the latest build logs, the v2 feeder
is being developed as a standalone subproject and has not yet been integrated back into
the main machine. We should track its progress and integrate when ready, but the
current v1 feeder is sufficient to get the full system working.

The v2 feeder's key innovations:

- Voice-coil actuator allows stop-start control (bricks stop dead when vibration
  ceases, creating spacing gaps)
- Brass spring strips (~0.8mm thick) at an angled inclination determine transport
  speed and direction
- ~140Hz operating frequency after tuning spring stiffness and angle
- DRV8871 H-bridge driver with PWM amplitude control

## Requirements

- Output: single bricks, spaced apart enough for individual scanning
- Handle bricks that arrive overlapping or lightly interlocked
- Target throughput: ≥1 brick per second
- Tolerant of the full common brick size range

## Key Components (v1, as integrated in base design)

| Component                   | Source                | Est. Cost    |
| --------------------------- | --------------------- | ------------ |
| V-channel feeder            | Base design STL files | ~$3 filament |
| Small DC motor ("280" size) | Off-the-shelf         | ~$3          |
| Motor driver                | Off-the-shelf         | ~$3          |

## Key Components (v2, when available)

| Component                        | Source                     | Est. Cost    |
| -------------------------------- | -------------------------- | ------------ |
| V-channel feeder (revised)       | TBD — from base project v2 | ~$3 filament |
| Solenoid (voice-coil actuator)   | Off-the-shelf              | ~$5          |
| DRV8871 H-bridge driver          | Off-the-shelf              | ~$5          |
| Brass strip (0.8mm, for springs) | Hardware store or online   | ~$5          |

## Planned Modifications

- Add beam-break sensor at exit to confirm single-brick passage and trigger scan
- Software-controlled stop-start: pause vibration during scanning, resume when ready
  (requires v2 feeder or similar actuator with fast response)

## Open Questions

- Is the v1 feeder reliable enough for initial system integration, or should we wait
  for v2?
- What vibration frequency/amplitude works best across the widest range of brick sizes?
- How do we handle "problem pieces" (long Technic beams, flexible hoses, minifig
  accessories) that don't singulate well?
- Should there be a pre-sort sieve stage to separate bricks by rough size before
  feeding?
