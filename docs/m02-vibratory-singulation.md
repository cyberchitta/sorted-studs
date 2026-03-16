# Module 2: Two-Stage Singulation

**Status:** Stage 1 from base design, Stage 2 original design (sorted-studs, adapted
from base project v2 feeder)
**Dependencies:** [Module 1 — Vibrating Hopper](m01-vibrating-hopper.md) (input)
**Feeds into:** [Module 3 — Multi-View Scanner](m03-multi-view-scanner.md)

---

## Purpose

Separate the stream of bricks from the hopper so that exactly one brick at a time
arrives at the scanning station. Widely regarded as the hardest mechanical problem in
any sorting machine.

## Design

Two-stage approach. Stage 1 breaks up clumps using brute-force vibration. Stage 2
transports bricks in a controlled, stoppable flow and releases them one at a time into
the scan station.

### Stage 1 — Clump Breaker (from base design)

Aluminum V-channel driven by a 280-size DC motor with an eccentric weight. High
amplitude, low frequency vibration makes bricks "dance" apart so they travel roughly
single-file. The V-shape self-centers bricks regardless of size.

From the [Hackaday.io Colour Sorter](https://hackaday.io/project/194170-colour-sorting-machine-for-lego)
build logs: vibration amplitude increases along the channel length (hinge at one end,
free end vibrates) so bricks accelerate and spread apart naturally.

### Stage 2 — Singulator (voice-coil feeder)

3D-printed V-channel suspended on angled brass leaf springs, driven by a solenoid
operated as a voice-coil actuator. Adapted from the base project's
[v2 feeder design](https://hackaday.io/project/194170-colour-sorting-machine-for-lego/log/244911-vibe-feeder-2).

The solenoid pulses the channel at the leaf spring resonant frequency (~60–140Hz).
The angled springs convert vertical oscillation into directional transport — bricks
move forward along the channel. When the drive signal stops, bricks stop dead
immediately. This stop-start behavior is the core singulation mechanism.

A beam-break sensor at the channel exit detects each brick as it falls off the end.
On detection, the Arduino pauses the solenoid PWM, halting all bricks still on the
channel. The brick drops into the scan station. After scanning and diverter
positioning complete, the control system resumes the PWM to advance the next brick.

Key design elements:

- **Voice-coil actuator:** A standard push-pull solenoid with the stock steel plunger
  replaced by a lightweight nylon rod. The lighter plunger reduces oscillating mass
  (keeping resonant frequency high) and the looser fit tolerates the slight lateral
  arc of the spring motion without binding.
- **Leaf springs:** Brass strip (0.8mm thick, 15–20mm wide) clamped at an angle
  (`#spring_angle`, starting at 30° from vertical). The angle determines the ratio
  of forward vs vertical motion — steeper angle means faster forward transport.
  Spring free length sets the resonant frequency via f = (1/2π) × √(k/m).
- **Beam-break sensor:** HW-201 IR proximity module with a mirror on the opposite
  side of the drop path, creating a doubled detection beam for reliable triggering
  across all brick sizes.
- **Channel material:** 3D-printed (PETG or ABS) rather than aluminum — lighter mass
  keeps resonant frequency high for a given spring stiffness, and allows integrated
  mounting features for springs and solenoid.

### Why Two Stages

Stage 1 handles the messy input — clumps, overlapping bricks, lightly interlocked
pieces — with aggressive vibration that doesn't need precision. Stage 2 receives
mostly-separated bricks and controls their release one at a time. Separating these
functions avoids compromising either: Stage 1 can be violent, Stage 2 can be precise.

Stage 1 may prove unnecessary if Stage 2's voice-coil feeder handles clump-breaking
well enough on its own. If testing shows that bricks arriving directly from the hopper
singulate reliably on Stage 2 without a clump-breaking stage, Stage 1 can be removed
and the hopper can feed Stage 2 directly (with the hopper exit narrowed from
`#stage1_width` (120mm) to `#channel_width` (90mm)). Start with both stages and
evaluate.

## Requirements

- Output: single bricks, individually released into the scan station
- Handle bricks that arrive overlapping or lightly interlocked
- Target throughput: ≥1 brick per second (limited by scan + divert time, not
  transport speed)
- Tolerant of the full common brick size range
- Stop-start singulation: bricks halt on channel within one vibration cycle when
  drive signal is removed

## Interface with Adjacent Modules

**Input (from Stage 1):** Bricks drop from Stage 1 exit onto Stage 2 entry with
`#stage_drop` (6mm) vertical gap. No mechanical connection between stages.

**Output (to Scanner):** Bricks fall off Stage 2 exit into the scan platform with
`#lap_step` (4mm) drop. Beam-break sensor at the exit signals the control system.

**Control (Module 9):**

- Arduino PWM output → DRV8871 → solenoid (frequency and duty cycle control)
- Beam-break digital input → Arduino (brick detection interrupt)
- Singulation loop: drive → detect → pause → scan → divert → resume

## Key Components

### Stage 1

| Component                   | Source          | Est. Cost |
| --------------------------- | --------------- | --------- |
| Aluminum V-channel (120mm)  | Shop fabricated | ~₹300     |
| DC motor ("280" size)       | Off-the-shelf   | ~₹150     |
| DRV8871 motor driver        | Off-the-shelf   | ~₹250     |
| 3D-printed eccentric weight | Printed         | ~₹30      |
| 3D-printed motor mount      | Printed         | ~₹30      |

### Stage 2

| Component                         | Source        | Est. Cost |
| --------------------------------- | ------------- | --------- |
| 3D-printed V-channel (90mm)       | Printed       | ~₹100     |
| Solenoid (12V push-pull, small)   | Off-the-shelf | ~₹150     |
| DRV8871 H-bridge driver           | Off-the-shelf | ~₹250     |
| Brass strip (0.8mm shim stock)    | Off-the-shelf | ~₹50      |
| Nylon rod (for plunger)           | Off-the-shelf | ~₹20      |
| HW-201 IR sensor module           | Off-the-shelf | ~₹30      |
| Small mirror (~10×10mm)           | Off-the-shelf | ~₹10      |
| 3D-printed spring clamps, housing | Printed       | ~₹30      |

## Open Questions

- What spring angle and free length give the best transport speed across the widest
  range of brick sizes?
- How do we handle "problem pieces" (long Technic beams, flexible hoses, minifig
  accessories) that don't singulate well?
- Should there be a pre-sort sieve stage to separate bricks by rough size before
  feeding?
- What is the minimum pause duration between bricks? This sets the throughput ceiling.
- Does the beam-break reliably detect very small pieces (1×1 tiles, round plates)?
  If not, a narrower beam or second sensor may be needed.
