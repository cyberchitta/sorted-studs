# M02 — Singulator Build

## Overview

Two-stage singulation. Stage 1 (clump breaker) uses the original eccentric
motor design on an aluminum V-channel. Stage 2 (spacer/singulator) uses a
solenoid-driven voice-coil design on a 3D-printed V-channel with leaf
springs and beam-break singulation control.

## Parts

### Part 1: Stage 1 Channel — Aluminum (shop fabricated)

1.5mm aluminum sheet, single 90° bend along full length.

**Finished dimensions (give to shop):**

- Length: 400mm
- 90° bend along full length, centered
- Each wing: 84.9mm along face from inside of bend to top edge
- Opening width across top (inside measurement): 120mm
- Deburr all edges

**After receiving:** Drill two M3 through-holes on one outer face for
motor mount plate. Position by marking through the printed mount plate's
slots.

### Part 2: Stage 1 Motor Mount Plate ×1 — 3D Printed

- Flat plate, 60mm × 40mm × 4mm
- Motor pocket on outer face: 25.5mm dia, 2mm deep
- Two M3 clamp holes flanking pocket (for zip-tie saddle)
- Two elongated slots: 3.4mm × 10mm, for M3 mounting bolts
- Motor center at `#motor_offset` (15mm) from channel centerline

**Print orientation:** Flat (pocket face up).

### Part 3: Stage 1 Eccentric Weight ×1 — 3D Printed

- Solid cylinder, 15mm dia, 10mm long
- Bore offset 3mm from center, sized for press-fit on 280 motor shaft

**Note:** Eccentric dimensions are starting points. Expect to iterate
by reprinting with different mass and offset based on testing.

### Part 4: Stage 2 Channel — 3D Printed

Single-piece V-channel with integrated mounting features.

- Length: 400mm
- V-angle: `#channel_v_angle` (45° half-angle)
- Opening width across top (inside measurement): `#channel_width` (90mm)
- Wall height above V-bottom: `#channel_wall_height` (40mm)
- Wall thickness: 2.5mm
- Integrated spring mounting tabs at each end: flat faces angled at
  `#spring_angle` (30°) from vertical, with M3 threaded inserts or
  nut traps for spring clamp bolts
- Solenoid mounting boss on underside near free (downstream) end:
  clearance hole for solenoid body, M3 bolt holes for clamp

**Print orientation:** Upside-down (opening face down) for best wall
quality. Print in two halves and bond if bed is under 400mm, with a
lap joint at the midpoint.

**Material:** PETG or ABS preferred over PLA for vibration fatigue
resistance.

### Part 5: Leaf Springs ×2 — Brass Strip

- Material: brass shim stock, 0.8mm thick
- Width: 15–20mm
- Free length: 20–30mm (tune to hit target resonant frequency)
- Each spring clamped between the channel spring tab and a matching
  3D-printed clamp plate using M3 bolts
- Lower end clamped to frame bracket at `#spring_angle` (30°) from
  vertical

**Note:** Spring dimensions are starting points. The target resonant
frequency is `#resonant_freq` (60–140Hz). Tune by adjusting free
length: shorter = stiffer = higher frequency. Width can also be
increased up to 20mm. If 0.8mm brass is too soft, try 1.6mm (two
layers stacked).

### Part 6: Spring Clamp Plates ×4 — 3D Printed

- Small plates (25mm × 25mm × 4mm) with two M3 through-holes
- Two clamp the springs to the channel tabs (upper)
- Two clamp the springs to the frame brackets (lower)

### Part 7: Solenoid + Nylon Plunger

- Small 12V push-pull solenoid (15–20mm stroke)
- Remove stock steel plunger
- Replace with nylon rod: turn or file to ~0.5mm smaller than bore
  diameter, length to suit stroke + engagement
- Loose fit is intentional — allows lateral flex as springs arc

**Sourcing (India):** Search "12V push pull solenoid small" on
Amazon.in or IndiaMart. The exact model is not critical — any small
12V solenoid with spring return and ~10–20mm stroke will work. The
plunger gets replaced anyway.

### Part 8: Beam-Break Sensor — HW-201 + Mirror

- HW-201 IR proximity/obstacle sensor module
- First-surface or ordinary small mirror (~10×10mm)
- 3D-printed housing that positions the HW-201 on one side of the
  brick drop path and the mirror on the opposite side, reflecting the
  IR beam back to the sensor
- Mount at the downstream exit of the Stage 2 channel, positioned so
  a falling brick breaks the beam

**Note:** The dual-beam (emitter → mirror → receiver) arrangement
doubles detection width compared to the bare module. Housing design
should allow vertical adjustment to align with the drop path.

## Module-Specific Parameters

| Variable            | Value    | Notes                                            |
| ------------------- | -------- | ------------------------------------------------ |
| `#stage1_length`    | 400mm    | Stage 1 (clump breaker) channel length           |
| `#stage2_length`    | 400mm    | Stage 2 (singulator) channel length              |
| `#stage_drop`       | 6mm      | Vertical drop at Stage 1 → Stage 2 handoff       |
| `#motor_280_dia`    | 25mm     | 280-size DC motor body diameter (Stage 1)        |
| `#motor_280_length` | 35mm     | 280-size DC motor body length (Stage 1)          |
| `#motor_offset`     | 15mm     | Motor center offset from channel centerline (S1) |
| `#spring_angle`     | 30°      | Leaf spring angle from vertical — tune for speed |
| `#spring_thickness` | 0.8mm    | Brass leaf spring stock thickness                |
| `#spring_width`     | 15mm     | Leaf spring width — increase up to 20mm to tune  |
| `#spring_free_len`  | 25mm     | Leaf spring free length — starting point         |
| `#resonant_freq`    | 60–140Hz | Target resonant frequency range                  |
| `#solenoid_voltage` | 12V      | Solenoid drive voltage                           |
| `#solenoid_current` | 250mA    | Solenoid current limit (set on DRV8871)          |

## Assembly

1. **Stage 1:** Bolt motor mount plate to Stage 1 aluminum channel.
   Press-fit eccentric weight onto 280 motor shaft. Seat motor in mount
   plate pocket, secure with zip-tie. Mount on frame with hinge at
   upstream end, free end supported by springs or rubber mounts.

2. **Stage 2 spring brackets:** Bolt two frame brackets to the frame at
   the Stage 2 position, with spring clamping faces angled at
   `#spring_angle` (30°) from vertical.

3. **Leaf springs:** Clamp one end of each brass spring strip to the
   channel's integrated spring tabs using clamp plates and M3 bolts.
   Clamp the other end to the frame brackets. The channel should now
   hang from the springs and oscillate freely when tapped.

4. **Solenoid:** Mount solenoid body to the _frame_ (not the channel)
   near the downstream end of Stage 2. Insert nylon plunger. The
   plunger tip bears against the channel underside so the solenoid
   drives the channel relative to the fixed frame.

5. **Position stages:** Mount Stage 1 upstream, Stage 2 downstream,
   with `#stage_drop` (6mm) vertical gap at the handoff. Stage 2 exit
   feeds into the scan platform with `#lap_step` (4mm) drop.

6. **Beam-break sensor:** Mount the HW-201 + mirror housing at the
   Stage 2 exit, spanning the drop path. Adjust vertical position so
   the IR beam crosses where bricks fall. Connect to Arduino digital
   input.

7. **Wiring:** Connect Stage 1 motor to DRV8871 driver and Arduino PWM
   output. Connect Stage 2 solenoid to a second DRV8871 and Arduino PWM
   output. Connect beam-break sensor to Arduino digital input. See
   Module 9 for control logic.

## Tuning Procedure

1. **Stage 1:** Adjust motor PWM duty cycle until bricks separate
   without launching out of the channel. This is coarse tuning — exact
   frequency is not critical.

2. **Stage 2 frequency:** Set PWM frequency to ~60Hz, low duty cycle
   (~10%). Place bricks on channel. Slowly increase frequency while
   watching for resonance — bricks will visibly jump more at the
   resonant frequency.

3. **Stage 2 amplitude:** At resonance, increase duty cycle (pulse
   width) to increase amplitude. Target: bricks transport at ≥60mm/sec.

4. **Spring tuning:** If resonant frequency is too low, shorten spring
   free length or increase spring width. If bricks bounce vertically
   instead of moving forward, increase `#spring_angle`.

5. **Beam-break verification:** Drop bricks past the sensor and confirm
   reliable triggering across a range of brick sizes, including small
   pieces (1×1 tiles).
