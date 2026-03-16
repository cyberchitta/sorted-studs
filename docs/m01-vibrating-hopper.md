# Module 1: Vibrating Hopper

**Status:** Original design (sorted-studs)
**Dependencies:** None (first stage in physical flow)
**Feeds into:** [Module 2 — Two-Stage Singulation](m02-vibratory-singulation.md)

---

## Purpose

Hold unsorted bricks and meter them onto the Stage 1 clump-breaking channel
at a controlled rate.

## Design

A box with an angled floor and a vibration motor. Bricks settle to the low
side of the floor and feed out through a slot at the bottom edge. The
vibration motor agitates the floor so bricks don't bridge or jam at the
slot. Feed rate is controlled by motor duty cycle — higher duty cycle means
more bricks per second.

### Construction

The hopper body is built from bent sheet metal or a plastic storage
container. The only critical dimension is the exit slot, which must match
the Stage 1 channel entry width (120mm). The floor is angled at 15–20° so
bricks gravity-feed toward the exit without requiring vibration to move
them — vibration only prevents jamming.

A 280-size DC motor with a heavy eccentric weight mounts to the outside of
the hopper floor. The vibration is slow and strong — enough to dislodge
bridged bricks without launching them out of the box.

The exit slot height controls how many bricks can leave at once. A slot
height of `#brick_max_height` (38mm) allows one brick-layer deep to exit.
An adjustable gate (a sliding plate over the slot) allows tuning this
during testing.

### Key Properties

- Holds at least 2–3 liters of bricks (~500–1000 pieces)
- Feed rate adjustable via PWM duty cycle
- Exit slot width matches Stage 1 entry: `#stage1_width` (120mm)
- Exit slot height adjustable, default `#brick_max_height` (38mm)
- Floor angle 15–20° toward exit

### Interface with Adjacent Modules

**Output (to Stage 1):** Bricks exit the slot and drop onto the entry end
of the Stage 1 clump-breaking channel. No mechanical connection. The hopper
is positioned above and behind Stage 1 so bricks fall freely onto it.

## Key Components

| Component                    | Source        | Est. Cost |
| ---------------------------- | ------------- | --------- |
| Container / bent sheet body  | Off-the-shelf | ~$5–10    |
| DC motor ("280" size)        | Off-the-shelf | ~$3       |
| Motor driver (e.g., DRV8871) | Off-the-shelf | ~$5       |
| 3D-printed eccentric weight  | Printed       | ~$1       |
| Sliding gate plate           | Sheet offcut  | ~$0       |

## Open Questions

- What floor angle prevents bridging across the widest range of brick sizes?
- Is a single exit slot sufficient, or would a wider opening with a comb/grate
  work better for flat plates?
- Does the hopper need internal baffles to prevent all bricks rushing to the
  exit at once?
