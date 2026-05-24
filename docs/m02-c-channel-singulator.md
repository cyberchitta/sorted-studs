# Module 2: C-Channel Singulator

**Status:** Chosen for v1 — CAD in progress (`cad/m02_c_channel_singulator/`).
Adapted from the faceted-rotor C-channel developed by
[basicallysource (Nexus sorter)](https://github.com/basicallysource/sorter)
and documented in its public blog/video archive.
**Alternatives considered (not pursued):**
[Module 2 — Two-Stage Vibratory Singulation](m02-vibratory-singulation.md),
[Module 1 — Drum Feeder](m01-drum-feeder.md),
[Module 1 — Vibrating Hopper](m01-vibrating-hopper.md).
**Dependencies:** none — the extended-wall variant integrates the bulk-input
hopper into the singulator's own lid (see Design).
**Feeds into:** [Module 3 — Multi-View Scanner](m03-multi-view-scanner.md)

---

## Purpose

Separate the stream of bricks into a single-file queue and release them one at a
time into the scan station, using geometry and rotation instead of vibration.

## Design

A spinning circular V-channel — a shallow outer trough (the "C") with a
continuously rotating inner rotor sitting inside it. Bricks dropped into the
channel fall into the V, are pressed against the outer stator wall by the
rotor, and are carried around the circle until they reach an open arc where
they fall off the outer edge one at a time.

Two jobs in one stage:

1. **Singulation** — clumps break apart on the drop into the V because the
   single-file geometry leaves no room for pieces to sit side-by-side.
2. **Conveyance** — the rotor drags pieces around the ring at a controlled
   pace until they reach the exit arc.

### Inverted (outside-ride) geometry

Pieces ride on the **outside** of the rotor, pressed against the outer stator
wall, and fall off the open outer edge at the exit. This is the geometry the
Nexus sorter (basicallysource) settled on after iterating through flat
turntables and centre-drop cone variants. It gives the overhead camera
(Module 3) an unobstructed view of each brick just before release and
eliminates the jam-in-the-centre-hole failure mode of the drop-through
designs.

### Stator (fixed outer trough)

A circular V-shaped trough mounted to the machine frame. The two walls form
the V that guides bricks into single file. One arc of the outer wall is cut
away to form the exit — pieces reaching it simply fall off the edge.

**Optional integrated hopper.** The stator walls can be extended upward and
topped with a lid to turn the stator itself into the bulk input hopper.
Bricks loaded into the lid fall directly onto the rotor. If this variant is
used, Module 1 (vibrating hopper) is not required.

### Rotor (spinning inner cone)

The most important part. A faceted cone — a low-polygon approximation of a
smooth cone, made of flat panels (`#rotor_facet_count`, start at 8) around its
circumference. Two reasons for facets over a smooth cone:

- **Larger contact patch.** A flat panel against a brick's flat underside gives
  far more grip than a line-contact cone, so the rotor drags bricks along
  reliably without needing aggressive speed.
- **Wave motion.** The transitions between facets produce a subtle
  rise-and-fall as each brick passes facet edges, which helps shake apart any
  clumps that survived the initial drop.

Printed in **white** filament. Black bricks against a black rotor are nearly
invisible to the camera; a white rotor gives even reflectance and high contrast
for every LEGO colour including white and black.

Other rotor surface treatments tried in the reference project (any of which may
be worth revisiting if the plain faceted rotor under-grips):

- Spoked grooves across each facet
- Cut-out spirals that nudge pieces outward
- A single rubber O-ring wrapped around the rotor circumference

### Drive

A stepper or geared DC motor with a belt-and-pulley or direct coupling to the
rotor shaft. Speed is tuned so bricks arrive at the exit arc at roughly the
rate the downstream scan + divert loop can absorb (~1 brick/second target).
The drive may be paused-and-resumed to rate-limit the exit, but no resonant
frequency tuning or fine indexing is required.

### Flow control — vision is not part of this module

The Nexus sorter reference uses its per-channel overhead cameras (the same
cameras that classify bricks) to detect pieces approaching the exit and stop
the rotor until the downstream stage is ready. sorted-studs keeps all vision
in [Module 3](m03-multi-view-scanner.md); the singulator never sees or
classifies bricks. Equivalent rate-limiting is achieved with simpler,
camera-free mechanisms:

- **Fixed slow speed.** Tune the rotor RPM so the average release rate matches
  the scan + divert loop. Simplest; no closed-loop control. Relies on queued
  bricks tolerating continuous rotation without jamming.
- **Beam-break stop-on-detect.** An HW-201 IR sensor (same module used by the
  vibratory variant) at the exit arc signals the Arduino to halt the rotor
  when a brick is about to fall. The scan + divert loop signals release to
  resume.
- **Ready-signal from Module 9.** Orchestration holds the rotor paused by
  default and pulses it to release the next piece when the scan station
  reports free.

All three options keep cameras out of this module.

### Exit

No door, no actuator. The outer stator wall is cut away over an arc at the
exit; any brick reaching that arc falls off the edge under gravity into the
drop path to the scan station. Single-file geometry ensures pieces exit one
(or at most a small group) at a time.

### Why this instead of vibratory singulation

- **No resonant tuning.** No leaf springs to size, no solenoid frequency sweep,
  no stop-start control loop.
- **One mechanism, two functions.** Clump-breaking and conveyance happen on the
  same part. Optionally, hopper function too (extended-wall variant).
- **Handles rubber, flexible, and awkward parts.** The reference project
  reports the faceted C-channel handling every piece type, including rubber
  parts that vibratory feeders often mishandle.
- **Quieter.** Continuous rotation vs. resonant vibration.

Tradeoff: larger footprint. At `#rotor_diameter` (starting 150–200mm) plus
stator margin, the C-channel occupies significantly more bench area than a
linear V-channel.

## Requirements

- Output: single bricks, released into the scan station drop path
- Handle bricks that arrive overlapping or lightly interlocked on the drop-in
- Target throughput: ≥1 brick per second at the exit arc
- Tolerant of the full common brick size range (up to `#brick_sphere_dia`)
- Handle rubber, flexible, and awkwardly-shaped pieces without jamming

## Interface with Adjacent Modules

**Input:** Bricks fall into the rotor from above. Either from Module 1's
vibrating hopper (standard), or directly from the extended-wall lid variant
(Module 1 omitted). Drop height and entry arc position are set so pieces land
on the rotor surface, not across the stator wall.

**Output (to Scanner):** Bricks fall off the stator's exit arc and drop into
the scan platform below. The existing scan-station interface parameters apply
(`#exit_width`, `#lap_step`). The exit arc position and drop geometry are
chosen so the fall path lands on the scanner entry.

**Control (Module 9):**

- Arduino PWM or step output → motor driver → rotor motor (speed and optional
  pause/resume)
- Beam-break sensor at exit drop → Arduino (brick-released interrupt, same
  signal shape as the vibratory variant)
- Flow-control scheme selected from the options above; the singulator itself
  exposes only motor and beam-break lines to the control layer, no vision

## Key Components

| Component                              | Source        | Est. Cost |
| -------------------------------------- | ------------- | --------- |
| 3D-printed faceted rotor (white PETG)  | Printed       | ~₹200     |
| 3D-printed stator (V-trough ring)      | Printed       | ~₹250     |
| Rotor shaft + bearings                 | Off-the-shelf | ~₹200     |
| Drive motor (stepper or geared DC)     | Off-the-shelf | ~₹300     |
| Motor driver (A4988 or DRV8871)        | Off-the-shelf | ~₹150     |
| Belt + pulleys (or direct coupling)    | Off-the-shelf | ~₹150     |
| HW-201 IR sensor + mirror (exit)       | Off-the-shelf | ~₹40      |
| 3D-printed frame / mounting ring       | Printed       | ~₹100     |

Optional (extended-wall hopper variant):

| Component                              | Source  | Est. Cost |
| -------------------------------------- | ------- | --------- |
| Extended stator walls + lid (printed)  | Printed | ~₹150     |

## Open Questions

- **Rotor diameter.** Reference project used ~150–200mm; what diameter
  balances footprint against single-file queue stability for our brick size
  distribution? Candidate for `#rotor_diameter` once tuned.
- **Facet count.** Start at 8; does 6 grip better (wider panels) or 12 convey
  more smoothly (less pulse per revolution)? Candidate for `#rotor_facet_count`.
- **V half-angle and rotor cone angle.** How do the stator V angle and the
  rotor sidewall angle combine to seat bricks against the outer wall without
  jamming? Likely reuses `#channel_v_angle` (45°) as a starting point.
- **Gap between rotor and stator walls.** Too small jams small parts; too large
  lets parts slip past single-file. Candidate for `#rotor_stator_gap`.
- **Rotor speed.** What RPM delivers ~1 brick/second at the exit arc for a
  given diameter?
- **Integrated vs. separate hopper.** Is the extended-wall variant reliable
  enough to replace Module 1, or does the added bulk above the rotor cause
  intermittent over-feed?
- **Exit arc angular width.** Wider arc = more bricks can potentially fall at
  once (defeats singulation); narrower arc = higher chance of a queued brick
  riding past. What width releases exactly one brick per transit?
- **Which flow-control scheme wins.** Fixed slow speed vs. beam-break
  stop-on-detect vs. orchestration-pulsed release — which is most reliable
  across brick sizes? Start with fixed slow speed (cheapest), fall back to
  beam-break if it over- or under-feeds.
