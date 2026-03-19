# Module 3: Multi-View Scanning Station

**Status:** Original design (sorted-studs innovation)
**Dependencies:** [Module 2 — Vibratory Singulation](module-02-vibratory-singulation.md) (input)
**Feeds into:** [Module 4 — Classification Engine](module-04-classification-engine.md) (image data), [Module 5 — Diverter](module-05-diverter.md) (physical brick)

---

## Purpose

Capture high-quality images of each brick from multiple angles simultaneously, in a
single camera frame, using mirrors. This is a core innovation of sorted-studs — most
existing sorters use a single top-down view, limiting their ability to distinguish parts
with identical profiles from one angle.

## Design Rationale: Why Multiple Mirrors

Most existing sorters photograph bricks from one angle (typically top-down). This fails
to distinguish many common parts: a 2×4 plate and 2×4 tile look identical from above;
a standard brick and a Technic brick differ only in side holes; slopes and curves are
only visible in profile.

The BrickSortingMachine project proved the concept of using a single camera + single
mirror to get two views simultaneously. sorted-studs extends this to multiple mirrors.

Advantages of multiple mirrors over multiple cameras:

- **Perfect synchronization.** All views captured in one exposure — no timing issues.
- **One pipeline.** Single image to preprocess, one camera driver, one set of exposure
  settings.
- **Lower cost.** One camera instead of two or three.
- **Consistent color.** Same sensor captures all views under identical lighting, so
  color classification is consistent across views.

## Design

```
             ┌─────────────────────────────┐
             │        CAMERA (top-down)     │
             │         pointed down         │
             └─────────────┬───────────────┘
                           │
          ┌────────────────▼────────────────┐
          │            LIGHT BOX            │
          │                                 │
          │  ┌─Mirror─┐  [TOP]  ┌─Mirror─┐ │
          │  │   B     │  VIEW  │   A     │ │
          │  │  ~45°   │        │  ~45°   │ │
          │  │  left   │        │  right  │ │
          │  │  side   │        │  side   │ │
          │  └─────────┘        └─────────┘ │
          │                                 │
          │  ┌─ ─ ─ transparent section ─ ┐ │
          │  │      gravity slide          │ │
          │  └─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┘ │
          │                                 │
          │         ┌─Mirror C─┐            │
          │         │  ~45°    │            │
          │         │  bottom  │            │
          │         │  view    │            │
          │         └──────────┘            │
          └─────────────────────────────────┘
```

The camera sees one composite image: direct top-down view in center, flanked by
reflected side views, with optional bottom view reflected up from below.

A 12MP sensor split across four views still yields ~2000×1500 per view — far more than
the 224×224 or 384×384 that classification models typically require.

## Requirements

- At least three simultaneous views: top-down, left profile, right profile
- Stretch goal: fourth bottom/anti-stud view via transparent section + mirror below
- All views in a single exposure
- Consistent, diffused, flicker-free lighting isolated from ambient
- ≥500×500 usable pixels per view
- Exposure fast enough to avoid motion blur at operating speed

## Key Components

| Component                                              | Source                        | Est. Cost          |
| ------------------------------------------------------ | ----------------------------- | ------------------ |
| iPhone (XS or later; iPhone 12+ recommended)           | Camera capture + ML inference | ~$200 (used phone) |
| First-surface mirrors (×2–3)                           | Optics supplier or Amazon     | ~$15               |
| 3D-printed light box enclosure                         | Original design               | ~$5 filament       |
| 3D-printed mirror mounts (adjustable, with set screws) | Original design               | ~$3 filament       |
| LED strip (daylight ~5000K, high-CRI) + diffuser       | Off-the-shelf                 | ~$10               |
| Clear acrylic/polycarbonate sheet (for bottom view)    | Hardware store                | ~$5                |
| VL53L1X time-of-flight sensor (×1–2)                   | Off-the-shelf                 | ~$5                |
| Load cell (100g–500g) + HX711 amplifier                | Off-the-shelf                 | ~$5                |

## Design Considerations

- **Mirror quality matters.** First-surface (front-coated) mirrors avoid the double
  reflections that standard back-coated mirrors produce. Small first-surface mirrors
  are available cheaply from optics suppliers.
- **Mirror mounts need fine adjustment.** Design with slotted holes and set screws so
  angles can be tuned without reprinting. Once dialed in, it stays put.
- **The light box is critical.** Must be opaque to ambient light. Diffused LED lighting
  eliminates shadows and reflections. Clamshell design for easy access to clear jams
  and clean mirrors.
- **Transparent section trade-offs.** Scratches, dust, and belt/slide texture on the
  acrylic may degrade the bottom view. May need regular cleaning or a replaceable
  window. Test whether the bottom view adds enough discrimination to justify the
  complexity.

## Supplementary Sensors

### Brick Detection: Time-of-Flight (replaces beam-break)

A VL53L1X time-of-flight sensor mounted above the gravity slide serves dual
purpose: brick presence detection and bounding box dimensional measurement.

- **Presence detection.** Distance drops from empty-slide baseline when a brick
  enters the scan zone. One-time calibration sets the threshold. Polling at 50Hz
  (20ms) is more than sufficient — at slide speeds of 50–100mm/s, the brick moves
  ~1mm between polls.
- **Dimensional measurement.** The distance profile as the brick transits gives
  height. Combined with known slide speed or a second ToF sensor along the slide
  axis, length is extracted. Width from a lateral sensor axis or inferred from
  the top-down camera view.

This replaces the IR beam-break sensors in the original design. The beam-break's
sub-millisecond response offers no practical advantage at these slide speeds, and
the ToF provides dimensional data at no additional cost.

### Weight Measurement

A load cell (100g–500g range) with HX711 amplifier integrated into the scan
position platform. The brick is momentarily stationary during image capture —
this pause is the natural weigh point.

- Tare between bricks.
- Read time <100ms, well within the scan cycle budget.
- Mechanical integration: the weigh platform must not interfere with the
  transparent section (if present) or mirror sight lines. Design as a short
  settling shelf at the scan position.

Together, ToF dimensions and weight produce a 4-scalar vector (length, width,
height, mass) that feeds the classification engine as a supplementary signal
alongside the visual embedding.

## Design Reference

The BrickSortingMachine project's scanner module demonstrates the camera + mirror
approach with a single mirror. Their design blog explains the synchronization and
computational advantages.

## Open Questions

- Optimal mirror angles and distances for resolution-per-view trade-off?
- Rolling capture (continuous video + frame extraction) vs. triggered still capture?
- Calibration procedure for geometric relationship between mirror views?
- Does the anti-stud (bottom) view add enough classification value to justify the
  transparent section complexity?
- Should we include a load cell for weight measurement as a supplementary signal?
- Load cell platform placement: above or below the transparent section? Does the
  weigh step happen before, during, or after image capture?
- Single ToF sensor (height profile only, infer length from transit time) vs. two
  sensors (direct height + length)?
