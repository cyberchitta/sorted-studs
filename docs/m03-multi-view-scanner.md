# Module 3: Multi-View Scanning Station

**Status:** Original design (sorted-studs innovation)
**Dependencies:** [Module 2 — Vibratory Singulation](module-02-vibratory-singulation.md) (input)
**Feeds into:** [Module 4 — Classification Engine](module-04-classification-engine.md) (image data), [Module 5 — Diverter](module-05-diverter.md) (physical brick)

---

## Purpose

Capture high-quality images of each brick from four angularly-separated
viewpoints simultaneously, in a single camera frame, using one physical camera
plus three first-surface mirrors arranged so the four viewpoints form a regular
tetrahedron around the brick. This is a core innovation of sorted-studs — most
existing sorters use a single top-down view, which cannot distinguish parts with
identical profiles (2×4 plate vs. 2×4 tile from above; standard vs. Technic
brick from above; any slope/curve detail).

## Design Rationale

### Why multiple mirrors over multiple cameras

- **Perfect synchronisation.** All views in one exposure — no timing issues.
- **One pipeline.** Single image to preprocess, one driver, one exposure.
- **Lower cost.** One sensor and lens rather than four.
- **Consistent colour.** Same sensor + same lighting across views → color
  classification consistent view-to-view.

### Why tetrahedral rather than ad-hoc mirror placement

The viewpoint optimisation in [klods-syn notebook
`nbs/200_viewpoints.md`](../../klods-syn/nbs/200_viewpoints.md) shows that for
four viewpoints around a target, the configuration that maximises the minimum
pairwise angular separation is the regular tetrahedron — all six pairwise
angles equal **109.47°** (= cos⁻¹(−1/3)). This is provably the most informative
4-view configuration: no two views are redundant, every pair is maximally
separated.

With only three views (as in most multi-mirror scanners), the optimum is three
views 120° apart in a single plane — but any small rotation of the brick about
that plane's normal leaves the views nearly identical. Tetrahedral adds a
fourth vertex out of the plane, eliminating the degeneracy.

## Geometry

### Coordinate frame

Origin at the brick centroid when it is stationary at the scan position.

- **+X** — to the operator's right
- **+Y** — up
- **+Z** — away from the operator (toward the back of the machine)

Units: millimetres throughout.

### Rig orientation — tilted forward

The tetrahedral viewpoint set is rotationally invariant; only the orientation of
the tet axis in world space is a design choice. We tilt the axis so that:

- The **camera** sits in front of and above the brick — natural phone/camera
  mount, a clear path from operator to screen, and no overhead mast.
- **One mirror** is behind and above the brick — lives on the back-upper wall
  of the enclosure.
- **Two mirrors** are below and slightly behind the brick, one on each side —
  view the brick's underside through a transparent window in the platform.

The camera elevation above horizontal is the chief tuning knob; at 25° the three
mirrors land at the physical positions tabulated below. Phase rotation around
the tet axis is independently chosen so the side mirrors are symmetric about
the operator's fore-aft axis (single-plane symmetry, x-mirror images).

### Viewpoint positions

At camera elevation θ = 25°, target-to-camera distance 250 mm:

| Viewpoint        | Position (mm)         | Role                   |
| ---------------- | --------------------- | ---------------------- |
| Camera (direct)  | (0, +105.7, −226.6)   | Front-high, looks back-and-down at brick |
| Virtual (top)    | (0, +465.6, +456.9)   | Reflection via back-upper mirror — sees brick from behind-and-above |
| Virtual (left)   | (−557.2, −197.4, +406.7) | Reflection via below-left mirror — sees brick underside from lower-left |
| Virtual (right)  | (+557.2, −197.4, +406.7) | Reflection via below-right mirror — sees brick underside from lower-right |

Pairwise angular spread: 109.47° across all six pairs (regular tetrahedron).

### Physical mirror positions

| Mirror     | Centre (mm)                | Normal (unit)            | Role |
| ---------- | -------------------------- | ------------------------ | ---- |
| M_back_upper  | (0, +176.0, +172.7)     | (0, −0.466, −0.885)      | Reflects camera → back-upper virtual viewpoint |
| M_below_left  | (−201.4, −140.1, +25.4) | (+0.690, +0.616, −0.380) | Reflects camera → below-left virtual viewpoint |
| M_below_right | (+201.4, −140.1, +25.4) | (−0.690, +0.616, −0.380) | Reflects camera → below-right virtual viewpoint |

Mirror face orientation = normal direction (reflective side faces the camera and
brick). First-surface mirrors, 80 × 80 mm nominal (revise after first
prototype).

### Transparent aperture

Only the two below-mirrors look through the platform. A circular transparent
region of **Ø180 mm** centred on the origin is sufficient for bricks up to ~60
mm in the longest dimension and 80 mm mirrors. Outside this disc the platform
may be opaque (for load cell mounting, lighting fixtures, etc.).

*Sizing derivation:* for each (brick-point, mirror-point) sightline, compute
the y = 0 crossing. The union across the brick's bounding volume (LxWxH ≲ 60 ×
30 × 60 mm centred at origin) and each mirror's face produces an elongated
quadrilateral extending from the brick base toward each below-mirror. Ø180
mm covers both below-mirrors' footprints with ~20 mm margin.

### Bounding envelope

Rounded up to practical numbers for enclosure design:

| Axis | Extent (mm) | Notes |
| ---- | ----------- | ----- |
| X    | ±240        | Outer edge of below-side mirrors + mount clearance |
| Y    | −180 to +260 | Below-mirror mount to back-upper mirror + camera mount |
| Z    | −280 to +230 | Camera mount to drop chute entry at back |

Overall: ~480 × 440 × 510 mm enclosure envelope (W × H × D). Actual enclosure
will be larger to include lighting, walls, and access doors.

### Parameter table (for CAD variants)

All positions above derive from these parameters. Re-running the optimiser in
`klods-syn/nbs/200_viewpoints.md` with a different tilt regenerates the table.

| Parameter              | Value     | Notes                                                 |
| ---------------------- | --------- | ----------------------------------------------------- |
| `camera_dist`          | 250 mm    | Target-to-camera distance                             |
| `ring_radius`          | 232.5 mm  | Pre-tilt mirror ring radius (from optimiser)          |
| `ring_height`          | 82.2 mm   | Pre-tilt mirror ring drop below target                |
| `tilt_angle`           | 65°       | Rotation about +X to bring camera from up to front-up |
| `camera_elevation`     | 25°       | Equivalent elevation of camera above horizontal       |
| `phase`                | 90°       | Mirror phase around tet axis (one mirror at x = 0)    |
| `mirror_size`          | 80 × 80 mm | First pass; revise after prototype                   |
| `brick_envelope`       | 60 × 30 × 60 mm | Max brick bounding box for FOV/aperture sizing |

## Physical Layout

### Side view (looking along +X, operator at left/−Z)

```
        Z−(front)                 Z+(back)
                                    drop chute from M2 singulator
                                    ▼
    camera ●                 [ M_back_upper ]
    (+Y=105, Z=−227)          ◣ (+Y=176, Z=+173)
           \               /
            \           /
             \       /
              \   /
               ◎  brick at origin on platform
    ═══════════════════════════════   y=0 platform
    . . . . . [  Ø180 transparent  ]  . . . . .
               \   /
                \_/
               [ M_below_left/right ]
              (Y=−140, Z=+25)
```

### Top view (looking down −Y)

```
                   ┌─────────┐
                   │ chute   │  Z+ (back)
                   └────┬────┘
                        │
           M_below_L  ● │ ●  M_below_R
                   (±201, +25)
                        │
                      ◎ brick @ origin
                        │
                    ● camera (Z=−227)
                         ↕ X axis
                        ↓
                    Z− (operator / front)
```

Back-upper mirror sits above the scan center at (0, +176, +173). Drop chute
comes in from higher and further back (e.g., chute exit around y ≥ +250, z ≥
+200) so the brick's descent path clears the mirror face.

## Camera

### Reference: Raspberry Pi HQ Camera

- **Sensor:** Sony IMX477, 12.3 MP (4056 × 3040), 7.4 × 5.6 mm
- **Lens mount:** C / CS — pick focal length to hit the FOV target below
- **Why Pi HQ over iPhone for the capture role:**
  - Rigid bolt-down mount; no risk of operator bumping a phone and moving the
    optical axis off calibration
  - Lens focal length is a choice, not a phone spec — critical because we need
    all four views in one frame
  - Cheap enough to dedicate to the rig ($50–$100 fully kitted)
  - Image quality sufficient for ≥1500×1500 pixels per view after crop

### Field of view requirement

Camera at (0, 105.7, −226.6) looking toward origin. All three physical mirrors
sit at distance ~405 mm from the camera, at 35° from the central look axis (by
tet symmetry, all three are equidistant and equiangular from the camera). Each
mirror subtends ~11° at 80 mm size.

**Required half-FOV ≥ 35° + 5.6° + margin → full FOV ≥ ~82°.**

A 2.8 mm C-mount lens on a Pi HQ sensor gives ~100° HFOV (comfortable margin);
4 mm gives ~80° (tight). Start with 2.8 mm for the prototype.

### Inference host

The Pi camera acts as a **pure capture device**. Inference runs on:

- **Mac (bootstrap phase)** — Pi streams frames over USB or Ethernet; Mac runs
  CoreML or PyTorch model. Matches the development plan in `CLAUDE.md` where
  the Mac is the inference host before the mobile path is complete.
- **iPhone (optional later)** — can be re-introduced as a headless inference
  node (rigidly mounted, never touched, operated via WiFi-served web UI from a
  laptop). Preserves Neural Engine deployment without making the phone an
  interactive surface.
- **Edge accelerator (future)** — Coral USB / Jetson Nano / similar if
  standalone operation without a Mac is wanted.

The architectural point: **the capture device and the inference device are
separate.** Calibration stability lives with the capture device (Pi HQ, bolted
down); ML deployment flexibility lives with whatever box runs the model.

### Mount

- 3D-printed pad-on-rail clamp hanging from the camera rig's rail D
  cantilever (see Frame Architecture above)
- Once locked, **not user-adjustable during operation** — calibration depends
  on fixed geometry
- Sacrificial shim stack for initial alignment; lock with threadlocker after
  first-use tuning

## Lighting

### Primary: ring light around the platform

An LED ring at platform level (y ≈ 0), outer diameter ≳ Ø500 mm, inner ring
clearance ≳ Ø300 mm so it sits outside the below-mirror sightline footprints.
Ring faces inward/upward toward the brick.

- **Colour:** 5000 K daylight, CRI ≥ 95 (colour fidelity matters — we classify
  by color)
- **Diffused** front face to soften highlights on glossy ABS
- **Matte black back** (outward-facing side) to minimise the ring's own
  visibility in the back-upper mirror's reflection

### Secondary: underside illumination (open)

The two below-mirrors see the brick's anti-stud face. A top/side ring light
alone will leave that face dim. Options to evaluate at prototype stage:

1. **Edge-lit transparent disc** — LEDs embedded in the rim of the Ø180 mm
   acrylic window, light propagates through the disc and exits uniformly upward
   at the brick underside. Can silhouette anti-stud detail; test carefully.
2. **Sub-platform ring** — small LED ring just outside the Ø180 mm disc,
   beneath the platform, pointing up-and-inward through the transparent window.
3. **Bounce only** — no dedicated underside light; rely on scattered light from
   top ring and matte-white enclosure interior.

Decision deferred until the first geometry + top-ring prototype is running and
we can see what the bottom views actually look like.

### Light-tightness

Enclosure is closed and painted matte black internally (except where diffuse
white bounce is wanted). Ambient light must be excluded — daylight changes would
drift colour classification.

## Platform

### Transparent disc

- **Material:** optical-grade cast acrylic (PMMA) or float glass
- **Diameter:** Ø180 mm (transparent region)
- **Thickness:** 3–5 mm — rigid enough to support one brick + load cell
  transfer
- **Surface finish:** both faces optically polished; anti-reflection coating
  optional for glare reduction
- **Replaceable:** mounted in a frame such that it can be swapped without
  disturbing the rest of the rig (scratches will accumulate)

### Opaque outer platform

Outside Ø180 mm, the platform can be opaque — hosts the feed landing zone,
ring light mount, load cell, and ToF sensor. Total platform diameter ~Ø360 mm
including this collar.

### Load cells inline with the hinge axis

The platform is a rectangular collar hinged along its front edge. Two
pillow blocks on the platform frame's top cross-rail carry the hinge
pin; each pillow block sits on a 500 g beam load cell sandwiched
between the block and the cross-rail. While the platform is held
level by a solenoid-driven pin at the back edge, the sum of the two
load-cell readings (minus the tared platform mass) reads the brick
mass. Retracting the pin tips the platform forward, dumping the brick
into the Module 5 diverter ingest.

Rationale: load cells inline with the hinge axis see the full
platform + brick load as pure compression on the pillow-block ↔
cross-rail interface. Any horizontal force from brick motion on the
platform is reacted by the hinge pin rather than appearing as a
moment on the load cells.

## Feed & Eject

### Feed from back (matches singulator exit)

The singulator (Module 2, C-channel variant) delivers one brick at a time to a
drop chute that enters at the back-upper corner of the enclosure.

- Chute exit at approximately **(0, +250, +220) mm** — above and behind the
  back-upper mirror, positioned so the brick's drop path does not graze the
  mirror face (the mirror at +176, +173 is in front of and below the chute
  exit; brick drops past it, not through it)
- Chute angled to deposit the brick onto the back edge of the opaque platform
  collar (outside the Ø180 transparent disc)

### Shuttle to scan position

A short linear actuator / push-plate moves the brick from the back landing zone
forward onto the transparent disc, then stops at origin. Actions:

1. Drop from chute onto back landing (z ≈ +120 mm)
2. Shuttle pushes forward to origin (z = 0), brick settles
3. Load cell reads mass (platform held level by back-edge pin)
4. Camera captures frame
5. Back-edge pin retracts; platform tips forward on its front-edge
   hinge; brick slides off the front edge into the Module 5 diverter
   ingest. Pin re-engages and the platform returns to level for the
   next brick.

Shuttle surface and push-plate are matte black to avoid showing up in mirror
views if visible in the frame.

### Ejecting forward

The platform tips forward on its front-edge hinge when the back-edge
solenoid pin retracts, dumping the brick into the diverter staging
zone. Eject direction (−Z, toward operator front) is opposite the
feed direction (+Z, from back), and the camera rig's open front face
leaves the eject path unobstructed.

## Supplementary Sensors

### Time-of-Flight for brick presence + dimensions

VL53L1X or equivalent, mounted on the underside of the camera rig's
cantilever rail D above the scan position, pointed straight down.

- **Presence detection.** Distance drops below empty-platform baseline when the
  shuttle brings a brick in.
- **Height measurement.** Direct — ceiling-to-top-of-brick subtracted from
  ceiling-to-empty-platform baseline.
- **Polling:** 50 Hz is ample at shuttle speeds of ≲ 100 mm/s.

Mounted on the camera rig's cantilever rail D (y ≈ +250 mm), clear of all
mirror cones and the camera's direct view of the brick (sensor sits above
camera axis).

### Load cell + HX711

- **Range:** 100–500 g (covers typical brick masses 0.3–20 g with headroom)
- **Readout:** HX711 amplifier, 10 SPS is fine — brick is stationary for ≳ 500
  ms during capture
- **Placement:** supports the full platform disc from below; tares between
  bricks (measured during the shuttle-forward motion when no brick is yet on
  the disc)

## Frame Architecture

Two independent frames share the floor plane. Separating them insulates
optical calibration from platform service operations (disc cleaning,
load-cell access, shuttle work) and provides an unobstructed forward
eject path to Module 5.

### Camera rig — sparse T-frame

Four 2020 T-slot aluminum extrusions plus a diagonal truss carry all
three mirror mounts and the camera mount. There is no structure in
the brick plane and nothing crosses the eject path; the "front face"
is fully open by construction.

- **rail A** — X-span at the below-mirror plane (y ≈ −140, z ≈ +28).
  Both below-mirror clamps grip rail A directly; beam reach is zero.
- **rail B** — Z-span from rail A back to under the back-upper mirror
  plane (y ≈ −140, z ≈ +180). Forms the T with rail A. Extends past
  both junctions in the Z direction as fore-aft outrigger feet.
- **rail C** — vertical post from rail B's back end up past the
  back-upper mirror to the cantilever height. Back-upper clamp grips
  rail C directly; beam reach is zero.
- **rail D** — Z-cantilever from the top of rail C forward over the
  camera. Camera clamp hangs from rail D's underside.
- **diagonal truss** — tension strut from the top of rail C
  forward-and-down to rail B. Converts rail D from a pure cantilever
  to a triangulated truss, collapsing ~1–2 mm of static tip deflection
  at the camera to near-zero.

Mirror and camera clamps are printed pad-on-rail parts — a clamp body
gripping the rail with a flat pad carrying the mount's four-corner
bolt pattern. Rails A, C, and D all pass through or immediately
behind the mount planes, so beam reach is eliminated everywhere.

Joint reinforcement: the T-junction (rail A ↔ B) and the L-junction
(rail C ↔ D) carry bending loads that exceed what stock corner
brackets resist. Each joint is closed with a printed gusset — heat-
set M4 inserts in the gusset mate with T-nuts in the extrusion
slots, clamping two faces of each member.

### Platform frame — freestanding

A small 2020-extrusion rectangle on its own feet, sized to slide into
the camera rig's open interior. Carries the hinged-quad platform and
its pillow blocks (see Platform section). The two frames share a
floor plane but need no precision indexing — only platform-to-brick
geometry is optically relevant, and the brick sits on the platform.

Module 5 (diverter) bolts to the platform frame rather than the
camera rig, giving the Module 2 → 3 → 5 pipeline a single shared
mechanical interface.

### Shroud

Ambient-light rejection is provided by an opaque fabric or light
printed shroud draped over the camera rig. The shroud carries no
load; the rig structure stands independently.

### Adjustment

Mirror mounts adjustable in pitch + yaw via the rev-1 three-screw
kinematic stack (`cad/m03_scanner/mirror_mount_kinematic/`). Coarse
position from printed clamp geometry, fine tune during first
calibration, lock with jam nuts.

## Key Components (revised)

| Component                                          | Source                    | Est. Cost |
| -------------------------------------------------- | ------------------------- | --------- |
| Raspberry Pi HQ Camera + 2.8 mm C-mount lens       | Pi Hut / Adafruit          | ~$75      |
| Raspberry Pi 4 (capture host, streams to Mac)       | standard                  | ~$55      |
| First-surface mirrors, 80 × 80 mm (×3)              | Edmund Optics / Amazon    | ~$20      |
| Ø180 mm optical acrylic disc, 4 mm thick            | McMaster / local plastics | ~$10      |
| LED ring, Ø500 mm, 5000 K, CRI ≥ 95, diffused      | Neewer / similar          | ~$40      |
| 3D-printed mirror mounts (×3)                      | original                  | ~$3 filament |
| 3D-printed camera bracket                          | original                  | ~$3 filament |
| 2020 T-slot aluminum extrusion (~4.5 m, camera + platform frames combined) | stock | ~$25 |
| T-nuts, corner brackets, M4/M5 hardware            | stock                     | ~$15       |
| 3D-printed joint gussets + pad-on-rail clamps      | original                  | ~$5 filament |
| 3D-printed pillow blocks + hinged-quad platform    | original                  | ~$10 filament |
| Matte black interior paint                          | hardware store            | ~$10      |
| VL53L1X ToF sensor                                  | Adafruit                  | ~$5       |
| 500 g load cell + HX711                             | Amazon                    | ~$8       |
| Linear actuator / servo-driven shuttle              | TBD (Module 2 interface)  | —         |
| iPhone (optional, for on-device inference)          | used                      | ~$200     |

## Design Considerations

- **Mirror quality.** First-surface mirrors only — back-coated mirrors produce
  secondary reflections that confuse the image.
- **Calibration.** After first assembly, capture a checkerboard or known
  calibration brick from every view and solve the extrinsics per mirror.
  Calibration is stable as long as nothing in the rig moves — hence the rigid
  camera mount and lock-down mirror adjusters.
- **Transparent disc maintenance.** Scratches and dust accumulate. Make the
  disc user-replaceable without disturbing anything else. Schedule periodic
  cleaning during operation (a blast of filtered air between bricks, perhaps).
- **Shuttle visibility.** If the shuttle push-plate is in any mirror's field of
  view during capture, matte-black it or design it to retract below the
  platform surface during the capture window.
- **Underside lighting is the biggest open risk.** The two below-mirror views
  are only as good as how well we illuminate the brick's anti-stud face.
  Budget prototype time specifically for this.

## Open Questions (for CAD and prototype stage)

- Optimal `camera_elevation` — is 25° the best trade-off, or should we test a
  few values between 20° and 35°?
- Mirror size — 80 × 80 mm is a guess. How small can we go before view resolution
  degrades?
- Shuttle mechanism — linear actuator, servo-driven slide, or something simpler
  like a tilting ramp?
- Underside illumination strategy — edge-lit disc vs. sub-platform ring vs.
  bounce only?
- How does Module 2's singulator exit land on the platform frame's back
  landing zone? (The platform frame, not the camera rig, is now the shared
  Module 2 → 3 → 5 mechanical interface — see Frame Architecture.)
- Calibration procedure — what's the minimal set of known targets we need to
  solve all four view extrinsics?

## Design Reference

Base mirror + camera approach from [BrickSortingMachine](https://bricksortingmachine.com).
Tetrahedral optimisation and parameter choice derived in
[klods-syn `nbs/200_viewpoints.md`](../../klods-syn/nbs/200_viewpoints.md).
