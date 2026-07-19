# Module 3: Multi-View Scanning Station

**Status:** Original design (sorted-studs innovation)
**Dependencies:** [Module 2 — C-Channel Singulator](m02-c-channel-singulator.md) (input)
**Feeds into:** [Module 4 — Classification Engine](m04-classification-engine.md) (image data), [Module 5 — Hex Tower Diverter](m05-diverter.md) (physical brick)

---

## Purpose

Capture high-quality images of each brick from four angularly-separated
viewpoints simultaneously, using **four cameras placed at the vertices of a
regular tetrahedron** centred on the brick. This is a core innovation of
sorted-studs — most existing sorters use a single top-down view, which cannot
distinguish parts with identical profiles (2×4 plate vs. 2×4 tile from above;
standard vs. Technic brick from above; any slope/curve detail).

The earlier sorted-studs design used a single physical camera plus three
first-surface mirrors at the same four tetrahedral positions. That topology
was abandoned in the 2026-05-04 rev-5 pivot for cost, optical-tolerancing,
and host-agnosticism reasons; the rationale is captured in
`_notes/cad/m03_scanner/cameras_design_notes.md`. **The four world-space viewpoint
positions did not change** across the pivot (they were already a verified
regular tetrahedron); only the device at each vertex changed from
"mirror reflecting one shared camera" to "independent capture node".

## Design Rationale

### Why four direct cameras over one camera + three mirrors

- **Cost.** 4× AI-Thinker ESP32-CAM (M12 lens variant) ≈ $48 total vs.
  Pi HQ camera + 3 first-surface mirrors + precise mirror mounts ≈ $120–160.
- **Independence.** Each camera is a self-contained network node; one
  failure doesn't take out the rest of the rig.
- **Host-agnostic.** Cameras serve frames over HTTP; the inference host
  can move (Mac today, edge accelerator later) without touching the rig.
- **No optical-alignment drag.** First-surface mirrors are sensitive to
  fingerprints, dust, and thermal drift; per-mirror tilt calibration would
  enter every image. Direct cameras eliminate the entire bounce path.
- **Synchronisation is cheap enough.** With an indexed turret (brick
  stationary at IMAGE during dwell), four independent HTTP triggers within
  the dwell window suffice — no microsecond sync needed.

### Why tetrahedral rather than ad-hoc placement

For four viewpoints around a target, the configuration that maximises the
minimum pairwise angular separation is the regular tetrahedron — all six
pairwise angles equal **109.47°** (= cos⁻¹(−1/3)). This is provably the
most informative 4-view configuration: no two views are redundant, every
pair is maximally separated. With only three views the optimum is three
views 120° apart in a single plane, but a small rotation of the brick about
that plane's normal leaves the views nearly identical. Tetrahedral adds a
fourth vertex out of the plane, eliminating the degeneracy.

The tetrahedral viewpoint set was originally derived for the mirror design
in `klods-syn/nbs/200_viewpoints.md` at tilt 65° (camera elevation 25°),
phase 90°, target distance 250 mm. Those parameters carry forward unchanged
to the camera design.

## Geometry

### Coordinate frame

Origin at the brick centroid when it is stationary at the scan position, on
the platform top face.

- **+X** — operator's right
- **+Y** — away from operator (toward the back of the machine)
- **+Z** — up (gravity is −Z)

Units: millimetres throughout. This matches the project-wide world frame
documented in `cad/CLAUDE.md`. (Note: the upstream viewpoint optimiser is
Y-up; values are converted to Z-up at the boundary in
`cad/m03_scanner/camera_rig/params.py`.)

### Rig orientation — tilted forward

The tetrahedral viewpoint set is rotationally invariant; only the orientation
of the tet axis in world space is a design choice. The axis is tilted so
that:

- One vertex sits in front of and above the brick — the **front-low**
  camera, hanging from the underside of the camera rig's rail D
  cantilever, looking back-and-down at the brick.
- One vertex is behind and above the brick — the **back-upper** camera, on
  the back-upper face of rail C.
- Two vertices are below and slightly behind the brick, one on each side —
  the **below-left** and **below-right** cameras, riding rail A's top
  face. They look up through a transparent window in the platform.

### Vertex positions

At target-to-camera distance 250 mm (verified at ~248 mm including the
PCB-to-vertex offset):

| Vertex             | Position (mm)            | Role                                                     |
| ------------------ | ------------------------ | -------------------------------------------------------- |
| `front_low`        | (0, −226.6, +105.7)      | Front-and-above; looks back-down at brick                |
| `back_upper`       | (0, +172.7, +176.0)      | Behind-and-above; looks forward-down at brick            |
| `below_left`       | (−201.4, +25.4, −140.1)  | Below platform, left of brick; looks up through aperture |
| `below_right`      | (+201.4, +25.4, −140.1)  | Below platform, right of brick; looks up through aperture |

Pairwise angular spread: 109.47° across all six pairs (regular tetrahedron;
verified by `camera_rig/params.py`'s docstring assertion: every pairwise
unit-vector dot product is −1/3 to within ±2 mm of the 248 mm radius).

Each lens optical axis points directly at the world origin:
`lens_axis_unit(vertex) = -vertex / |vertex|`.

The authoritative source for these coordinates is
`cad/m03_scanner/camera_rig/params.py`. Update that file and this doc
together whenever a value changes.

### Transparent aperture

The two below-vertex cameras look up through the platform. A transparent
region of **≥ Ø172 mm** centred on the origin is sufficient for bricks up
to ~60 mm in the longest dimension; the Ø172 figure is the union of both
below-camera sightline cones at the platform plane (z = 0). The current
platform is a 180 × 180 × 4 mm cast acrylic plate — that footprint is set
by the Ø172 clear region plus a few millimetres of edge-clamp grip.

### Bounding envelope

Rounded to practical numbers for enclosure design:

| Axis | Extent (mm)  | Notes                                              |
| ---- | ------------ | -------------------------------------------------- |
| X    | ±240         | Outer reach of below-side camera mounts + clearance |
| Y    | −280 to +260 | Front-low camera mount → back-upper camera mount    |
| Z    | −180 to +230 | Below cameras → drop chute entry                    |

Overall: ~480 × 540 × 410 mm enclosure envelope (W × D × H). Actual
enclosure will be larger to include lighting, walls, and access doors. A
size-optimisation sweep that tightens the rig against the smaller
ESP32-CAM mount footprint is planned (the tetrahedron was originally sized
for 110 × 110 mm mirror plates).

### Parameter table (for CAD variants)

All positions above derive from these parameters. Re-running the optimiser
in `klods-syn/nbs/200_viewpoints.md` with a different tilt regenerates the
table.

| Parameter              | Value             | Notes                                                       |
| ---------------------- | ----------------- | ----------------------------------------------------------- |
| `camera_dist`          | 250 mm            | Origin-to-camera distance (tetrahedron radius)              |
| `tilt_angle`           | 65°               | Tilt of the tet axis in world space                         |
| `camera_elevation`     | 25°               | Equivalent elevation of the front-low camera above horizontal |
| `phase`                | 90°               | Phase around tet axis (one vertex on the YZ plane)          |
| `brick_envelope`       | 60 × 30 × 60 mm   | Max brick bounding box for FOV / aperture sizing            |

## Cameras

### Reference: AI-Thinker ESP32-CAM (M12 lens variant)

- **PCB:** 40.5 × 27 × 4.5 mm rectangular; 4× M2-class corner mounting holes
  on a ~22 × 35 mm pitch (nominal — pending caliper verification on a
  physical board).
- **Sensor:** OmniVision OV2640, 2 MP (1600 × 1200).
- **Lens mount:** M12×0.5 (board lens). The M12 swap variant is required;
  the stock fixed-lens board has ~70° HFOV which would waste ~75% of sensor
  pixels on background at 250 mm working distance.
- **Connectivity:** WiFi (PCB-trace antenna or external IPEX); each camera
  is reachable as `cameraN.local` via mDNS.
- **Capture endpoint:** stock `CameraWebServer` / ESPHome / `esp32-cam-rtsp`
  firmware exposes an HTTP `/capture` endpoint returning a JPEG.
- **Cost:** ~$12 each (~$48 for four).

The full hardware-comparison rationale (vs. UVC, Pi GS, Pi HQ, ESP32-S3,
Freenove) is in `_notes/cad/m03_scanner/cameras_design_notes.md`.

### Field of view

At 250 mm camera-to-brick distance, an 80 mm subject (a margin around the
180 × 180 mm platform projected through perspective foreshortening) needs
HFOV ≈ 41° to fill a 4:3 frame with ~5 mm margin — equivalent to a ~5–6 mm
M12 lens. The M12 lens is selectable per the framing requirement; the
chosen value enters the optical calibration.

### Inference host

Cameras act as **pure capture devices**. Inference runs on:

- **Mac (current).** Python service fans 4 parallel HTTP GETs to
  `http://cameraN.local/capture` and runs the classifier (CoreML or
  PyTorch). All four frames arrive within the indexer's IMAGE-station
  dwell.
- **Edge accelerator (future).** Coral USB / Jetson Nano / similar if
  standalone operation without a Mac is wanted.
- **iPhone (deferred).** The earlier "iPhone runs capture + inference"
  path from the project root `CLAUDE.md` is deferred indefinitely; the
  Mac path is the live target.

The architectural point: **the capture device and the inference device are
separate.** Each camera's calibration lives with the rigid mount;
inference flexibility lives with whatever box runs the model.

### Mount

- 3D-printed `camera_mount_esp32cam/` body — single FDM part replicated
  four times, parameterised by `mount_in_clamp` for each vertex pose.
- Holds the PCB by four M2 corner bolts (oversized Ø2.4 holes absorb
  pitch tolerance until the nominal 22 × 35 mm pattern is verified with
  calipers).
- Antenna clearance pocket on the short edge containing the PCB-trace
  antenna; M12 lens column clearance forward of the PCB.
- Same rail-clamp interface (`mount_in_clamp` API) the previous
  mirror/camera mounts used; clamps to the existing rev-3 sparse T-frame
  unchanged.
- One physical-board validation print is required before printing four for
  installation — see `_notes/cad/m03_scanner/cameras_design_notes.md`.

## Lighting

### Primary: ring light around the platform

An LED ring at the platform plane (z = 0), outer diameter ≳ Ø500 mm, inner
clearance ≳ Ø300 mm so it sits outside the below-camera sightline
footprints. Ring faces inward/upward toward the brick.

- **Colour:** 5000 K daylight, CRI ≥ 95 (colour fidelity matters — we
  classify by color)
- **Diffused** front face to soften highlights on glossy ABS
- **Matte black back** to minimise the ring's own visibility in the
  back-upper camera's frame

### Secondary: underside illumination (open)

The two below-cameras see the brick's anti-stud face. A top/side ring
light alone will leave that face dim. Options to evaluate at prototype
stage:

1. **Edge-lit transparent disc** — LEDs embedded in the rim of the
   transparent platform region, light propagating through the disc and
   exiting upward at the brick underside. Risks silhouetting anti-stud
   detail; test carefully.
2. **Sub-platform ring** — small LED ring just outside the transparent
   aperture, beneath the platform, pointing up-and-inward through the
   transparent window.
3. **Bounce only** — no dedicated underside light; rely on scattered light
   from the top ring and matte-white enclosure interior.

Decision deferred until the first geometry + top-ring prototype is running
and we can see what the bottom views actually look like.

### Light-tightness

Enclosure is closed and painted matte black internally (except where
diffuse white bounce is wanted). Ambient light must be excluded — daylight
changes would drift colour classification.

## Platform

### Transparent acrylic plate

- **Material:** optical-grade cast acrylic (PMMA) sheet stock
- **Footprint:** 180 × 180 × 4 mm — set by the Ø172 transparent zone
  (union of both below-camera cones) plus a few millimetres of edge-clamp
  grip
- **Surface finish:** both faces optically polished; anti-reflection
  coating optional for glare reduction
- **Replaceable:** mounted in a frame such that it can be swapped without
  disturbing the rest of the rig (scratches will accumulate)

### Rotating turret

The current platform-handling design is a 4-station rotary indexed turret
(LOAD, IMAGE, DUMP, RETURN at 90° spacing). At any instant one platform
sits at IMAGE with its centroid at the world origin and its top face at
z = 0. A bench-mounted kicker driven by a cam on the turret shaft tilts
the platform at DUMP to drop the brick into the Module 5 diverter inlet.

Calibration is fixed to the IMAGE world coordinates; the indexer's stop
position must hold ±0.2 mm in X/Y and ±0.1° in tilt or the four platforms
will produce slightly different images of the same brick. See
`_notes/cad/m03_scanner/turret_design_brief.md` for the full turret design.

### Load cells (deferred)

Per-platform load-cell-on-hinge mass measurement is deferred from the
optical-only slice. When it lands, it swaps the printed bushing blocks for
load-cell-pocket variants on the same cross-rail bolt pattern with no
change to the platform frame.

## Feed & Eject

### Feed from back (matches singulator exit)

The singulator (Module 2, C-channel variant) delivers one brick at a time
to a drop chute that enters at the back-upper corner of the enclosure.

- Chute exit at approximately **(0, +250, +220) mm** — above and behind
  the back-upper camera mount, positioned so the brick's drop path does
  not graze any structural element on its way to the LOAD platform.
- Chute angled to deposit the brick onto the LOAD platform (at the +X
  station).

### Cycle at IMAGE

1. Turret indexes a loaded platform from LOAD to IMAGE (90° rotation).
2. Brick settles at origin during dwell.
3. All four cameras are triggered; each returns a JPEG over HTTP.
4. Turret advances; the platform that just imaged moves to DUMP, the
   next-loaded platform moves to IMAGE.

### Ejecting forward at DUMP

At DUMP the bench-mounted kicker tilts the platform forward; the brick
slides off into the Module 5 diverter ingest. Eject direction (−Y, toward
operator front) is opposite the feed direction; the camera rig's open
front face leaves the eject path unobstructed.

## Supplementary Sensors

### Time-of-Flight for brick presence + dimensions

VL53L1X or equivalent, mounted on the underside of the camera rig's
cantilever rail D above the IMAGE station, pointed straight down.

- **Presence detection.** Distance drops below empty-platform baseline
  when a platform with a brick arrives at IMAGE.
- **Height measurement.** Direct — ceiling-to-top-of-brick subtracted from
  ceiling-to-empty-platform baseline.
- **Polling:** 50 Hz is ample.

Mounted clear of all four camera FOVs (sensor sits above the
front-low camera's optical axis on rail D).

## Frame Architecture

Two independent frames share the floor plane. Separating them insulates
optical calibration from platform service operations and provides an
unobstructed forward eject path to Module 5.

### Camera rig — sparse T-frame

Four 2020 T-slot aluminum extrusions plus diagonal bracing carry all four
camera mounts. There is no structure in the brick plane and nothing
crosses the eject path; the "front face" is fully open by construction.

- **rail A** — X-span at z ≈ −210, y ≈ +28; both below-vertex camera
  clamps grip rail A's top face directly.
- **rail B** — Y-span from rail A back to under the back-upper vertex.
  Forms the T with rail A.
- **rail C** — vertical post from rail B's back end up past the back-upper
  vertex to the cantilever height. Back-upper camera clamp grips rail C's
  −Y face.
- **rail D** — Y-cantilever from the top of rail C forward over the
  front-low vertex. Front-low camera clamp hangs from rail D's underside.
- **brace_bc** — single ~45° diagonal inside the rail B ↔ C corner.
- **strut_d** — single mitered diagonal from rail C's column top
  forward-and-down to rail D's free end. Converts rail D from a bare
  cantilever to a propped beam (~10× tip-deflection reduction).
- **gussets** — printed top-side T-gusset over the rail A ↔ B junction
  and a +X-side L-gusset over the rail C ↔ D junction; both stiffen the
  bending-loaded joints beyond what stock corner brackets resist.

Rail positions are unchanged from the rev-4 mirror design — the vertex
coordinates didn't move in the pivot. The size-optimisation sweep against
the smaller ESP32-CAM mount footprint is the planned next slice.

### Platform frame — freestanding

A small 2020-extrusion frame on its own feet, sized to slide into the
camera rig's open interior. Carries the turret hub + drive + 4 platform
arms. The two frames share a floor plane but need no precision indexing
— only platform-to-brick geometry is optically relevant.

Module 5 (diverter) bolts to the platform frame rather than the camera
rig, giving the Module 2 → 3 → 5 pipeline a single shared mechanical
interface.

### Shroud

Ambient-light rejection is provided by an opaque fabric or light printed
shroud draped over the camera rig. The shroud carries no load.

## Key Components

| Component                                                                  | Source                    | Est. Cost    |
| -------------------------------------------------------------------------- | ------------------------- | ------------ |
| AI-Thinker ESP32-CAM (M12 variant) ×4                                       | AliExpress / Amazon       | ~$48         |
| M12 board lens (~5–6 mm FL) ×4                                              | M12 lenses / similar      | ~$20         |
| USB-UART flasher (one-time)                                                 | stock                     | ~$5          |
| Powered USB hub (4-port for camera power)                                   | stock                     | ~$15         |
| 180 × 180 × 4 mm optical acrylic sheet                                      | McMaster / local plastics | ~$10         |
| LED ring, Ø500 mm, 5000 K, CRI ≥ 95, diffused                              | Neewer / similar          | ~$40         |
| 3D-printed camera mounts (×4)                                               | original                  | ~$8 filament |
| 2020 T-slot aluminum extrusion (~4.5 m, camera + platform frames combined) | stock                     | ~$25         |
| T-nuts, corner brackets, M2/M4/M5 hardware                                  | stock                     | ~$15         |
| 3D-printed joint gussets                                                    | original                  | ~$5 filament |
| 3D-printed turret hub + 4 arms + cam + kicker                               | original                  | ~$25 filament |
| Matte black interior paint                                                  | hardware store            | ~$10         |
| VL53L1X ToF sensor                                                          | Adafruit                  | ~$5          |

## Design Considerations

- **Camera firmware.** Stock `CameraWebServer` on each camera gives a
  `/capture` endpoint; flash once, set WiFi credentials, expose mDNS as
  `cameraN.local`. ~30–60 minutes per camera the first time.
- **Calibration.** After first assembly, capture a checkerboard or known
  calibration brick from every camera and solve the per-camera intrinsics
  + extrinsics. Calibration is stable as long as nothing in the rig moves
  — hence the rigid mounts with no user-adjustable adjusters during
  operation.
- **Transparent acrylic maintenance.** Scratches and dust accumulate. The
  plate is user-replaceable without disturbing anything else. Schedule
  periodic cleaning during operation (filtered air between bricks,
  perhaps).
- **Underside lighting is the biggest open risk.** The two below-camera
  views are only as good as how well we illuminate the brick's anti-stud
  face. Budget prototype time specifically for this.
- **PCB hole-pattern verification.** The mount is designed against
  published nominal dimensions for the AI-Thinker ESP32-CAM; one
  validation print against a physical board is required before printing
  four for installation.

## Open Questions (for CAD and prototype stage)

- M12 lens focal length — pick once, calibrate, lock; sweep 4–7 mm range
  if framing margin proves wrong.
- Rig size optimisation — vertex coordinates were sized for 110 × 110 mm
  mirror plates; the smaller ESP32-CAM mount footprint should let
  `camera_dist` shrink, reducing the whole envelope.
- Underside illumination strategy — edge-lit disc vs. sub-platform ring
  vs. bounce only.
- WiFi reliability with four cameras on one AP under realistic workshop
  RF conditions — fallback is wired Ethernet via an ESP32-S3 variant
  with native USB.
- Calibration procedure — minimal set of known targets to solve all four
  camera extrinsics.

## Design Reference

Tetrahedral viewpoint optimisation derived in
[klods-syn `nbs/200_viewpoints.md`](../../klods-syn/nbs/200_viewpoints.md).
Mirrors → cameras pivot rationale in
`_notes/cad/m03_scanner/cameras_design_notes.md`. Hardware comparison and the
full ESP32-CAM-vs-alternatives decision trail are in the same doc.
