# Module 4: Brick Classification Engine

**Status:** Original design (sorted-studs)
**Dependencies:** [Module 3 — Multi-View Scanner](m03-multi-view-scanner.md) (image input)
**Feeds into:** [Module 5 — Diverter](m05-diverter.md) (bin routing command), [Module 6 — Inventory](m06-inventory-rebrickable.md) (classification result)

---

## Purpose

Take the multi-view image from the scanning station and produce a classification:
Rebrickable part ID, color ID, and confidence score.

## Requirements

- Identify part number from Rebrickable catalog (~3,000 common parts for v1,
  extensible to 70,000+)
- Identify color separately from part shape
- Confidence score; below threshold → route to "uncertain" bin
- Inference ≤2 seconds per brick to keep up with mechanical throughput

## Compute Platforms

The system supports two operating modes with identical HTTP interfaces. Development
mode runs all intelligence on a Mac; deployment mode runs it on-device on an iPhone.
The Arduino/ESP32 hardware layer is unchanged between modes — it talks to whichever
host is running the application layer.

### Development Mode: Mac + iPhone-as-Camera

During development, the Mac is the sole compute node for the vision pipeline.
The iPhone acts only as a camera, streaming frames to the Mac over local WiFi.

- **iPhone role:** capture frames on trigger, serve them via HTTP. No on-device
  inference.
- **Mac role:** receive frames, run classification (PyTorch), send
  `route_to_bin(N)` to ESP32, update inventory, serve UI.
- **Model format:** PyTorch (`.pt`). No CoreML export needed during dev.
- **Advantages:** full debugging tools (breakpoints, profiling, TensorBoard),
  no model size constraints, fast iteration on architecture and training,
  GPU training and inference on the same machine.

#### Camera Capture Interface

The iPhone runs a minimal capture app that exposes an HTTP endpoint:

- `POST /capture` — triggers a frame capture, returns the image as JPEG
- `GET /status` — returns camera readiness

The Mac polls or triggers this endpoint as part of the scan cycle. Latency
budget for the WiFi round-trip is ~50–100ms, well within the ≤2 second
inference window.

### Deployment Mode: iPhone (via CoreML)

For standalone deployment, the iPhone handles both image capture and inference,
making it the sole compute node for the vision pipeline. No Mac required.

- **iPhone role:** capture frames, run CoreML inference, send `route_to_bin(N)`
  to ESP32, update inventory.
- **Model format:** CoreML (`.mlpackage`), exported from PyTorch via
  `coremltools`.
- **iPhone model selection:** any iPhone with a Neural Engine (iPhone XS or
  later). iPhone 12 or later recommended for the improved ISP.
- **Neural Engine performance:** ~10–50ms for typical classification models,
  well within the ≤2 second budget.

The iPhone acts as a local HTTP server. The ESP32 POSTs sensor events
("brick in position"); the iPhone captures, infers, and responds with a
routing command. See [Module 9](m09-control-orchestration.md) for protocol
details.

### Future: Jetson (optional)

An NVIDIA Jetson-based stack is a viable alternative for builders who prefer
Linux-native deployment or need higher throughput. The HTTP interface is
platform-agnostic — switching compute platforms is a drop-in replacement.
See [Appendix D](appendices.md#d-jetson-notes-future-alternative-stack).

## Design Approach

### Two-Stage Classification

Separate part shape recognition from color recognition. This dramatically reduces
the class space compared to a single model handling every part × color combination.
A 2×4 brick exists in 50+ colors; by separating shape from color, the shape model
only needs to learn ~3,000 shape classes, and the color model handles ~60 standard
LEGO colors.

### Multi-View Fusion

The multi-mirror scanning station produces multiple views per brick in a single
frame. The classification engine processes each view through a shared feature
extractor and combines features before final classification. Two approaches to
evaluate:

- **Multi-branch fusion model:** one model with parallel input branches (one per
  view), shared backbone, combined at a fusion layer before the classification head
- **Per-view ensemble:** independent models per view with a voting/aggregation layer

The multi-branch approach is more elegant and likely more accurate; the ensemble
approach is easier to develop incrementally.

### Model Architecture

Target for v1: EfficientNet-B0 or MobileNetV3 (fast, Neural Engine optimised in
deployment mode). Evaluate against ConvNeXt-T (Brickognize's architecture) once
baseline is established.

In development mode, larger models (ResNet-50, ConvNeXt-T) can be used directly
since the Mac has no model-size constraint. Distillation or architecture search
to find the best deployment-sized model happens before CoreML export.

### CoreML Export (Deployment Only)

Models are trained in PyTorch on Mac, then exported to CoreML format
(`.mlpackage`) via `coremltools` when ready for iPhone deployment. This is a
late-stage step — most development and iteration happens in PyTorch only.

### Hybrid Bootstrapping with Brickognize

Use the Brickognize API as a reference oracle during early development:

1. Classify each brick locally AND via Brickognize API
2. Compare results
3. Use disagreements to identify training priorities
4. Over time, local model improves and API dependency decreases

Brickognize recognizes nearly all LEGO parts using a ConvNeXt-T model and provides
a free API at `api.brickognize.com/docs`.

### Synthetic Training Data

Render from the LDraw parts library (3D CAD models of virtually every LEGO part)
via Blender and BlenderProc. Key references:

- **Brickognize research paper** demonstrates photo-realistic rendering with domain
  randomization (randomized lighting, textures, backgrounds, camera positions) to
  bridge the synthetic-to-real gap.
- **Daniel West** generated 25M+ synthetic images from LDraw models. Achieved 93%
  on real parts; synthetic-only was insufficient without domain randomization and
  a small real-world dataset.
- **Existing datasets:** B200C (800k renders of 200 parts), publicly available
  photo+render collections (~155k photos, ~1.5M renders)
- **Existing rendering pipelines:** BrickRenderer, Lego Rendering Pipeline,
  BrickRegistration (all in awesome-lego-machine-learning repo)

Training images can be rendered from camera angles matching the specific mirror
geometry, so the model sees exactly the views it will encounter in production.

## Open Questions

- Multi-view fusion model vs. per-view ensemble — which performs better in practice?
- Should weight measurement (load cell) supplement visual classification?
- Minimum viable number of part classes for a useful v1? (200? 500? 1,000?)
- How to handle the long tail of rare parts that won't appear in most bulk lots?
- Training strategy for the color classifier — synthetic data, or real-world
  calibration under actual scanner lighting?
- Which iPhone lens performs best at `#working_distance` (150mm)?
- Optimal image streaming format/resolution from iPhone to Mac in dev mode?
