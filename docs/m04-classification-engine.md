# Module 4: Brick Classification Engine

**Status:** Original design (sorted-studs)
**Dependencies:** [Module 3 — Multi-View Scanner](module-03-multi-view-scanner.md) (image input)
**Feeds into:** [Module 5 — Diverter](module-05-diverter.md) (bin routing command), [Module 6 — Inventory](module-06-inventory-rebrickable.md) (classification result)

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

## Compute Platform

### Primary: iPhone (via CoreML)

The classification engine runs on an iPhone connected to the system over local WiFi.
The iPhone handles both image capture (Module 3) and inference, making it the sole
compute node for the vision pipeline. The Arduino handles all hardware control
(motors, sensors, diverter) independently.

The iPhone's Neural Engine runs CoreML models with low latency (~10–50ms for typical
classification models), well within the ≤2 second inference budget. This makes it
a stronger embedded inference platform than a Raspberry Pi for non-trivial models,
without the cost or complexity of a Jetson.

The iPhone acts as a local HTTP server. The Arduino POSTs a trigger when a brick is
in position; the iPhone captures, infers, and returns `route_to_bin(N)` with a
classification result. See [Module 9 — Control & Orchestration](module-09-control-orchestration.md)
for the full protocol.

**iPhone model selection:** Any iPhone with a Neural Engine (iPhone XS or later)
is sufficient. An iPhone 12 or later is recommended for the improved ISP and
optical zoom lens options.

### Development: Mac

Model training, dataset pipeline, and iterative development happen on a Mac.
Both platforms share CoreML as the inference runtime, so a model validated on
Mac deploys to iPhone without modification. The Mac also serves as the inference
host during early development before the iOS app is built, using the same
HTTP interface the Arduino expects.

### Future: Jetson (optional)

A separate NVIDIA Jetson-based stack can be developed as an alternative embedded
compute platform if needed — for example, for higher-throughput production builds
or community members who prefer a Linux-native deployment. The HTTP interface
between the compute node and Arduino is identical regardless of platform, so
switching is a drop-in replacement. See [Appendix D](#appendix-d-jetson-notes)
for notes.

## Design Approach

### Two-Stage Classification

Separate part shape recognition from color recognition. This dramatically reduces the
class space compared to a single model handling every part × color combination. A 2×4
brick exists in 50+ colors; by separating shape from color, the shape model only needs
to learn ~3,000 shape classes, and the color model handles ~60 standard LEGO colors.

### Multi-View Fusion

The multi-mirror scanning station produces multiple views per brick in a single frame.
The classification engine should process each view through a shared feature extractor
and combine features before final classification. Two approaches to evaluate:

- **Multi-branch fusion model:** one model with parallel input branches (one per view),
  shared backbone, combined at a fusion layer before the classification head
- **Per-view ensemble:** independent models per view with a voting/aggregation layer

The multi-branch approach is more elegant and likely more accurate; the ensemble
approach is easier to develop incrementally.

### CoreML Model Format

Models are trained in PyTorch or TensorFlow on Mac, then exported to CoreML format
(`.mlmodel` / `.mlpackage`) via `coremltools`. The exported model is bundled directly
into the iOS app. Model updates during development are a simple file replacement with
no interface changes.

Target model architecture for v1: EfficientNet-B0 or MobileNetV3 (fast, Neural Engine
optimised). Evaluate against ConvNeXt-T (Brickognize's architecture) once baseline
is established.

### Hybrid Bootstrapping with Brickognize

Use the Brickognize API as a reference oracle during early development:

1. Classify each brick locally AND via Brickognize API
2. Compare results
3. Use disagreements to identify training priorities
4. Over time, local model improves and API dependency decreases

Brickognize recognizes nearly all LEGO parts using a ConvNeXt-T model and provides a
free API at `api.brickognize.com/docs`.

### Synthetic Training Data

Render from the LDraw parts library (3D CAD models of virtually every LEGO part) via
Blender and BlenderProc. Key references:

- **Brickognize research paper** demonstrates photo-realistic rendering with domain
  randomization (randomized lighting, textures, backgrounds, camera positions) to
  bridge the synthetic-to-real gap. Training data was rendered from 20 random camera
  angles per scene using BlenderProc.
- **Daniel West** generated 25M+ synthetic images from LDraw models but found pure
  synthetic training insufficient — domain randomization and a small real-world dataset
  were needed to close the gap. Achieved 93% on real parts, 74% on synthetic-only.
- **Existing datasets:** B200C (800k renders of 200 parts), publicly available
  photo+render collections (~155k photos, ~1.5M renders)
- **Existing rendering pipelines:** BrickRenderer, Lego Rendering Pipeline,
  BrickRegistration (all listed in awesome-lego-machine-learning repo)

Critically, we can render training images from camera angles that match our specific
mirror geometry, so the model sees exactly the views it will encounter in production.

## Open Questions

- Multi-view fusion model vs. per-view ensemble — which performs better in practice?
- Should weight measurement (load cell) supplement visual classification?
- Minimum viable number of part classes for a useful v1? (200? 500? 1,000?)
- How do we handle the long tail of rare parts that won't appear in most bulk lots?
- What's the right training strategy for the color classifier — synthetic data, or
  real-world calibration under the actual scanner lighting?
- Which iPhone lens performs best at `#working_distance` (150mm)? Likely wide, but
  verify empirically.
