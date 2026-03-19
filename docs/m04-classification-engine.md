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

Both platforms use the same model architecture (ConvNeXt V2 Tiny). The iPhone's
Neural Engine (35 TOPS on A18 Pro) is comparable to the Mac's (38 TOPS on M4) —
the same 16-core ANE design. There is no need for a separate mobile-optimized
architecture or model distillation.

### Development Mode: Mac + iPhone-as-Camera

During development, the Mac is the sole compute node for the vision pipeline.
The iPhone acts only as a camera, streaming frames to the Mac over local WiFi.

- **iPhone role:** capture frames on trigger, serve them via HTTP. No on-device
  inference.
- **Mac role:** receive frames, run classification (PyTorch), send
  `route_to_bin(N)` to ESP32, update inventory, serve UI.
- **Model format:** PyTorch (`.pt`). Full debugging tools — breakpoints,
  profiling, TensorBoard, fast iteration on architecture and training.

#### Camera Capture Interface

The iPhone runs a minimal capture app that exposes an HTTP endpoint:

- `POST /capture` — triggers a frame capture, returns the image as JPEG
- `GET /status` — returns camera readiness

The Mac polls or triggers this endpoint as part of the scan cycle. Latency
budget for the WiFi round-trip is ~50–100ms, well within the ≤2 second
inference window.

### Deployment Mode: iPhone (via CoreML → ANE)

For standalone deployment, the iPhone handles both image capture and inference,
making it the sole compute node for the vision pipeline. No Mac required.

- **iPhone role:** capture frames, run CoreML inference on Neural Engine, send
  `route_to_bin(N)` to ESP32, update inventory.
- **Model format:** CoreML (`.mlpackage`), exported from PyTorch via
  `coremltools`.
- **iPhone model selection:** any iPhone with a Neural Engine (iPhone XS or
  later). iPhone 12 or later recommended for the 16-core ANE.
- **Neural Engine performance:** ConvNeXt V2 Tiny inference is expected in
  low double-digit milliseconds, well within the ≤2 second budget.

The iPhone acts as a local HTTP server. The ESP32 POSTs sensor events
("brick in position"); the iPhone captures, infers, and responds with a
routing command. See [Module 9](m09-control-orchestration.md) for protocol
details.

#### ANE Export Constraints

The Orion project (see [References](reference-projects.md)) catalogs 20
constraints on MIL IR programs that affect CoreML→ANE compilation:

- **GELU activation** must be replaced with its tanh approximation. ConvNeXt
  uses GELU natively; this is a one-line substitution during `coremltools`
  export.
- **`concat` operations** cause ANE compilation failure — multi-tensor
  operations must be decomposed.
- **Tensor layout restrictions** on input/output buffer sizes and ordering.

These are export-time concerns, not architecture constraints. The PyTorch
model is unchanged; only the CoreML conversion handles these adaptations.

#### Deployment Tier 2: Direct ANE Dispatch (Stretch Goal)

Recent reverse-engineering of Apple's private `_ANEClient` and `_ANECompiler`
APIs has demonstrated 2–4× throughput over CoreML by bypassing its scheduling
overhead. This path is relevant if higher bricks-per-minute throughput is
needed, but carries risk: private APIs can break on any iOS/macOS update.

For current throughput targets, CoreML→ANE (Tier 1) is sufficient. Direct
dispatch is tracked as a future optimization. See maderix/ANE and Orion in
[References](reference-projects.md).

### Future: Jetson (optional)

An NVIDIA Jetson-based stack is a viable alternative for builders who prefer
Linux-native deployment or need higher throughput. The HTTP interface is
platform-agnostic — switching compute platforms is a drop-in replacement.
See [Appendix D](appendices.md#d-jetson-notes-future-alternative-stack).

## Design Approach

### Classification Strategy

There are three viable approaches to brick identification, each with
different trade-offs for our multi-view, high-class-count problem:

#### Approach A: Standard Multi-View Classifier

A ConvNeXt V2 Tiny backbone with parallel input branches (one per view),
shared feature extractor, fused at a late layer before a classification
head that outputs a Rebrickable part ID.

- **Pros:** single forward pass, well-understood training pipeline, direct
  confidence scores.
- **Cons:** adding new parts requires retraining. Class count growth
  (3,000→70,000) is a scaling challenge. Long-tail parts with few training
  examples will have poor accuracy.

#### Approach B: Metric Learning Image Search (à la Brickognize)

A ConvNeXt V2 Tiny backbone trained as a feature encoder using contrastive
or triplet loss. Each brick is encoded as a vector; classification is
nearest-neighbor search against a reference database of known part embeddings.

Brickognize uses this approach and covers nearly all LEGO parts ever made.
The key insight: the model learns whether two images show the same part,
not what specific part an image shows. This means:

- **Pros:** new parts added by inserting one reference image — no retraining.
  Scales to 70,000+ parts trivially. Graceful handling of rare parts. The
  multi-view scanner gives us multiple embeddings per brick to fuse,
  improving discrimination.
- **Cons:** requires a vector database and nearest-neighbor search step.
  Confidence calibration is less direct than softmax. Slightly more complex
  inference pipeline.

#### Approach C: Hybrid — Coarse Classifier + Metric Refinement

A two-stage pipeline: a fast classifier narrows candidates to a part
category or family (e.g., "2×N bricks", "slope elements", "Technic beams"),
then metric learning disambiguates within the category.

- **Pros:** fast rejection of obvious mismatches. Metric search operates
  over a smaller candidate set. Combines confidence from both stages.
- **Cons:** most complex to build and maintain. Two models to train.

#### Recommendation

Approach B (metric learning) is the primary path, based on Brickognize's
demonstrated success at scale and its natural fit with our multi-view
input. Approach C is the fallback if nearest-neighbor search latency
becomes a bottleneck at 70,000+ parts. Approach A is not recommended given
the class count scaling requirement.

### Two-Stage Classification

Separate part shape recognition from color recognition. This dramatically
reduces the class space compared to a single model handling every part ×
color combination. A 2×4 brick exists in 50+ colors; by separating shape
from color, the shape model only needs to learn shape features, and the
color classifier handles ~60 standard LEGO colors independently.

For the metric learning approach, color separation is even more natural:
the feature encoder learns shape-invariant embeddings, and a separate
lightweight color classifier operates on the raw pixel data from a
consistent region of the scan image.

### Model Architecture: ConvNeXt V2 Tiny

ConvNeXt V2 Tiny (28.6M parameters, ~4.5 GFLOPs) is the backbone for
both development and deployment. This is the same model on both platforms
— no distillation, no separate mobile architecture.

**Why ConvNeXt V2 Tiny:**

- **Best fine-tuning performance.** In systematic comparison across 14
  datasets and 7 domains, ConvNeXt-Tiny consistently outperforms
  EfficientNetV2-S, Swin-Tiny, RegNet, and others in resource-efficient
  fine-tuning. Higher ImageNet pretraining accuracy does not predict higher
  downstream accuracy — ConvNeXt wins on actual transfer performance.
- **Masked autoencoder pretraining.** ConvNeXt V2 adds a fully convolutional
  masked autoencoder framework, allowing self-supervised pretraining on
  unlabeled data. This directly benefits our synthetic-to-real domain gap:
  we can pretrain on large volumes of unrendered LDraw data.
- **Global Response Normalization (GRN).** Prevents feature collapse by
  scaling channels based on relative magnitude. Better feature utilization
  means better fine-grained discrimination between visually similar parts.
- **ANE compatible.** Pure CNN — no attention mechanisms or variable-length
  sequences. The ANE was designed for exactly this class of model. ConvNeXt
  uses LayerNorm (ANE-compatible) and GELU (requires tanh approximation
  substitution at export — one-line fix).
- **Size variants available.** If deployment constraints change, ConvNeXt V2
  has smaller variants (Atto 3.7M, Femto 5.2M, Pico 9.1M, Nano 15.6M)
  that use the same architecture, enabling drop-in scaling without
  rewriting the pipeline.

**Superseded candidates** (retained for reference):

- EfficientNet-B0 / MobileNetV3: a generation behind in fine-tuning
  accuracy. MobileNetV3 remains viable as an emergency fallback if ANE
  compatibility issues prove insurmountable with ConvNeXt.
- ResNet-50: solid baseline but outperformed by ConvNeXt V2 in every
  tested domain.
- Swin-Tiny: transformer overhead, worse than ConvNeXt in low-data
  fine-tuning, and more complex ANE export path.

### Multi-View Fusion

The multi-mirror scanning station produces multiple views per brick in a
single frame. The classification engine processes each view through a
shared feature extractor and combines features before final classification.

**Recommended approach: late fusion by feature concatenation.**

Research on multi-view CNN fusion (PLOS ONE, 2021) systematically evaluated
three strategies across multiple domains:

1. **Early fusion** — fusing convolutional feature maps at intermediate
   network depths
2. **Late fusion** — concatenating bottleneck representations before the
   classification/embedding head
3. **Score fusion** — independent per-view inference with post-hoc
   combination

Late fusion by feature vector concatenation with one fully connected layer
yielded the best accuracy increase while requiring half the parameters of
deep feature map fusion. This aligns with BrickSortingMachine's production
implementation, which fuses mid-level features from dual perspectives
jointly before the classification head.

For the metric learning approach, late fusion produces a single fused
embedding vector per brick from all views — this vector is what gets
compared against the reference database.

### Supplementary Signals: Weight and Dimensions

The scan station (Module 3) provides two non-visual measurement channels
alongside the camera image:

**Weight.** A load cell at the scan position measures brick mass. Multiple
sorting projects (Akiyuki's NXT sorter, DISBY) have demonstrated that weight
disambiguates visually similar parts and detects cases where multiple bricks
are presented simultaneously.

**Bounding box dimensions.** Time-of-flight sensors on the gravity slide
measure length, width, and height in millimeters. This provides absolute
scale that the camera inherently lacks — the camera sees proportions, not
physical dimensions. A 1×2 brick and a 2×4 brick have identical aspect
ratios from the top but differ by a factor of two in linear dimensions.

Together these produce a 4-scalar vector (length, width, height, mass) that
fuses with the visual embedding in the classification pipeline. For the
metric learning approach (Approach B), the scalar vector is concatenated
with the visual embedding before nearest-neighbor search, narrowing the
candidate set before fine-grained visual discrimination.

**Status:** committed for v1. Sensor integration is specified in Module 3.

### Hybrid Bootstrapping with Brickognize

Use the Brickognize API as a reference oracle during early development:

1. Classify each brick locally AND via Brickognize API
2. Compare results
3. Use disagreements to identify training priorities
4. Over time, local model improves and API dependency decreases

**Important:** Brickognize is a metric learning image search engine, not a
standard classifier. It converts each submitted image to a vector and finds
the nearest match in its reference database. This is the same fundamental
approach as our recommended Approach B. Comparing our multi-view embeddings
against Brickognize's single-view results will directly reveal the
discrimination value added by multiple perspectives.

Brickognize provides a free API at `api.brickognize.com/docs`.

### Synthetic Training Data

Render from the LDraw parts library (3D CAD models of virtually every LEGO
part) via Blender and BlenderProc. Key references:

- **Brickognize research paper** demonstrates photo-realistic rendering with
  domain randomization (randomized lighting, textures, backgrounds, camera
  positions) to bridge the synthetic-to-real gap. Synthetic pretraining
  combined with as few as 10 real images exceeded 90% segmentation accuracy.
- **Daniel West** generated 100k+ synthetic images from LDraw models.
  Achieved 93% on real parts; synthetic-only was insufficient without domain
  randomization and a small real-world dataset.
- **Existing datasets:** B200C (800k renders of 200 parts), publicly
  available photo+render collections (~155k photos, ~1.5M renders)
- **Existing rendering pipelines:** BrickRenderer, Lego Rendering Pipeline,
  BrickRegistration (all in awesome-lego-machine-learning repo)

Training images can be rendered from camera angles matching the specific
mirror geometry, so the model sees exactly the views it will encounter in
production.

For the metric learning approach, synthetic renders also populate the
initial reference database — each part gets reference embeddings from
rendered views, then these are progressively supplemented and replaced
with real-world embeddings from production scanning.

### CoreML Export

Models are trained in PyTorch on Mac, then exported to CoreML format
(`.mlpackage`) via `coremltools`. This export is part of the regular CI/test
cycle, not a one-time late-stage step — we validate ANE compatibility
continuously to catch operator issues early.

Export checklist:

- Substitute GELU with tanh approximation
- Verify all operations compile to ANE (no silent CPU/GPU fallback)
- Benchmark inference time on target iPhone hardware
- Validate classification accuracy matches PyTorch within tolerance

## Open Questions

- Metric learning embedding dimension — 128? 256? 512? Trade-off between
  discrimination and search speed at 70,000+ parts.
- Vector database for on-device nearest-neighbor search — approximate
  nearest neighbor (FAISS/Annoy) vs. exact search at our scale?
- Minimum viable number of part classes for a useful v1? (200? 500? 1,000?)
- How to handle decorated/printed parts — same mold but different prints?
- Training strategy for the color classifier — synthetic data, or real-world
  calibration under actual scanner lighting?
- Which iPhone lens performs best at working distance (~150mm)?
- Does the anti-stud (bottom) view from Module 3 add enough metric learning
  discrimination to justify the transparent section complexity?
- ConvNeXt V2 masked autoencoder pretraining — can we pretrain on raw LDraw
  renders (no labels) to bootstrap the feature encoder before supervised
  fine-tuning?
