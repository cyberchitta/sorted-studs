# sorted-studs

**An open-source AI-powered LEGO brick sorting, cataloging, and build-planning system.**

sorted-studs takes a pile of loose LEGO bricks and turns them into buildable sets. It mechanically singulates and scans each brick, identifies it using computer vision, catalogs it into a digital inventory synced with Rebrickable, determines which official sets or MOCs can be built from the current collection, and generates step-grouped "build packets" — bagged groups of bricks corresponding to stages of the build instructions.

The project serves as both a practical tool for LEGO enthusiasts dealing with bulk unsorted bricks and a portfolio-grade demonstration of applied ML, computer vision, and mechatronics integration.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Module 1: Bulk Storage & Feed System](#module-1-bulk-storage--feed-system)
3. [Module 2: Singulation Stage](#module-2-singulation-stage)
4. [Module 3: Multi-View Scanning Station](#module-3-multi-view-scanning-station)
5. [Module 4: Brick Classification Engine](#module-4-brick-classification-engine)
6. [Module 5: Diverter & Bin Output](#module-5-diverter--bin-output)
7. [Module 6: Inventory Database & Rebrickable Sync](#module-6-inventory-database--rebrickable-sync)
8. [Module 7: Set Coverage & Build Planner](#module-7-set-coverage--build-planner)
9. [Module 8: Packet Generator](#module-8-packet-generator)
10. [Module 9: Control System & Orchestration](#module-9-control-system--orchestration)
11. [Module 10: User Interface](#module-10-user-interface)
12. [Reference Projects & Prior Art](#reference-projects--prior-art)
13. [Appendices](#appendices)

---

## System Overview

### Physical Flow

```
 ┌──────────┐    ┌──────────────┐    ┌──────────────┐    ┌────────────┐    ┌───────────┐
 │  BULK    │───▶│ SINGULATION  │───▶│  MULTI-VIEW  │───▶│  DIVERTER  │───▶│  OUTPUT   │
 │ STORAGE  │    │    STAGE     │    │   SCANNER    │    │   STAGE    │    │   BINS    │
 └──────────┘    └──────────────┘    └──────────────┘    └────────────┘    └───────────┘
                                           │
                                           ▼
                                    ┌──────────────┐
                                    │CLASSIFICATION│
                                    │   ENGINE     │
                                    └──────┬───────┘
                                           │
                                           ▼
                                    ┌──────────────┐    ┌──────────────┐    ┌───────────┐
                                    │  INVENTORY   │───▶│ SET COVERAGE │───▶│  PACKET   │
                                    │   DATABASE   │    │  & PLANNER   │    │ GENERATOR │
                                    └──────────────┘    └──────────────┘    └───────────┘
```

### Data Flow

```
Camera frame (single image, multiple views via mirrors)
  → Image preprocessing (crop per view, normalize, background subtract)
    → Classification model (part ID + color ID + confidence)
      → Inventory record { part_id, color_id, quantity, bin_location, timestamp }
        → Rebrickable sync (push inventory, pull set/MOC part lists)
          → Coverage analysis (which sets can be built, what's missing)
            → Packet generation (group parts by instruction step ranges)
```

### Design Principles

- **Modular construction.** Each physical stage is a self-contained unit that can be developed, tested, and improved independently. Stages connect via simple mechanical interfaces (a brick drops or slides from one stage to the next).
- **Software-defined sorting.** The physical diverter doesn't need to know what a brick is. It receives a simple command ("send to bin N") from the classification engine. All intelligence lives in software.
- **Fail-safe defaults.** Any brick the system can't confidently identify goes to a designated "uncertain" bin for manual review or re-scanning, rather than being misclassified.
- **Open source, reproducible.** All 3D-printed parts are designed for common FDM printers (≥200mm build plate). Electronics use widely available off-the-shelf components. Software dependencies are open source.

---

## Module 1: Bulk Storage & Feed System

### Purpose

Hold a large quantity of unsorted bricks and progressively feed them into the singulation stage at a controlled rate, without jamming or overloading downstream stages.

### Requirements

- Hold at least 2–3 liters of mixed loose bricks (roughly 500–1000 pieces).
- Feed bricks at a controllable rate, no faster than the singulation stage can process.
- Handle the full range of brick sizes, from 1×1 round plates to large baseplates and Technic beams, without jamming.
- Minimize manual intervention once loaded.

### Design Approach

Two dominant approaches exist in prior art. The first is a "belt under pile" design, where a slow-moving conveyor belt forms the floor of a hopper and gradually pushes bricks toward the singulation stage. Daniel West, Jacques Mattheij, and BrickSortingMachine all use variants of this. The second is a "step feeder" or elevator design, where a vertical mechanism scoops small batches from a bin and drops them onto the next stage. Akiyuki's NXT sorter and Francisco Garcia's TensorFlow sorter use this approach.

The belt-under-pile approach is mechanically simpler and better suited to 3D printing. The BrickSortingMachine project notes an improvement: piling bricks directly on the belt rather than having the belt move beneath a stationary pile, which prevents voids from forming in the stored bricks.

### Key Components

- Hopper bin (3D-printed or modified storage container)
- Low-speed conveyor belt or auger feed mechanism
- Geared motor with speed control
- Optional: load sensor or optical gate to detect when hopper is empty

### Open Questions

- What's the optimal hopper geometry to prevent bridging and jams with mixed brick sizes?
- Should the feed rate be fixed or dynamically adjusted based on downstream singulation performance?

---

## Module 2: Singulation Stage

### Purpose

Take the stream of bricks from bulk storage and separate them so that exactly one brick at a time arrives at the scanning station. This is widely regarded as the hardest mechanical problem in the entire system.

### Requirements

- Output: single bricks, spaced far enough apart that the scanner captures one at a time.
- Handle bricks that arrive overlapping, stacked, or lightly interlocked.
- Minimal brick loss (bricks shouldn't fall off the machine).
- Tolerant of the full size range of bricks.

### Design Approach

Almost every documented sorter uses some form of vibration-based singulation. The general pattern is: a slow conveyor or gravity feed delivers a clump of bricks onto a vibrating channel or plate. The vibration shakes them apart so they travel single-file. A speed-differential belt step then spaces them out longitudinally — bricks drop from a slow belt onto a fast belt, increasing the gap between consecutive pieces.

Daniel West's design chains a primary belt, secondary belt, and vibration feeder. BrickSortingMachine uses v-channel vibration feeders. The Hackaday.io colour sorter uses a drum feeder into a vibratory tray with variable amplitude along its length.

### Key Components

- Vibration feeder: V-channel or flat plate driven by an eccentric motor or voice-coil actuator
- Speed-differential belt transition (slow belt → fast belt)
- Guide rails to keep bricks on the path
- Optical gate or beam-break sensor at the exit to confirm single-brick passage

### Open Questions

- What vibration frequency and amplitude works best for the widest range of brick sizes?
- How do we handle "problem pieces" like long Technic beams or flexible hoses that don't singulate well?
- Is a two-stage approach (vibrate then speed-differential) sufficient, or do we need three stages for reliability?

---

## Module 3: Multi-View Scanning Station

### Purpose

Capture high-quality images of each brick from multiple angles simultaneously, in a single camera frame, using mirrors. This is a core innovation of sorted-studs — most existing sorters use a single top-down view, which limits their ability to distinguish parts with identical profiles from one angle.

### Requirements

- Capture at least three simultaneous views: top-down, left-side profile, right-side profile.
- Stretch goal: capture a fourth bottom/anti-stud view via a transparent conveyor section and mirror.
- All views captured in a single camera exposure (no synchronization issues).
- Consistent, diffused, flicker-free lighting isolated from ambient conditions.
- Sufficient resolution that each individual view provides at least 500×500 usable pixels of the brick.
- Fast enough exposure to avoid motion blur at the conveyor's operating speed.

### Design Approach

This builds on the BrickSortingMachine's single camera + single mirror concept, extending it to a full multi-mirror enclosure.

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
          │  │      conveyor belt          │ │
          │  └─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┘ │
          │                                 │
          │         ┌─Mirror C─┐            │
          │         │  ~45°    │            │
          │         │  bottom  │            │
          │         │  view    │            │
          │         └──────────┘            │
          └─────────────────────────────────┘
```

The camera sees one composite image: the direct top-down view in the center, flanked by reflected side views, and optionally the bottom view reflected up from below. Each view occupies its own region of the sensor.

### Key Components

- Camera module: ≥12MP sensor, global shutter preferred to avoid rolling-shutter distortion on moving bricks
- 2–3 first-surface mirrors (front-coated to avoid ghosting from glass-back reflections)
- 3D-printed mirror mounts with fine-adjustment set screws
- Light box enclosure (opaque to ambient light)
- LED strip lighting: daylight-white (~5000K), diffused, high-CRI for accurate color detection
- Transparent conveyor section: clear acrylic or polycarbonate belt segment (for bottom view)
- Trigger mechanism: beam-break sensor or software-based motion detection to fire capture when a brick is centered

### Open Questions

- What mirror angles and distances give the best balance between view coverage and image resolution per view?
- Does the transparent conveyor section introduce enough visual noise (scratches, dust, belt texture) to degrade the bottom view?
- Should we use a rolling capture (continuous video with frame extraction) or a triggered still capture?
- How do we calibrate the geometric relationship between mirror views for consistent preprocessing?

---

## Module 4: Brick Classification Engine

### Purpose

Take the multi-view image from the scanning station and produce a classification: Rebrickable part ID, color ID, and confidence score.

### Requirements

- Identify part number from the Rebrickable parts catalog. The most common ~3,000 parts cover the vast majority of bulk lots, but the system should be extensible to the full catalog of ~70,000+ parts.
- Identify color separately from part shape (a 2×4 brick exists in 50+ colors).
- Provide a confidence score. Below a configurable threshold, the brick is routed to the "uncertain" bin.
- Run fast enough to keep up with the mechanical throughput (target: ≤2 seconds per brick).

### Design Approach

**Two-stage classification:** separate part shape recognition from color recognition. This dramatically reduces the class space for each model compared to a single model that must handle every part × color combination.

**Multi-view fusion:** the multi-mirror scanning station produces multiple views per brick. The classification engine should process each view through a shared feature extractor and combine the features before final classification. This is the ML showcase aspect of the project.

**Hybrid approach for bootstrapping:** use the Brickognize API as a reference oracle during early development. Brickognize already recognizes nearly all LEGO parts using a ConvNeXt-T model and provides a free API. Classify each brick locally AND via Brickognize, compare results, and use disagreements to identify training priorities. Over time, the local model improves and API dependency decreases.

**Synthetic training data:** use the LDraw parts library (3D CAD models of virtually every LEGO part) to render training images via Blender and BlenderProc. The Brickognize paper demonstrates that rendering scenes from multiple random camera angles with randomized lighting and physically-based textures, combined with domain randomization, bridges the synthetic-to-real gap effectively. Multiple existing rendering pipelines (BrickRenderer, Lego Rendering Pipeline, BrickRegistration) can be adapted.

**Existing datasets to bootstrap training:** the B200C dataset (800k renders of 200 parts), publicly available photo+render collections (~155k photos and ~1.5M renders), and conveyor-belt video datasets.

### Open Questions

- Is a multi-view fusion model (one model, multiple input branches) better than an ensemble of per-view models with a voting/aggregation layer?
- How do we handle the long tail of rare parts that won't appear in most bulk lots?
- Should weight/mass measurement (via a small load cell in the scanning station) supplement visual classification?
- What's the minimum viable number of part classes for a useful v1?

---

## Module 5: Diverter & Bin Output

### Purpose

Route each classified brick to the correct output bin based on the classification result.

### Requirements

- Minimum 6 output bins for a useful first version. 18+ bins for a mature system.
- Diversion must complete before the next brick arrives at the diverter.
- Gentle handling — bricks shouldn't be damaged or launched out of bins.
- "Uncertain" bin is always one of the outputs.

### Design Approach

Three main approaches exist in prior art. Daniel West uses air jets to blow bricks off a conveyor into side bins — fast but noisy and requires compressed air. Servo-driven gates or flaps on a conveyor are the most common DIY approach. Rotating carousel / turntable designs (like BrickSortingMachine's "Pinball Palace") use a rotating chute to direct bricks to bins arranged in a circle.

For a 3D-printed build, servo-driven gates or a rotating chute are most practical. The Hackaday.io colour sorter uses a stepper-driven diverter chute designed for low mass and fast repositioning. A cascade of binary diverters (left/right at each stage) can scale to many bins with simple hardware.

### Key Components

- Diverter mechanism: servo-driven gate, rotating chute, or cascade of binary flaps
- Output bins or bags (for packet generation, bins may be labeled bags)
- Position feedback to confirm diverter is in the correct position before releasing a brick

### Open Questions

- How many bins do we need for the v1 prototype? A small number (6–8) with coarse categories (plates, bricks, slopes, Technic, tiles, other) vs. a large number for fine-grained sorting?
- Can we use a "re-circulation" design where the machine loops bricks back through itself for progressively finer sorting across multiple passes?
- For packet generation (Module 8), do we route directly into labeled bags rather than fixed bins?

---

## Module 6: Inventory Database & Rebrickable Sync

### Purpose

Maintain a persistent local inventory of all scanned bricks, and synchronize bidirectionally with Rebrickable's online inventory system.

### Requirements

- Store: part_id, color_id, quantity, physical_location (which bin), scan_timestamp, confidence_score, source_image_reference.
- Sync with Rebrickable via their REST API: push local inventory up, pull set/MOC part lists down.
- Handle the case where manual corrections are made (user overrides a misclassification).
- Support multiple "collections" (e.g., different batches of bulk bricks from different sources).

### Functionality

- CRUD operations on inventory records.
- Bulk import/export (CSV, Rebrickable XML format).
- Rebrickable API integration: authenticate, push part lists, pull set inventories and MOC inventories.
- Conflict resolution for bidirectional sync.
- Audit trail: every scan event is logged with its raw image, classification result, and confidence.

### Reference

Rebrickable provides a well-documented REST API at `rebrickable.com/api/`. Bricqer's integration with Brickognize demonstrates a real-time scan-to-inventory workflow that's worth studying for UX patterns.

---

## Module 7: Set Coverage & Build Planner

### Purpose

Compare the current brick inventory against the part lists of official LEGO sets and community MOCs, and surface which builds are possible or nearly possible.

### Requirements

- Given the current inventory, produce a ranked list of buildable sets/MOCs ordered by percentage completeness.
- Identify exactly which parts are missing for each candidate build, with quantities.
- Support color-substitution mode: "you have all the right shapes but some in different colors."
- Link missing parts to BrickLink/BrickOwl for sourcing.

### Functionality

- Pull complete part lists for any set or MOC from the Rebrickable API.
- Coverage algorithm: for each set, compare required parts against inventory, computing percentage match and generating a missing-parts manifest.
- Filtering and ranking: filter by theme, year, piece count, or completeness threshold.
- "What can I build right now?" mode: only show 100% complete builds.
- "What's one order away?" mode: show builds at ≥90% with a BrickLink shopping list for the remainder.

---

## Module 8: Packet Generator

### Purpose

Given a chosen build (set or MOC), group the required bricks into "packets" — bagged groups corresponding to stages of the build instructions — so the builder can work through instructions without searching for parts.

### Requirements

- Parse or reference build instructions to determine which parts are used in which step ranges.
- Group parts into logical packets (e.g., "Bag 1: Steps 1–14", "Bag 2: Steps 15–28").
- Generate a printable pick list for each packet.
- Optionally integrate with the diverter to physically sort bricks directly into packet bags.

### Functionality

- Step-to-part mapping: determine which parts appear at which instruction steps. For official sets, Rebrickable provides part lists but not step-level granularity; instruction files (if available) or manual annotation may be needed.
- Packet optimization: minimize the number of bags while keeping each bag to a manageable size.
- Pick list generation: for each packet, produce a list with part images, quantities, and bin locations.
- Physical integration: when the sorter is running in "packet mode," the diverter routes parts directly into bags labeled for each packet.

### Open Questions

- Where do we get step-level part data? Official LEGO instructions PDFs don't have structured part-per-step data. Can we parse them with CV, or do we rely on community-annotated data?
- For MOCs, instruction formats vary wildly. What's the minimum standard we support?
- Is it more practical to do a two-pass approach: first sort into general inventory bins, then re-scan and sort into packet bags for a specific build?

---

## Module 9: Control System & Orchestration

### Purpose

Coordinate all physical modules (motors, sensors, camera, diverter) and software modules (classification, inventory, planning) into a coherent real-time pipeline.

### Requirements

- Real-time coordination: trigger camera capture when a brick is in position, send classification result to diverter before the next brick arrives.
- Graceful handling of jams, misfeeds, and mechanical faults.
- State machine for overall operation: idle, feeding, scanning, classifying, diverting, paused, error.
- Logging of all events for debugging and performance analysis.

### Design Approach

BrickSortingMachine implements this as independent services (Python processes) communicating via TCP sockets with a simple message-passing protocol, with the Raspberry Pi acting as a relay server. This pattern of decoupled services communicating over a message bus is well-proven and allows individual modules to be restarted or updated independently.

### Open Questions

- What's the right boundary between the embedded/real-time layer (motor control, sensor reading) and the application layer (classification, inventory)?
- How do we handle the latency of ML inference in the pipeline without stalling the mechanical flow?

---

## Module 10: User Interface

### Purpose

Provide visibility into the system's operation and access to the inventory, build planning, and packet generation features.

### Requirements

- Real-time dashboard: show current scanning activity, classification results with images, throughput rate, error rate.
- Inventory browser: search, filter, and manually correct the brick inventory.
- Build planner: browse buildable sets, view coverage, generate shopping lists.
- Packet manager: select a build, generate packets, print pick lists.
- System control: start/stop, adjust speed, view logs, trigger recalibration.

### Design Approach

A web-based dashboard is the most flexible — accessible from any device on the local network, no platform-specific development. The BrickSortingMachine project includes an HMI component. Bricqer's workflow (scan → recognize → confirm → add to inventory) is a good UX reference for the scanning interface.

---

## Reference Projects & Prior Art

### Sorting Machines (Hardware + Software)

| Project                             | Key Contribution                                                                                                                                      | Links                                                                                                              |
| ----------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| Daniel West's Universal LEGO Sorter | Pioneered CNN-based universal recognition. Vibration feeder + air jet diverter. ResNet-50 on synthetic data from LDraw. ~93% accuracy on known parts. | [YouTube](https://youtube.com), [Towards Data Science articles](https://towardsdatascience.com)                    |
| BrickSortingMachine                 | Open-source Python SW stack. Camera + mirror multi-view scanner. LEGO-built frame. Detailed design blog + survey of all existing sorters.             | [GitHub](https://github.com/BrickSortingMachine/BrickSortingMachine), [Blog](https://bricksortingmachine.com)      |
| Nexus / Sorter (basicallysource)    | Open-source with CAD designs. YOLO-based detection. Clean repo with firmware, robot control, UI.                                                      | [GitHub](https://github.com/basicallysource/sorter)                                                                |
| pbackx's sort_bot                   | EV3 + Raspberry Pi. Recently overhauled — no longer requires training your own model. Low barrier to entry.                                           | [Codeberg](https://codeberg.org/peterb/sort_bot), [Rebrickable MOC-90902](https://rebrickable.com/mocs/MOC-90902/) |
| legosorter.io                       | In-progress blog documenting full DIY build from scratch. Good for learning from real iteration.                                                      | [legosorter.io](https://legosorter.io)                                                                             |
| Hackaday.io Colour Sorter           | No-AI approach (color only). Excellent mechanical design for vibration feeder and low-inertia diverter.                                               | [Hackaday.io](https://hackaday.io/project/194170)                                                                  |

### Recognition & Classification

| Project     | Key Contribution                                                                                        | Links                                                                                    |
| ----------- | ------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| Brickognize | Best-in-class recognition API. ConvNeXt-T model. Free API. Recognizes nearly all parts, sets, minifigs. | [brickognize.com](https://brickognize.com), [API docs](https://api.brickognize.com/docs) |
| RebrickNet  | Rebrickable's native AI. ~300 parts, 93% part accuracy. Trained on community-submitted images.          | [Rebrickable](https://rebrickable.com/rebricknet/)                                       |
| Instabrick  | Commercial scanner + inventory system. BrickLink integration. Pick-to-light sorting.                    | [instabrick.org](https://www.instabrick.org)                                             |

### Training Data & Rendering

| Resource                      | Description                                                                                             | Links                                                             |
| ----------------------------- | ------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------- |
| awesome-lego-machine-learning | Master index of all ML + LEGO projects, datasets, papers, and tools.                                    | [GitHub](https://github.com/360er0/awesome-lego-machine-learning) |
| LDraw Parts Library           | Open-source 3D CAD models of virtually every LEGO part ever made.                                       | [ldraw.org](https://ldraw.org)                                    |
| BlenderProc                   | Procedural synthetic data generation in Blender. Used by Brickognize paper for photo-realistic renders. | [GitHub](https://github.com/DLR-RM/BlenderProc)                   |
| BrickRenderer                 | Render realistic training images from LDraw parts.                                                      | Listed in awesome-lego-machine-learning                           |
| B200C Dataset                 | 800k high-quality renders for 200 parts.                                                                | Listed in awesome-lego-machine-learning                           |

### APIs & Data Sources

| Service         | Purpose                                                            | Links                                                        |
| --------------- | ------------------------------------------------------------------ | ------------------------------------------------------------ |
| Rebrickable API | Set/MOC part lists, part catalog, color catalog, user inventories. | [rebrickable.com/api/](https://rebrickable.com/api/)         |
| Brickognize API | Image → part/set/minifig recognition.                              | [api.brickognize.com/docs](https://api.brickognize.com/docs) |
| BrickLink       | Part sourcing, pricing, marketplace.                               | [bricklink.com](https://bricklink.com)                       |

---

## Appendices

### A. Terminology

- **Singulation**: The process of separating a stream of objects into individual items spaced apart.
- **Part ID**: The Rebrickable/LDraw part number identifying a brick's shape (e.g., "3001" = 2×4 brick).
- **Color ID**: The Rebrickable color number (e.g., "0" = Black, "1" = Blue).
- **MOC**: "My Own Creation" — a community-designed build not officially sold by LEGO.
- **Anti-stud**: The underside pattern of a LEGO brick, including tubes, pin holes, and hollow studs.
- **First-surface mirror**: A mirror with the reflective coating on the front face of the glass, avoiding double reflections caused by standard back-coated mirrors.
- **Domain randomization**: A technique for training ML models on synthetic data by randomizing visual properties (lighting, texture, background) so the model generalizes to real-world conditions.

### B. Bill of Materials (Estimated, v1 Prototype)

| Category  | Components                                                                   | Est. Cost |
| --------- | ---------------------------------------------------------------------------- | --------- |
| Compute   | Raspberry Pi 4/5, camera module (HQ 12MP), microSD                           | ~$80      |
| Embedded  | Arduino or ESP32 for motor/sensor control                                    | ~$15      |
| Motors    | 2–3 geared DC motors (conveyor), 1–2 vibration motors, 2–3 servos (diverter) | ~$40      |
| Optics    | 2–3 first-surface mirrors, LED strip (daylight, high-CRI), diffuser panel    | ~$30      |
| Structure | 3D-printed parts (est. 1–2kg filament)                                       | ~$30      |
| Conveyor  | Silicone or rubber belt material, rollers                                    | ~$20      |
| Sensors   | 2–3 IR beam-break sensors                                                    | ~$10      |
| Misc      | Wiring, connectors, fasteners, power supply                                  | ~$25      |
| **Total** |                                                                              | **~$250** |

Optional: GPU-capable host machine for ML inference (can be an existing laptop/desktop with a discrete GPU, or inference can be offloaded to a cloud endpoint during development).

### C. Development Phases

**Phase 1 — Mechanical Proof of Concept:** Build Modules 1–3 and 5 with a minimal bin count (6). Manually verify that bricks can be fed, singulated, photographed from multiple angles, and diverted. No ML yet — classification is manual or via Brickognize API calls.

**Phase 2 — Classification Pipeline:** Build Module 4. Set up synthetic data rendering from LDraw. Train an initial model on the top ~200 most common parts. Benchmark against Brickognize API. Integrate multi-view fusion.

**Phase 3 — Inventory & Planning:** Build Modules 6 and 7. Rebrickable API integration. Coverage analysis. Web dashboard for inventory browsing and build planning.

**Phase 4 — Packet Generation:** Build Module 8. Step-level part mapping research. Pick list generation. Physical integration with diverter for bag-filling mode.

**Phase 5 — Polish & Scale:** Expand part recognition to 2,000+ parts. Increase bin count. Improve throughput. Harden error handling. Write documentation for community builders.

---

_sorted-studs is not affiliated with the LEGO Group. LEGO® is a trademark of the LEGO Group, which does not sponsor, authorize, or endorse this project._
