# sorted-studs

**An open-source AI-powered LEGO brick sorting, cataloging, and build-planning system.**

sorted-studs takes a pile of loose LEGO bricks and turns them into buildable sets. It
mechanically singulates and scans each brick, identifies it using computer vision,
catalogs it into a digital inventory synced with Rebrickable, determines which official
sets or MOCs can be built from the current collection, and generates step-grouped
"build packets" — bagged groups of bricks corresponding to stages of the build
instructions.

sorted-studs is a practical tool for LEGO enthusiasts dealing with bulk unsorted
bricks. It is designed to be open-source and reproducible, so others can build and
iterate on their own systems.

---

## System Overview

### Physical Flow

```
 ┌──────────┐    ┌──────────────┐    ┌──────────────┐    ┌────────────┐    ┌───────────┐
 │   DRUM   │───▶│  VIBRATORY   │───▶│  MULTI-VIEW  │───▶│  DIVERTER  │───▶│  OUTPUT   │
 │  FEEDER  │    │ SINGULATION  │    │   SCANNER    │    │            │    │   BINS    │
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

- **Modular construction.** Each physical stage is a self-contained unit that can be
  developed, tested, and improved independently. Stages connect via simple mechanical
  interfaces (a brick drops or slides from one stage to the next).
- **Software-defined sorting.** The physical diverter receives a simple command
  ("send to bin N") from the classification engine. All intelligence lives in software.
- **Fail-safe defaults.** Any brick the system can't confidently identify goes to a
  designated "uncertain" bin for manual review or re-scanning.
- **Open source, reproducible.** All 3D-printed parts designed for common FDM printers
  (≥200mm build plate). Electronics use widely available off-the-shelf components.

### Base Mechanical Platform

The mechanical foundation is derived from the
[Colour Sorting Machine for Lego](https://hackaday.io/project/194170-colour-sorting-machine-for-lego)
by RobG on Hackaday.io. This project provides a proven, fully 3D-printed mechanical
platform with downloadable STL files, a complete bill of materials, and 23 detailed
build logs. The original machine sorts by colour only using an RGB sensor and Arduino.
sorted-studs reuses the mechanical flow (drum feeder → vibratory singulation → gravity
slide → diverter) and replaces the classification system with a multi-view camera
scanning station and ML pipeline.

| Base Design                      | sorted-studs Replacement                                                                      |
| -------------------------------- | --------------------------------------------------------------------------------------------- |
| TCS34725 colour sensor           | Multi-view camera scanning station ([Module 3](docs/m03-multi-view-scanner.md))               |
| Arduino Nano + HSL classifier    | Arduino (retained) + iPhone + CoreML pipeline ([Module 4](docs/m04-classification-engine.md)) |
| Small rotary diverter (few bins) | Larger diverter — design TBD ([Module 5](docs/m05-diverter.md))                               |

---

## Modules

### Hardware

| #   | Module                                                        | Status                         | Spec |
| --- | ------------------------------------------------------------- | ------------------------------ | ---- |
| 1   | [Drum Feeder & Bulk Storage](docs/m01-drum-feeder.md)         | Build from base design         | 📄   |
| 2   | [Vibratory Singulation](docs/m02-vibratory-singulation.md)    | Build from base design         | 📄   |
| 3   | [Multi-View Scanning Station](docs/m03-multi-view-scanner.md) | Original design (sorted-studs) | 📄   |
| 5   | [Diverter & Bin Output](docs/m05-diverter.md)                 | Design TBD — scalable          | 📄   |

### Software

| #   | Module                                                            | Status                         | Spec |
| --- | ----------------------------------------------------------------- | ------------------------------ | ---- |
| 4   | [Brick Classification Engine](docs/m04-classification-engine.md)  | Original design (sorted-studs) | 📄   |
| 6   | [Inventory & Rebrickable Sync](docs/m06-inventory-rebrickable.md) | To build                       | 📄   |
| 7   | [Set Coverage & Build Planner](docs/m07-set-coverage-planner.md)  | To build                       | 📄   |
| 8   | [Packet Generator](docs/m08-packet-generator.md)                  | To build                       | 📄   |
| 9   | [Control & Orchestration](docs/m09-control-orchestration.md)      | To build                       | 📄   |
| 10  | [User Interface](docs/m10-user-interface.md)                      | To build                       | 📄   |

### Reference

- [Reference Projects & Prior Art](docs/reference-projects.md)
- [Appendices (terminology, BOM, dev phases)](docs/appendices.md)

---

## Development Phases

**Phase 1 — Mechanical Foundation.** Print and build the base design (Modules 1, 2,
5-small). Verify bricks feed, singulate, and route reliably.

**Phase 2 — Multi-View Scanner.** Design and build the multi-mirror light box
(Module 3). Integrate camera + Pi. Validate multi-angle capture.

**Phase 3 — Classification Pipeline.** Synthetic data rendering, initial model training,
Brickognize API benchmarking (Module 4).

**Phase 4 — Inventory & Planning.** Rebrickable integration, coverage analysis, web
dashboard (Modules 6, 7, 10).

**Phase 5 — Diverter Upgrade.** Larger diverter for 18+ bins (Module 5).

**Phase 6 — Packet Generation.** Step-level mapping, pick lists, bag-filling (Module 8).

**Phase 7 — Polish & Scale.** 2,000+ parts recognition. Community documentation.

---

_sorted-studs is not affiliated with the LEGO Group. LEGO® is a trademark of the LEGO
Group, which does not sponsor, authorize, or endorse this project._
