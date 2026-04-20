# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**sorted-studs** is an AI-powered LEGO brick sorting system. It mechanically singulates bricks, scans each one with a multi-view camera station (using mirrors for simultaneous angles), classifies part ID + color via ML, catalogs the inventory synced to Rebrickable, and generates "build packets" — bagged brick groups for instruction steps.

The repository is currently in the **specification/design phase** — it contains only documentation (`docs/`) and no source code yet.

## Repository Structure

- `docs/` — Module specifications (hardware and software)
- `README.md` — System overview, physical/data flow diagrams, module table, development phases

## Architecture

The system has two physical compute layers:

- **Arduino/ESP32** — real-time motor control, sensor reading, beam-break detection,
  diverter positioning (deterministic timing)
- **iPhone** — camera capture, image preprocessing, ML inference via CoreML Neural
  Engine. Communicates with Arduino over local WiFi using a simple HTTP protocol.
- **Mac** — model training, dataset pipeline, iterative development. Also serves as
  the inference host early in development before the iOS app is complete.

Communication between Arduino and iPhone: WiFi HTTP. Arduino POSTs sensor events
("brick in position"), iPhone responds with actions ("route to bin 7").

### Module Map

| #   | Module                       | Status                 |
| --- | ---------------------------- | ---------------------- |
| 1   | Drum Feeder                  | Build from base design |
| 2   | Vibratory Singulation        | Build from base design |
| 3   | Multi-View Scanning Station  | Original design        |
| 4   | Brick Classification Engine  | Original design        |
| 5   | Diverter & Bin Output        | TBD                    |
| 6   | Inventory & Rebrickable Sync | To build               |
| 7   | Set Coverage & Build Planner | To build               |
| 8   | Packet Generator             | To build               |
| 9   | Control & Orchestration      | To build               |
| 10  | User Interface               | To build               |

### Key Design Decisions

**Classification (Module 4):** Two-stage model — separate part shape (~3,000 classes)
from color (~60 classes) to avoid a combinatorial explosion. Multi-view fusion
preferred over per-view ensemble. Brickognize API (`api.brickognize.com`) used as
reference oracle during bootstrapping. Synthetic training data rendered from LDraw
models via Blender/BlenderProc. Models trained on Mac, exported to CoreML, deployed
on iPhone.

**Scanner (Module 3):** Single camera + multiple first-surface mirrors captures top, left profile, right profile (and optionally bottom) in one exposure. This is the core innovation over existing sorters that use single-view top-down only.

**Orchestration (Module 9):** Decoupled Python services communicating via TCP sockets (modeled after BrickSortingMachine project). Pi relays messages between services; allows independent restart/update of modules.

**Inventory (Module 6):** Local SQLite-style store with Rebrickable REST API sync. Data model: `Inventory { part_id, color_id, quantity, bin_location, collection_id }`, `ScanEvent { event_id, part_id, color_id, confidence, image_ref, timestamp }`.

### Confidence Threshold

Any brick the classifier can't confidently identify routes to an "uncertain" bin for manual review — fail-safe default throughout the design.

### External APIs

- **Rebrickable REST API** (`rebrickable.com/api/`) — inventory sync, set/MOC part lists
- **Brickognize API** (`api.brickognize.com`) — reference oracle for classification bootstrapping

### Base Mechanical Platform

Derived from the [Colour Sorting Machine for Lego](https://hackaday.io/project/194170-colour-sorting-machine-for-lego) (RobG, Hackaday.io). sorted-studs reuses the drum feeder → vibratory singulation → gravity slide → diverter mechanical flow and replaces the TCS34725 colour sensor + Arduino HSL classifier with the multi-view camera station + ML pipeline.
