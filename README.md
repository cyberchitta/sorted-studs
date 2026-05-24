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

## Status

**Second CAD pass; nothing fabricated yet.** Three modules
(singulator, scanner, diverter) have detailed Build123d assemblies under
`cad/` and rendered exhibits on the [build-in-public
subsite](https://sortedstuds.cyberchitta.cc). The first CAD pass never
reached fabrication; this second pass carries forward the lessons and
targets a workshop build. A handful of open mechanical questions
remain before the first physical print.

---

## System Overview

### Physical Flow

```
 ┌──────────────┐    ┌──────────────┐    ┌────────────┐    ┌───────────┐
 │  C-CHANNEL   │───▶│  MULTI-VIEW  │───▶│  HEX-TOWER │───▶│  OUTPUT   │
 │  SINGULATOR  │    │   SCANNER    │    │  DIVERTER  │    │   BINS    │
 └──────────────┘    └──────────────┘    └────────────┘    └───────────┘
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

### Compute Architecture

Three compute layers connected over local WiFi:

**Real-time layer** — Arduino handles motor control, sensors, and diverter
positioning. An ESP32 bridges the Arduino's serial interface to WiFi as HTTP
endpoints.

**Capture layer** — four ESP32-CAM nodes (AI-Thinker boards with the M12
lens variant) at the vertices of a regular tetrahedron around the brick.
Each runs stock `CameraWebServer` firmware exposing an HTTP `/capture`
endpoint as `cameraN.local`. Pure capture devices, no on-board inference.

**Application layer** — Mac runs the classification pipeline, inventory,
planning, and UI. During each scan the Mac fans 4 parallel HTTP GETs to
`cameraN.local/capture`, fuses the views, runs PyTorch inference, and
returns a routing decision to the Arduino. An iPhone-as-host path was
considered earlier and is deferred indefinitely; the Mac is the live
capture + inference host. A future edge accelerator (Coral / Jetson) can
take the Mac's role without changing the cameras.

### Data Flow

```
4 camera frames (one per tetrahedral vertex, captured in parallel)
  → Image preprocessing (per-view crop, normalize, background subtract)
    → Multi-view classification model (part ID + color ID + confidence)
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
- **Capture/inference decoupling.** The capture layer (ESP32-CAM nodes) and the
  inference layer (Mac) talk over HTTP only, so either side can be swapped
  independently — e.g. moving inference to an edge accelerator later for
  standalone operation without changing the cameras.
- **Open source, reproducible.** All 3D-printed parts designed for common FDM printers
  (≥200mm build plate). Mechanical structure is 2020 aluminum extrusion plus printed
  brackets and bodies. Electronics use widely available off-the-shelf components.

### Inspiration and Divergence

The project began as a port of the
[Colour Sorting Machine for Lego](https://hackaday.io/project/194170-colour-sorting-machine-for-lego)
by RobG on Hackaday.io, replacing the TCS34725 colour sensor with a
multi-view camera + ML pipeline. As the CAD work progressed the
constraint of "FDM-printable + aluminum extrusion only" pushed us
away from RobG's non-printed pieces (V-channel, rotary diverter); we
ended up at a singulator + diverter architecture much closer to
[the Basically LEGO Sorter](https://basically.website/sorter-v2) by
LegoSpencer and team. The classification approach (multi-view tetrahedral
camera station + ML) remains the sorted-studs original innovation.

| What changed                     | sorted-studs choice                                                                        |
| -------------------------------- | ------------------------------------------------------------------------------------------ |
| TCS34725 colour sensor           | 4-camera tetrahedral scanning station ([Module 3](docs/m03-multi-view-scanner.md))         |
| Arduino HSL classifier           | Mac + ConvNeXt V2 + 4× ESP32-CAM ([Module 4](docs/m04-classification-engine.md))           |
| Flat V-channel singulator        | Cylindrical C-channel with faceted rotor ([Module 2](docs/m02-c-channel-singulator.md))    |
| Small rotary diverter            | Hex tower cascade ([Module 5](docs/m05-diverter.md))                                       |

---

## Modules

Status tiers: **Spec** = `docs/mNN-*.md` only; **CAD** = `cad/mNN_*/` Build123d
assembly in progress; **Built** = physically fabricated and tested.

### Hardware

| #   | Module                                                        | Status | Spec |
| --- | ------------------------------------------------------------- | ------ | ---- |
| 2   | [C-Channel Singulator](docs/m02-c-channel-singulator.md)      | CAD    | 📄   |
| 3   | [Multi-View Scanning Station](docs/m03-multi-view-scanner.md) | CAD    | 📄   |
| 5   | [Hex Tower Diverter](docs/m05-diverter.md)                    | CAD    | 📄   |

The C-channel singulator integrates the bulk-input hopper into its own
extended-wall lid (the "Module 1" hopper / drum feeder is no longer a
separate stage). Earlier hopper and singulation variants are kept as
design archive under "Alternatives considered" below.

### Software

| #   | Module                                                            | Status | Spec |
| --- | ----------------------------------------------------------------- | ------ | ---- |
| 4   | [Brick Classification Engine](docs/m04-classification-engine.md)  | Spec   | 📄   |
| 6   | [Inventory & Rebrickable Sync](docs/m06-inventory-rebrickable.md) | Spec   | 📄   |
| 7   | [Set Coverage & Build Planner](docs/m07-set-coverage-planner.md)  | Spec   | 📄   |
| 8   | [Packet Generator](docs/m08-packet-generator.md)                  | Spec   | 📄   |
| 9   | [Control & Orchestration](docs/m09-control-orchestration.md)      | Spec   | 📄   |
| 10  | [User Interface](docs/m10-user-interface.md)                      | Spec   | 📄   |

### Alternatives considered

Kept as design archive — not part of the current build path.

- [Module 1 — Drum Feeder](docs/m01-drum-feeder.md) (from RobG's base design)
- [Module 1 — Vibrating Hopper](docs/m01-vibrating-hopper.md)
- [Module 2 — Two-Stage Vibratory Singulation](docs/m02-vibratory-singulation.md)

### Reference

- [Reference Projects & Prior Art](docs/reference-projects.md)
- [Appendices (terminology, BOM, dev phases)](docs/appendices.md)

---

## Repository Layout

```
docs/           Module specifications (the source of design intent).
cad/            Build123d / cad-khana assemblies — one directory per module
                under active CAD. Each has its own NEXT_STEPS.md tracking
                live design state. Top-level cad/assembly.py composes the
                modules that have integrated cross-checks (currently m05
                diverter + m03 turret unit).
subsite/        Build-in-public eleventy site (bun-managed). Published
                exhibits are GLB models exported from cad/.
pyproject.toml  uv-managed Python environment. cad-khana and chitra-cad
                are installed editable from sibling repos.
```

CAD tooling: [cad-khana](https://github.com/cyberchitta/cad-khana) wraps
Build123d with assembly-level interference/clearance assertions and
per-part printability checks. [chitra-cad](https://github.com/cyberchitta/chitra-cad)
renders the same assemblies through Blender Cycles for the exhibits.
Run checks with `.venv/bin/khana check <assembly.py>`.

---

## Development Phases

**Phase 1 — CAD foundation (current).** Detailed Build123d assemblies
for the singulator, scanner, and diverter, including interference and
clearance assertions across the integrated rig. The
[subsite](https://sortedstuds.cyberchitta.cc) publishes rendered
exhibits as modules stabilize.

**Phase 2 — First physical build.** Print and assemble Modules 2, 3, 5
together. Verify mechanical flow end-to-end with manual brick routing,
no classification yet.

**Phase 3 — Capture + classification.** Bring up the four ESP32-CAM
nodes against the printed scanner rig. Train ConvNeXt V2 on synthetic
LDraw renders (Module 4), benchmark against Brickognize API as a
reference oracle.

**Phase 4 — Inventory & planning.** Rebrickable integration, coverage
analysis, web dashboard (Modules 6, 7, 10).

**Phase 5 — Packet generation.** Step-level mapping, pick lists,
bag-filling (Module 8).

**Phase 6 — Polish & scale.** 2,000+ part recognition. Optional edge-
accelerator (Coral / Jetson) deployment for standalone operation
without a Mac. Community documentation.

---

_sorted-studs is not affiliated with the LEGO Group. LEGO® is a trademark of the LEGO
Group, which does not sponsor, authorize, or endorse this project._
