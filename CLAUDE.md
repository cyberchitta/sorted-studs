# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**sorted-studs** is an AI-powered LEGO brick sorting system. It mechanically singulates bricks, scans each one with a multi-view camera station (four ESP32-CAM nodes at the vertices of a regular tetrahedron around the brick), classifies part ID + color via ML on a Mac host, catalogs the inventory synced to Rebrickable, and generates "build packets" — bagged brick groups for instruction steps.

The repository is in a **second CAD pass** — three modules (singulator, scanner, diverter) have detailed Build123d / cad-khana assemblies under `cad/`, a build-in-public subsite publishes rendered exhibits, and the application/firmware code is still pending. Nothing has been fabricated yet.

## Repository Structure

- `README.md` — System overview, modules table, dev phases (public-facing).
- `docs/` — Module specifications (source of design intent).
- `cad/` — Build123d / cad-khana assemblies. One directory per module under
  active CAD (`m02_c_channel_singulator`, `m03_scanner`, `m05_diverter`).
  Each has its own `_notes/cad/<module>/NEXT_STEPS.md` tracking the live
  design state — see `_notes/` below.
  Top-level `cad/assembly.py` runs cross-module assertions.
  `cad/CLAUDE.md` documents the assembly decomposition conventions.
- `subsite/` — Build-in-public eleventy site (bun-managed). Edits stay
  uncommitted until the user signals a publish.
- `pyproject.toml` + `uv.lock` + `.venv/` — uv-managed Python environment.
  `cad-khana` and `chitra-cad` are installed editable from sibling repos.
- `AGENTS.md` — Cross-tool agent guidance (Claude Code, Codex CLI).

## Architecture

The system has three physical compute layers connected over local WiFi:

- **Arduino + ESP32 bridge** — real-time motor control, sensor reading,
  beam-break detection, diverter positioning (deterministic timing). The
  ESP32 translates the Arduino's serial interface to HTTP for the Mac.
- **ESP32-CAM nodes (×4)** — one per tetrahedral vertex; each runs stock
  CameraWebServer firmware exposing an HTTP `/capture` endpoint over
  WiFi as `cameraN.local`. Pure capture devices, no inference on-board.
- **Mac** — application host. Fans 4 parallel HTTP GETs to
  `cameraN.local/capture` during each IMAGE-station dwell, runs image
  preprocessing, ML inference (PyTorch), inventory updates, and the UI.
  Communicates with the Arduino/ESP32 over HTTP.

An earlier iPhone-as-host path was considered (capture-only in dev,
capture+inference via CoreML/ANE in deployment) and is deferred
indefinitely. Future swap path is an edge accelerator (Coral / Jetson)
taking the Mac's role without changing the cameras.

### Module Map

Status tiers: **Spec** = `docs/` only; **CAD** = `cad/mNN_*/` assembly in
progress; **Built** = physically fabricated.

| #   | Module                       | Status |
| --- | ---------------------------- | ------ |
| 2   | C-Channel Singulator         | CAD    |
| 3   | Multi-View Scanning Station  | CAD    |
| 4   | Brick Classification Engine  | Spec   |
| 5   | Hex Tower Diverter           | CAD    |
| 6   | Inventory & Rebrickable Sync | Spec   |
| 7   | Set Coverage & Build Planner | Spec   |
| 8   | Packet Generator             | Spec   |
| 9   | Control & Orchestration      | Spec   |
| 10  | User Interface               | Spec   |

The C-channel singulator integrates the bulk-input hopper into its own
extended-wall lid; separate hopper variants (drum feeder, vibrating hopper)
are kept under `docs/` as design archive only.

### Key Design Decisions

**Scanner (Module 3):** Four ESP32-CAM nodes (AI-Thinker, M12 lens variant)
at the vertices of a regular tetrahedron centred on the brick — 1 front-low,
1 back-upper, 2 below (looking up through a transparent platform aperture).
Pairwise lens-axis separation is 109.47° — provably the most informative
4-view configuration. Multi-view fusion is the core innovation over existing
sorters that rely on a single top-down view. An earlier mirrors-and-one-camera
topology at the same four vertex positions was abandoned in 2026-05-04 for
cost, optical-tolerancing, and host-agnosticism reasons; see
`_notes/cad/m03_scanner/cameras_design_notes.md`.

**Singulator (Module 2):** Cylindrical C-channel (spinning faceted rotor
inside a fixed V-trough) with bricks riding the outer wall and falling off
an exit arc. Geometry from the Basically/Nexus sorter; replaces RobG's
flat V-channel. Extended-wall variant integrates the bulk-input hopper.

**Diverter (Module 5):** Hex tower cascade — 4-floor stack of hexagonal
basket rings around a central rotating column with per-floor ramp + trap.
6 through-columns at inner + outer hex vertices, off-axis motor driving a
ring gear on the base column's bottom collar. Compact vertical footprint;
scales by stacking floors. Architecture committed; per-sub-assembly state
in `_notes/cad/m05_diverter/NEXT_STEPS.md`.

**Classification (Module 4):** Two-stage model — separate part shape from
color to avoid combinatorial explosion. ConvNeXt V2 Tiny backbone; metric
learning (à la Brickognize) preferred over standard classifier for the
70k-part scaling case. Brickognize API used as reference oracle during
bootstrapping. Synthetic training data rendered from LDraw models via
Blender/BlenderProc.

**Orchestration (Module 9):** Single application host (Mac) talks to the
Arduino/ESP32 bridge over HTTP. The ESP32 is a transport bridge only — no
application logic on the embedded side.

**Inventory (Module 6):** Local SQLite-style store with Rebrickable REST
API sync. Data model: `Inventory { part_id, color_id, quantity, bin_location,
collection_id }`, `ScanEvent { event_id, part_id, color_id, confidence,
image_ref, timestamp }`.

### Confidence Threshold

Any brick the classifier can't confidently identify routes to an "uncertain"
bin for manual review — fail-safe default throughout the design.

### External APIs

- **Rebrickable REST API** (`rebrickable.com/api/`) — inventory sync, set/MOC part lists
- **Brickognize API** (`api.brickognize.com`) — reference oracle for classification bootstrapping

## Working in this repo

- **Working notes — `_notes/`.** A symlink to a private `working-notes` repo,
  gitignored here. Its layout mirrors the source tree, so a note sits at the
  same relative path as the code it describes:

  - `_notes/cad/m03_scanner/NEXT_STEPS.md` — per-module live design state
  - `_notes/cad/m03_scanner/cameras_design_notes.md` — module design rationale
  - `_notes/cad/m05_diverter/ramp_mechanism/NEXT_STEPS.md` — sub-assembly state
  - `_notes/cad/archived/` — retired notes, folder layout mirroring `cad/`
  - `_notes/subsite/NEXT_STEPS.md` — subsite state

  Invisible to the `Grep`/`Glob` tools (global `CLAUDE.md` has the `rg` forms).
  `ls _notes/…` to discover what is there. Write new notes into `_notes/`,
  never beside the source.

- **CAD changes.** Read `cad/CLAUDE.md` before editing any assembly. Run
  `.venv/bin/khana check <path>/assembly.py` on the touched sub-assembly and
  the enclosing top-level after any geometry change. Each module's
  `_notes/cad/<module>/NEXT_STEPS.md` is the live design state — update it in
  the same slice as the geometry change.
- **Subsite changes.** Edits stay uncommitted until the user signals
  "ready to publish."
- **Commits.** Write the message straight into git (heredoc on stdin),
  with a pathspec on `git commit`. See the global `CLAUDE.md` — this repo
  has no local exception.
