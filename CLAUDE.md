# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**sorted-studs** is an AI-powered LEGO brick sorting system. It mechanically singulates bricks, scans each one with a multi-view camera station (four ESP32-CAM nodes at the vertices of a regular tetrahedron around the brick), classifies part ID + color via ML on a Mac host, catalogs the inventory synced to Rebrickable, and generates "build packets" — bagged brick groups for instruction steps.

The repository is in a **second CAD pass** — three modules (singulator, scanner, diverter) have detailed Build123d / cad-khana assemblies under `cad/`, a build-in-public subsite publishes rendered exhibits, and the application/firmware code is still pending. Nothing has been fabricated yet.

## Repository Structure

- `README.md` — System overview, modules table, dev phases (public-facing).
  (No `docs/` — the aspirational module specs moved to `_notes/specs/`
  on 2026-07-21; accurate ones get written back per module as designs settle.)
- `cad/` — Build123d / cad-khana assemblies. One directory per module under
  active CAD (`m02_c_channel_singulator`, `m03_scanner`, `m05_diverter`).
  Each has its own `_notes/state/<module>.md` tracking the live
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

Status tiers: **Design** = `_notes/specs/` draft only; **CAD** = `cad/mNN_*/`
assembly in progress; **Built** = physically fabricated. Treat everything in
`_notes/specs/` as aspirational until rewritten — that is why it left `docs/`.

| #   | Module                       | Status |
| --- | ---------------------------- | ------ |
| 2   | C-Channel Singulator         | CAD    |
| 3   | Multi-View Scanning Station  | CAD    |
| 4   | Brick Classification Engine  | Design |
| 5   | Hex Tower Diverter           | CAD    |
| 6   | Inventory & Rebrickable Sync | Design |
| 7   | Set Coverage & Build Planner | Design |
| 8   | Packet Generator             | Design |
| 9   | Control & Orchestration      | Design |
| 10  | User Interface               | Design |

The C-channel singulator integrates the bulk-input hopper into its own
extended-wall lid; separate hopper variants (drum feeder, vibrating hopper)
are kept in `_notes/specs/` as design archive only.

### Key Design Decisions

**Scanner (Module 3):** Four ESP32-CAM nodes (AI-Thinker, M12 lens variant)
at the vertices of a regular tetrahedron centred on the brick — 1 front-low,
1 back-upper, 2 below (looking up through a transparent platform aperture).
Pairwise lens-axis separation is 109.47° — provably the most informative
4-view configuration. Multi-view fusion is the core innovation over existing
sorters that rely on a single top-down view. An earlier mirrors-and-one-camera
topology at the same four vertex positions was abandoned in 2026-05-04 for
cost, optical-tolerancing, and host-agnosticism reasons; see
`_notes/decisions/m03-cameras.md`.

**Singulator (Module 2):** Cylindrical C-channel (spinning faceted rotor
inside a fixed V-trough) with bricks riding the outer wall and falling off
an exit arc. Geometry from the Basically/Nexus sorter; replaces RobG's
flat V-channel. Extended-wall variant integrates the bulk-input hopper.

**Diverter (Module 5):** Hex tower cascade — 4-floor stack of hexagonal
basket rings around a central rotating column with per-floor ramp + trap.
6 through-columns at inner + outer hex vertices, off-axis motor driving a
ring gear on the base column's bottom collar. Compact vertical footprint;
scales by stacking floors. Architecture committed; per-sub-assembly state
in `_notes/state/m05-diverter.md`.

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
  gitignored here. Organised by **genre, not by source path** — one folder per
  kind, module as a filename prefix, depth 2 (flattened 2026-07-21 from a
  source-tree mirror):

  - `_notes/worklist.md` — index; says which file to open, holds no work itself
  - `_notes/state/<module>.md` — per-module live design state (`m03-scanner`,
    `m05-ramp-mechanism`, `cad-top-level`, `subsite`, …)
  - `_notes/decisions/<module>-<topic>.md` — durable rationale; stable paths,
    cited from `params.py` docstrings
  - `_notes/specs/<module>-<name>.md` — the former `docs/` module specs.
    **Aspirational, not descriptive** — they changed faster than the design
    and are not a source of truth; check against `state/` and the CAD
  - `_notes/backlog/<module>.md` — prospective features
  - `_notes/handoff/<topic>.md` — active task state
  - `_notes/archive/<module>-<topic>.md` — closed; `archive/README.md` has the gate
  - `_notes/lessons.md` — staging; drains to `cad/CLAUDE.md`, then to cad-khana
  - `_notes/explorers/*.html` — standalone three.js geometry explorers

  Invisible to the `Grep`/`Glob` tools (global `CLAUDE.md` has the `rg` forms).
  Start at `_notes/worklist.md`. Write new notes into `_notes/`, never beside
  the source.

- **CAD changes.** Read `cad/CLAUDE.md` before editing any assembly. Run
  `.venv/bin/khana check <path>/assembly.py` on the touched sub-assembly and
  the enclosing top-level after any geometry change. Each module's
  `_notes/state/<module>.md` is the live design state — update it in
  the same slice as the geometry change.
- **Subsite changes.** Edits stay uncommitted until the user signals
  "ready to publish."
- **Commits.** Write the message straight into git (heredoc on stdin),
  with a pathspec on `git commit`. See the global `CLAUDE.md` — this repo
  has no local exception.
