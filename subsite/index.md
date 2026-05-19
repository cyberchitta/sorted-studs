---
title: sorted-studs
description: An AI-powered LEGO sorting machine. Mechanical singulation, four-camera multi-view scanning, ML classification, automated bin output, Rebrickable-synced inventory. Building in public.
---

# sorted-studs

An AI-powered LEGO sorting machine. Mechanical singulation, four-camera multi-view
scanning, ML classification, automated bin output, Rebrickable-synced inventory.

Currently in spec and design phase. Building in public.


## Module 03 — Multi-view scanner

Four cameras at the vertices of a regular tetrahedron around the brick. Two profile
views, one top, one underside, all captured simultaneously during the IMAGE-station
dwell.

<!-- Four interactive panes go here -->

1. **Interactive 3D** — colored GLB, orbit controls *(placeholder)*
2. **Photo-real render** — studio-lit still *(placeholder)*
3. **Exploded view** — looping animation *(placeholder)*
4. **In motion** — turret indexing *(placeholder)*


## Built with three custom skills

The toolchain is the second story. Three Python skills, each matched to a phase:

- **cad-khana** — diagnostics-first design wrapper around Build123d. Assembly-level
  interference and clearance assertions, optional per-part printability checks.
- **chitra-cad** — CAD-specific photo-real rendering via Blender Cycles (`bpy`).
  Per-part materials, studio rig, auto-framed camera.
- **rig-gati** — parametric kinematics baked to GLTF animation tracks. Scrubs an
  assembly's joint parameters and exports the resulting transforms as animation.

The split is technical, not arbitrary. Design needs exact B-Rep geometry — exact
booleans, real interference checks, STEP export. Rendering only needs triangles,
so Blender's mesh-and-materials ecosystem wins. Animation needs the parametric
model re-evaluated per frame, with the resulting transforms baked onto the
tessellated GLTF. One tool per regime.


## Status

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

Mechanical platform derived from
[RobG's Colour Sorting Machine for Lego](https://hackaday.io/project/194170-colour-sorting-machine-for-lego);
the multi-view scanner and ML classifier replace the single-sensor colour pipeline.
