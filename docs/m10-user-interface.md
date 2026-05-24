# Module 10: User Interface

**Status:** To build
**Dependencies:** [Module 6 — Inventory](module-06-inventory-rebrickable.md), [Module 7 — Planner](module-07-set-coverage-planner.md), [Module 8 — Packets](module-08-packet-generator.md), [Module 9 — Orchestration](module-09-control-orchestration.md)

---

## Purpose

Provide visibility into the system's operation and access to inventory, build planning,
and packet generation features.

## Requirements

- **Real-time dashboard:** scanning activity, classification results with images,
  throughput rate, error rate
- **Inventory browser:** search, filter, manually correct the brick inventory
- **Build planner:** browse buildable sets, view coverage, generate shopping lists
- **Packet manager:** select a build, generate packets, print pick lists
- **System control:** start/stop, adjust speed, view logs, trigger recalibration

## Design Approach

Web-based dashboard served from the Mac and accessible from any device on the
local network. No platform-specific development needed. The Mac hosts the web
app alongside the classification, inventory, and orchestration services — all
the application-layer responsibilities (see [Module 9](m09-control-orchestration.md))
run in one place.

## Design Reference

- BrickSortingMachine includes an HMI component
- Bricqer's workflow (scan → recognize → confirm → add to inventory) is a good UX
  reference for the scanning interface
