# Module 6: Inventory Database & Rebrickable Sync

**Status:** To build
**Dependencies:** [Module 4 — Classification](module-04-classification-engine.md) (classification results as input)
**Feeds into:** [Module 7 — Set Coverage](module-07-set-coverage-planner.md), [Module 10 — UI](module-10-user-interface.md)

---

## Purpose

Maintain a persistent local inventory of all scanned bricks, and synchronize
bidirectionally with Rebrickable's online inventory system.

## Requirements

- Store per-brick: part_id, color_id, quantity, physical_location (bin),
  scan_timestamp, confidence_score, source_image_reference
- Sync with Rebrickable REST API: push inventory, pull set/MOC part lists
- Manual correction support (user overrides misclassifications)
- Multiple collections (e.g., different bulk lot batches)

## Functionality

- CRUD operations on inventory records
- Bulk import/export (CSV, Rebrickable XML format)
- Rebrickable API integration: authenticate, push part lists, pull set/MOC inventories
- Conflict resolution for bidirectional sync
- Audit trail: every scan event logged with raw image, classification result, confidence

## Data Model (simplified)

```
Inventory:    { part_id, color_id, quantity, bin_location, collection_id }
ScanEvent:    { event_id, part_id, color_id, confidence, image_ref, timestamp }
Collection:   { collection_id, name, source_description, date_added }
```

## Design Reference

- Rebrickable REST API: `rebrickable.com/api/`
- Bricqer's Brickognize integration demonstrates a real-time scan → recognize →
  confirm → add-to-inventory workflow worth studying for UX patterns
