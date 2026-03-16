# Module 9: Control System & Orchestration

**Status:** To build
**Dependencies:** All hardware modules (1–3, 5), all software modules (4, 6–8)
**Feeds into:** Coordinates everything

---

## Purpose

Coordinate all physical modules (motors, sensors, camera, diverter) and software
modules (classification, inventory, planning) into a coherent real-time pipeline.

## Requirements

- Real-time coordination: trigger camera capture when brick is positioned, send
  classification result to diverter before next brick arrives
- Graceful handling of jams, misfeeds, and mechanical faults
- State machine: idle, feeding, scanning, classifying, diverting, paused, error
- Event logging for debugging and performance analysis

## Architecture

Two compute layers:

- **Real-time layer (Arduino/ESP32):** motor control, sensor reading, beam-break
  detection, diverter positioning. Deterministic timing.
- **Application layer (iPhone):** camera capture, image preprocessing, ML inference
  via CoreML, inventory updates. Communicates with Arduino over local WiFi.

Communication between layers via WiFi HTTP. Simple command protocol: Arduino POSTs
sensor events ("brick detected at position X"), iPhone responds with actions
("route to bin 7"). The iPhone runs a lightweight local HTTP server for this purpose.

During development, the Mac can substitute for the iPhone in the application layer
using the identical HTTP interface — no Arduino code changes required when switching
between them.

## Design Reference

BrickSortingMachine implements this as independent Python services communicating via
TCP sockets with a simple message-passing protocol. Their approach of decoupled
services — each restartable and updatable independently — is a good model. In
sorted-studs the equivalent is the iPhone app's internal service structure, with
the same clean separation between capture, inference, and inventory.

## Open Questions

- Latency management: how to keep mechanical flow moving during ML inference?
- Buffer strategy: can we scan one brick while the previous brick is being diverted?
- Should the iPhone hold a persistent WiFi connection to the Arduino, or is
  request/response per brick sufficient given the mechanical timing budget?
