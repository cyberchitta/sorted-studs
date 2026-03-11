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
- **Application layer (Raspberry Pi):** camera capture, image preprocessing, ML
  inference, inventory updates, UI serving. Non-deterministic but fast.

Communication between layers via serial (USB) or I2C. Simple command protocol:
Arduino reports sensor events ("brick detected at position X"), Pi responds with
actions ("route to bin 7").

## Design Reference

BrickSortingMachine implements this as independent Python services communicating via
TCP sockets with a simple message-passing protocol. Their Raspberry Pi acts as a relay
server routing messages between services. This decoupled-services pattern allows
individual modules to be restarted or updated independently.

## Open Questions

- Latency management: how to keep mechanical flow moving during ML inference?
- Buffer strategy: can we scan one brick while the previous brick is being diverted?
- Should the Pi run inference locally (TFLite/ONNX) or offload to a networked GPU
  machine during development?
