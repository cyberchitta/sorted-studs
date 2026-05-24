# Module 9: Control System & Orchestration

**Status:** To build
**Dependencies:** All hardware modules (2, 3, 5), all software modules (4, 6–8)
**Feeds into:** Coordinates everything

---

## Purpose

Coordinate all physical modules (motors, sensors, cameras, diverter) and software
modules (classification, inventory, planning) into a coherent real-time pipeline.

## Requirements

- Real-time coordination: trigger camera capture when brick is positioned, send
  classification result to diverter before next brick arrives
- Graceful handling of jams, misfeeds, and mechanical faults
- State machine: idle, feeding, scanning, classifying, diverting, paused, error
- Event logging for debugging and performance analysis

## Architecture

### Hardware Nodes

| Node               | Role                                                        | Connectivity         |
| ------------------ | ----------------------------------------------------------- | -------------------- |
| Arduino            | Motor control, sensors, beam-break, diverter positioning    | Serial to ESP32      |
| ESP32 bridge       | WiFi ↔ serial bridge between Arduino and Mac                | WiFi + Serial        |
| ESP32-CAM (×4)     | One per tetrahedral vertex; HTTP `/capture` returns JPEG    | WiFi (`cameraN.local`) |
| Mac                | Application host: capture orchestration, ML, inventory, UI  | WiFi                 |

### Compute Layers

**Real-time layer (Arduino + ESP32 bridge):** motor control, sensor reading,
beam-break detection, diverter positioning. The Arduino handles deterministic
timing. The ESP32 bridges the Arduino's serial interface to WiFi, exposing it
as HTTP endpoints. The ESP32 runs no application logic — it translates
between serial commands and HTTP requests.

**Capture layer (ESP32-CAM ×4):** independent capture nodes, one at each
tetrahedral vertex. Each runs stock `CameraWebServer` firmware and serves
JPEGs on demand via HTTP. No on-board inference; no buffering beyond the
single most-recent frame. See [Module 3](m03-multi-view-scanner.md) for the
per-camera spec.

**Application layer (Mac):** image preprocessing, ML inference, inventory
updates, UI, and the orchestrator that drives the scan cycle. The Mac is
the single source of routing decisions.

### Communication Protocol

All communication between the application layer and the embedded hardware
uses HTTP over local WiFi.

**Mac → ESP32 bridge (Arduino commands):**

- `POST /diverter/route` `{ "bin": N }` — route current brick to bin N
- `POST /feeder/speed` `{ "duty_cycle": 0.0–1.0 }` — set feed rate
- `POST /system/pause` / `POST /system/resume` — pause/resume mechanical flow

**ESP32 bridge → Mac (Arduino events):**

- `POST /events/brick_detected` `{ "position": "scan_zone", "timestamp": T }`
- `POST /events/diverter_ready` `{ "timestamp": T }`
- `POST /events/jam_detected` `{ "stage": N, "timestamp": T }`

The ESP32 bridge pushes events to a configurable callback URL pointed at the
Mac. The Mac's URL is set on the ESP32 once (via serial command at provision
time or as a hardcoded build constant).

**Mac → ESP32-CAM (×4) (capture):**

- `GET http://cameraN.local/capture` — returns JPEG of the current frame.
  Mac fans 4 parallel GETs during the scanner's IMAGE-station dwell window.

### Scan Cycle

```
Arduino  ──(serial: brick_detected)──▶  ESP32 bridge
                                              │
                                              ▼
Mac  ◀──(HTTP POST /events/brick_detected)────┘
 │
 │  fan-out 4 parallel HTTP GETs
 ├──▶ camera1.local/capture ──▶ JPEG ──┐
 ├──▶ camera2.local/capture ──▶ JPEG ──┤
 ├──▶ camera3.local/capture ──▶ JPEG ──┼──▶ Mac: classify (PyTorch)
 └──▶ camera4.local/capture ──▶ JPEG ──┘            │
                                                    ▼
Mac  ──(HTTP POST /diverter/route {"bin": N})──▶ ESP32 bridge ──(serial)──▶ Arduino
                                                                                │
Mac  ◀──(HTTP POST /events/diverter_ready)───── ESP32 bridge ◀──(serial)───────┘
```

1. Arduino detects brick in scan zone (beam-break sensor)
2. ESP32 bridge POSTs `brick_detected` event to the Mac
3. Mac fans 4 parallel HTTP GETs to `cameraN.local/capture`
4. Mac runs preprocessing + multi-view classification → part ID, color ID,
   confidence
5. Mac sends `POST /diverter/route { "bin": N }` to the ESP32 bridge
6. ESP32 bridge relays the route command to Arduino over serial
7. Arduino positions diverter, releases brick
8. Arduino signals `diverter_ready` → ESP32 bridge POSTs to Mac
9. Next brick released from the singulator (Module 2)

## ESP32 Bridge Firmware

Minimal:

- Connect to local WiFi network
- Listen for HTTP commands, translate to serial messages for Arduino
- Read serial messages from Arduino, POST as events to the Mac's callback URL
- Mac callback URL configurable via serial command or build-time constant
- No application logic — pure transport bridge

## Design Reference

[BrickSortingMachine](https://github.com/bricksortingmachine/bricksortingmachine)
implements decoupled Python services communicating via TCP sockets, each
independently restartable. sorted-studs takes the same decoupling principle
but draws the line at the embedded/application boundary: the Mac runs as
one process (or a small set of processes with intra-process queues), the
embedded side is purely transport. The clean HTTP boundary between the Mac
and the ESP32 bridge / ESP32-CAM nodes is the equivalent split — each
side is independently restartable and replaceable.

## Future: Edge-accelerator host

If the application layer ever moves off the Mac to a Coral-on-Pi or Jetson,
the HTTP contracts above are unchanged — only the host running the
orchestrator differs. The ESP32 bridge and the ESP32-CAM nodes don't know
which device is calling them.

## Open Questions

- Latency: can we scan one brick while the previous brick is being diverted
  (pipelining)?
- ESP32 bridge connection model: persistent WebSocket vs. request/response
  per event?
- Should the ESP32 bridge buffer events if the Mac is temporarily
  unreachable, or halt the mechanical flow?
- Capture-time synchronization: is per-camera shutter sync (e.g. via the
  brick_detected timestamp) needed, or is "all 4 GETs complete within the
  IMAGE-station dwell" sufficient?
- OTA firmware update for ESP32 bridge / ESP32-CAM over WiFi during
  development?
