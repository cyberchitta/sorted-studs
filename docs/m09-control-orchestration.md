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

Three hardware nodes, two compute layers:

### Hardware Nodes

| Node    | Role                                            | Connectivity    |
| ------- | ----------------------------------------------- | --------------- |
| Arduino | Motor control, sensors, beam-break, diverter    | Serial to ESP32 |
| ESP32   | WiFi bridge between Arduino and application     | WiFi + Serial   |
| iPhone  | Camera capture (dev: capture only; deploy: +ML) | WiFi            |
| Mac     | Dev only: ML inference, orchestration, UI       | WiFi            |

### Compute Layers

**Real-time layer (Arduino + ESP32):** motor control, sensor reading, beam-break
detection, diverter positioning. The Arduino handles deterministic timing. The
ESP32 bridges the Arduino's serial interface to WiFi, exposing it as HTTP
endpoints. The ESP32 runs no application logic — it translates between serial
commands and HTTP requests.

**Application layer (Mac or iPhone):** camera capture, image preprocessing,
ML inference, inventory updates, UI. In development mode, the Mac fills this
role. In deployment mode, the iPhone does.

### Communication Protocol

All communication between the application layer and the real-time layer uses
HTTP over local WiFi. The ESP32 exposes these endpoints:

**From application layer → ESP32 (commands):**

- `POST /diverter/route` `{ "bin": N }` — route current brick to bin N
- `POST /feeder/speed` `{ "duty_cycle": 0.0–1.0 }` — set feed rate
- `POST /system/pause` / `POST /system/resume` — pause/resume mechanical flow

**From ESP32 → application layer (events):**

- `POST /events/brick_detected` `{ "position": "scan_zone", "timestamp": T }`
- `POST /events/diverter_ready` `{ "timestamp": T }`
- `POST /events/jam_detected` `{ "stage": N, "timestamp": T }`

The ESP32 pushes events to a configurable callback URL — pointed at the Mac
during development, at the iPhone during deployment. Switching between modes
is a single URL configuration change on the ESP32.

### Development Mode

```
iPhone ──(HTTP: frames)──▶ Mac ──(HTTP: route cmd)──▶ ESP32 ──(serial)──▶ Arduino
                            ▲                            │
                            └──(HTTP: sensor events)─────┘
```

The Mac runs the full application stack: receives frames from iPhone, runs
PyTorch inference, updates inventory, sends routing commands to ESP32. The
iPhone runs only a minimal camera capture server.

### Deployment Mode

```
ESP32 ──(HTTP: brick event)──▶ iPhone ──(capture + infer)──▶ iPhone
                                  │
ESP32 ◀──(HTTP: route cmd)───────┘
ESP32 ──(serial)──▶ Arduino
```

The iPhone handles everything: receives brick-detected events from ESP32,
captures a frame, runs CoreML inference, and responds with a routing command.
No Mac in the loop.

## ESP32 Bridge Firmware

The ESP32 firmware is minimal:

- Connect to local WiFi network
- Listen for HTTP commands, translate to serial messages for Arduino
- Read serial messages from Arduino, POST as events to callback URL
- Configurable callback URL (set via serial command or hardcoded per mode)
- No application logic — pure transport bridge

This keeps the ESP32 code simple and stable. All intelligence stays in the
application layer.

## Scan Cycle (Both Modes)

1. Arduino detects brick in scan zone (beam-break sensor)
2. ESP32 POSTs `brick_detected` event to application layer
3. Application layer triggers iPhone camera capture
   - Dev mode: Mac sends `POST /capture` to iPhone, receives JPEG
   - Deploy mode: iPhone captures locally (no network hop)
4. Application layer runs classification → gets part ID, color ID, confidence
5. Application layer sends `POST /diverter/route { "bin": N }` to ESP32
6. ESP32 relays route command to Arduino over serial
7. Arduino positions diverter, releases brick
8. Arduino signals `diverter_ready` → ESP32 POSTs to application layer
9. Next brick released from singulation stage

## Design Reference

BrickSortingMachine implements this as independent Python services communicating
via TCP sockets with a simple message-passing protocol. Their approach of
decoupled services is a good model. In sorted-studs the equivalent is the
clean HTTP boundary between compute layers — each side is independently
restartable and replaceable.

## Open Questions

- Latency: can we scan one brick while the previous brick is being diverted
  (pipelining)?
- ESP32 connection model: persistent WebSocket vs. request/response per event?
- Should the ESP32 buffer events if the application layer is temporarily
  unreachable, or halt the mechanical flow?
- OTA firmware update for ESP32 over WiFi during development?
