# Appendices

## A. Terminology

- **Singulation**: Separating a stream of objects into individual items spaced apart.
- **Part ID**: Rebrickable/LDraw part number identifying a brick's shape (e.g., "3001"
  = 2×4 brick).
- **Color ID**: Rebrickable color number (e.g., "0" = Black, "1" = Blue).
- **MOC**: "My Own Creation" — a community-designed LEGO build.
- **Anti-stud**: The underside pattern of a LEGO brick (tubes, pin holes, hollow studs).
- **First-surface mirror**: Mirror with reflective coating on the front face, avoiding
  double reflections from standard back-coated glass.
- **Domain randomization**: ML training technique that randomizes visual properties in
  synthetic data so models generalize to real-world conditions.
- **Voice-coil actuator**: A solenoid driven at high frequency via PWM to produce
  controlled vibration.

## B. Bill of Materials (Estimated, v1 Prototype)

Costs are rough — final BOM will follow the first physical build.

### Mechanical

| Component                                  | Purpose                                 | Est. Cost |
| ------------------------------------------ | --------------------------------------- | --------- |
| 2020 aluminum extrusion (~10 m)            | Scanner rig, diverter columns, frames   | ~$30      |
| T-nuts, M4/M5 SHCS hardware                | Extrusion assembly                      | ~$15      |
| 3D-printed parts (~2 kg PLA/PETG)          | Brackets, rotors, baskets, mounts       | ~$30      |
| Clear cast acrylic sheet (Ø660 mm, 10 mm)  | Diverter top deck                       | ~$25      |
| Clear cast acrylic sheet (180×180×4 mm)    | Scanner platform (transparent aperture) | ~$5       |
| Bearings, slewing-ring or lazy-susan       | Diverter rotation                       | ~$15      |

### Electronics / compute

| Component                                  | Purpose                                 | Est. Cost |
| ------------------------------------------ | --------------------------------------- | --------- |
| 4× AI-Thinker ESP32-CAM (M12 lens variant) | Multi-view capture nodes                | ~$48      |
| 4× M12 lenses (focal TBD per testing)      | Brick framing at 250 mm working dist    | ~$20      |
| ESP32 dev board (WROOM-32)                 | WiFi ↔ serial bridge to Arduino         | ~$5       |
| Arduino Mega 2560                          | Motor / sensor / diverter control       | existing  |
| Stepper + driver (NEMA 17 + A4988)         | Singulator rotor drive                  | ~$15      |
| Hobby servo (SG90 / MG90S)                 | Scanner dump-lift actuator              | ~$3       |
| Slewing-ring + motor for diverter rotation | Hex tower drive                         | ~$40      |
| LED strip (5000K, high-CRI) + diffuser     | Scanner lighting                        | ~$10      |
| HW-201 IR beam-break sensors (×2–3)        | Brick position detection                | ~$5       |
| Mac (host)                                 | Capture orchestration + ML + UI         | existing  |

### Total: ~$260 in new parts (plus existing Mac + Arduino)

3D printer is not included (~$200–300 for a Bambu Lab A1 Mini or Prusa
MINI+ class machine). All printed parts are sized for ≥200 mm build
plates.

## C. Development Phases

Phase plan is maintained in the project README. This appendix only
records the historical sequencing for reference; the canonical order is
in `README.md`.

## D. Edge-accelerator notes (future alternative stack)

The current application host is a Mac (see [Module 4](m04-classification-engine.md)
and [Module 9](m09-control-orchestration.md)). For builders who want to
run the system standalone without a Mac, two viable edge paths exist:

**Coral USB on a Raspberry Pi (~$80 total).** Compile the trained
ConvNeXt V2 Tiny model to TFLite + Edge TPU. The Pi runs the
orchestrator + image preprocessing and dispatches inference to the
Coral. Throughput is comparable to a low-end Mac for this model size.

**NVIDIA Jetson Orin Nano (~$150).** Export PyTorch → TensorRT via
`torch2trt` or ONNX. Linux-native, higher throughput headroom than
Coral, but more power-hungry and more expensive.

**Integration cost is zero on the embedded side.** The HTTP contracts
in [Module 9](m09-control-orchestration.md) (commands to the ESP32
bridge, captures from `cameraN.local`) are platform-agnostic. Switching
hosts only changes the device that runs the orchestrator; the Arduino
and the four ESP32-CAM nodes don't know the difference.
