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

### From base design (Hackaday.io Colour Sorter)

| Component                    | Purpose                 | Est. Cost |
| ---------------------------- | ----------------------- | --------- |
| NEMA 17 stepper (17HS3404-Z) | Diverter rotation       | ~$10      |
| A4988 stepper driver         | Stepper control         | ~$3       |
| Arduino Mega 2560            | Motor/sensor control    | existing  |
| DC motor ("280") + driver    | Drum feeder             | ~$5       |
| DC motor ("280") + DRV8871   | Vibratory feeder (v1)   | ~$8       |
| 3D-printed parts (~500g–1kg) | Frame, scoops, channels | ~$15      |

### Added for sorted-studs

| Component                               | Purpose                          | Est. Cost                 |
| --------------------------------------- | -------------------------------- | ------------------------- |
| iPhone XS or later (iPhone 12+ rec.)    | Camera capture + ML inference    | ~$200 (existing hardware) |
| ESP32 dev board (WROOM-32)              | WiFi bridge: Arduino ↔ app layer | ~$5                       |
| First-surface mirrors (×2–3)            | Side/bottom views                | ~$15                      |
| LED strip (5000K, high-CRI) + diffuser  | Scan lighting                    | ~$10                      |
| VL53L1X ToF sensor (×1–2)               | Brick detection + dimensions     | ~$5                       |
| Load cell (100g–500g) + HX711 amplifier | Brick weight measurement         | ~$5                       |
| Clear acrylic sheet                     | Bottom view window               | ~$5                       |
| Additional 3D-printed parts (~500g)     | Light box, mirror mounts         | ~$10                      |

### Total: ~$300–350

Does not include 3D printer (~$200–300 for Bambu Lab A1 Mini or Prusa MINI+). iPhone
assumed to be existing hardware; if purchasing, an iPhone 12 secondhand runs ~$150–200.
Mac assumed for development. Arduino Mega assumed existing.

## C. Development Phases

1. **Mechanical Foundation** — Print/build base design (Modules 1, 2, 5-small). Verify
   feed, singulate, and route.
2. **Multi-View Scanner** — Design multi-mirror light box (Module 3). Camera + Pi
   integration.
3. **Classification Pipeline** — Synthetic data, model training, Brickognize
   benchmarking (Module 4).
4. **Inventory & Planning** — Rebrickable integration, coverage, dashboard (Modules 6,
   7, 10).
5. **Diverter Upgrade** — Larger diverter for 18+ bins (Module 5).
6. **Packet Generation** — Step mapping, pick lists, bag-filling (Module 8).
7. **Polish & Scale** — 2,000+ parts, community docs.

## D. Jetson Notes (Future Alternative Stack)

The primary embedded compute platform is iPhone + CoreML (see Module 4). An
NVIDIA Jetson-based stack is a viable future alternative for builders who prefer
a Linux-native deployment or need higher throughput than the Neural Engine provides.

**Relevant Jetson options:**

- Jetson Nano (discontinued but widely available secondhand, ~$50–80) — marginal
  for serious models; roughly Pi-class for inference
- Jetson Orin Nano (~$150) — meaningfully faster; suitable for ResNet-50 class models
- Jetson Orin NX / AGX Orin — high-end, not needed for this application

**Integration:** The HTTP interface between compute node and Arduino (Module 9) is
platform-agnostic. A Jetson running the same inference server exposes the same
`route_to_bin(N)` API. No Arduino code changes required.

**Model format:** PyTorch models export to TensorRT on Jetson via `torch2trt` or
ONNX. This is a separate export path from CoreML but uses the same trained weights.

```

```
