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
| Arduino Nano                 | Motor/sensor control    | ~$5       |
| DC motor ("280") + driver    | Drum feeder             | ~$5       |
| DC motor ("280") + DRV8871   | Vibratory feeder (v1)   | ~$8       |
| 3D-printed parts (~500g–1kg) | Frame, scoops, channels | ~$15      |

### Added for sorted-studs

| Component                              | Purpose                             | Est. Cost |
| -------------------------------------- | ----------------------------------- | --------- |
| Raspberry Pi 4/5                       | Camera, ML inference, orchestration | ~$60      |
| Pi HQ Camera (12MP) + lens             | Multi-view capture                  | ~$50      |
| First-surface mirrors (×2–3)           | Side/bottom views                   | ~$15      |
| LED strip (5000K, high-CRI) + diffuser | Scan lighting                       | ~$10      |
| IR beam-break sensors (×2–3)           | Brick position detection            | ~$8       |
| Clear acrylic sheet                    | Bottom view window                  | ~$5       |
| Additional 3D-printed parts (~500g)    | Light box, mirror mounts            | ~$10      |

### Total: ~$200–250

Does not include 3D printer (~$200–300 for Bambu Lab A1 Mini or Prusa MINI+) or
optional GPU for training.

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
