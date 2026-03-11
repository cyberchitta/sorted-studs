# Reference Projects & Prior Art

## Mechanical Sorting Machines

| Project                              | Key Contribution                                                                                                                                                  | Build Material   | Links                                                                                                         |
| ------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------- | ------------------------------------------------------------------------------------------------------------- |
| **Hackaday.io Colour Sorter (RobG)** | **Base platform for sorted-studs.** 3D-printed drum feeder, vibratory singulation, gravity slide, rotary diverter. 20 STL files, 14-component BOM, 23 build logs. | 3D-printed       | [Hackaday.io](https://hackaday.io/project/194170)                                                             |
| BrickSortingMachine                  | Open-source SW stack. Camera + mirror scanner. Survey of all sorter designs.                                                                                      | LEGO Technic     | [GitHub](https://github.com/BrickSortingMachine/BrickSortingMachine), [Blog](https://bricksortingmachine.com) |
| Daniel West's Universal Sorter       | CNN-based universal recognition. 18-bin air-jet diverter. ResNet-50 on synthetic data.                                                                            | LEGO             | [YouTube](https://youtube.com)                                                                                |
| Paco Garcia's TensorFlow Sorter      | 3D-printed + PVC conveyor. Published STLs. 5-part blog series.                                                                                                    | 3D-printed + PVC | [Thingiverse](https://thingiverse.com/thing:3377239), [Medium](https://medium.com/@pacogarcia3)               |
| Nexus/Sorter (basicallysource)       | 3D-printed, YOLO-based. V2 in development with community Discord.                                                                                                 | 3D-printed       | [GitHub](https://github.com/basicallysource/sorter), [Website](https://basically.website)                     |
| pbackx's sort_bot                    | EV3 + Pi. No training required. Low barrier.                                                                                                                      | LEGO EV3         | [Codeberg](https://codeberg.org/peterb/sort_bot)                                                              |
| legosorter.io                        | In-progress blog. Good for learning from real iteration.                                                                                                          | Mixed            | [legosorter.io](https://legosorter.io)                                                                        |

## Recognition & Classification

| Project     | Key Contribution                                                     | Links                                                                               |
| ----------- | -------------------------------------------------------------------- | ----------------------------------------------------------------------------------- |
| Brickognize | Best-in-class API. ConvNeXt-T. Free. Nearly all parts/sets/minifigs. | [brickognize.com](https://brickognize.com), [API](https://api.brickognize.com/docs) |
| RebrickNet  | Rebrickable's AI. ~300 parts, 93% part accuracy.                     | [Rebrickable](https://rebrickable.com/rebricknet/)                                  |
| Instabrick  | Commercial scanner + inventory. BrickLink integration.               | [instabrick.org](https://instabrick.org)                                            |

## Training Data & Rendering

| Resource                      | Description                                 | Links                                                             |
| ----------------------------- | ------------------------------------------- | ----------------------------------------------------------------- |
| awesome-lego-machine-learning | Master index of all ML + LEGO projects.     | [GitHub](https://github.com/360er0/awesome-lego-machine-learning) |
| LDraw Parts Library           | 3D CAD models of virtually every LEGO part. | [ldraw.org](https://ldraw.org)                                    |
| BlenderProc                   | Procedural synthetic data generation.       | [GitHub](https://github.com/DLR-RM/BlenderProc)                   |
| B200C Dataset                 | 800k renders for 200 parts.                 | In awesome-lego-machine-learning                                  |

## APIs & Data Sources

| Service         | Purpose                         | Links                                                        |
| --------------- | ------------------------------- | ------------------------------------------------------------ |
| Rebrickable API | Parts, sets, MOCs, inventories. | [rebrickable.com/api/](https://rebrickable.com/api/)         |
| Brickognize API | Image → part recognition.       | [api.brickognize.com/docs](https://api.brickognize.com/docs) |
| BrickLink       | Part sourcing, pricing.         | [bricklink.com](https://bricklink.com)                       |
