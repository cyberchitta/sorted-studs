# System-Wide Parameters

These variables are defined in the OnShape Variable Studios and referenced
across multiple modules. Update here when values change.

## Brick Envelope

| Variable            | Value | Notes                                        |
| ------------------- | ----- | -------------------------------------------- |
| `#brick_sphere_dia` | 80mm  | 90% of all LEGO parts fit within this sphere |
| `#brick_max_height` | 38mm  | Tallest common brick upright + margin        |

## Interface Dimensions

| Variable                  | Value | Notes                                                          |
| ------------------------- | ----- | -------------------------------------------------------------- |
| `#channel_width`          | 90mm  | Stage 2 / scan platform entry width                            |
| `#channel_wall_height`    | 40mm  | Minimum containment height above V-bottom                      |
| `#channel_v_angle`        | 45°   | V half-angle (both singulation stages)                         |
| `#stage1_width`           | 120mm | Stage 1 (clump breaker) opening width — hopper exit must match |
| `#exit_width`             | 90mm  | Singulator exit = scan platform entry                          |
| `#exit_height_off_ground` | 120mm | Placeholder — revisit after diverter design                    |
| `#slide_angle`            | 20°   | Gravity slide angle                                            |
| `#scan_floor_length`      | 80mm  | Flat scan zone length                                          |
| `#lap_step`               | 4mm   | Stage 2 floor height above scan platform floor                 |

## Material

| Variable        | Value | Notes                          |
| --------------- | ----- | ------------------------------ |
| `#al_thickness` | 1.5mm | Aluminum sheet stock thickness |

## Scan Station

| Variable             | Value | Notes                                             |
| -------------------- | ----- | ------------------------------------------------- |
| `#acrylic_thickness` | 3mm   | Standard clear acrylic sheet stock                |
| `#camera_fov_width`  | 120mm | Conservative FOV at working distance              |
| `#mirror_angle`      | 45°   | Side mirror angle for 90° view redirect           |
| `#working_distance`  | 150mm | Camera lens to brick surface — verify empirically |

## Print Clearances

| Variable                 | Value | Notes                             |
| ------------------------ | ----- | --------------------------------- |
| `#print_clearance`       | 0.3mm | Gap between mating printed parts  |
| `#print_clearance_loose` | 0.5mm | Gap for sliding or rotating parts |
