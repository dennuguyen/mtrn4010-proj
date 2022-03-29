# Project 1

## Run Instructions

1. Run `Solution_ABCD.m`.

## Data Legend

- table [3x3068 uint32]: table of events?
    - 1: timestamp
    - 2: index
    - 3: type data
- vw [2x2337 single]: Speed encoder and gyro data.
- scans [321x731 uint16]: LiDAR ranges with each pass.
- verify [1x1 struct]:
    - poseL [3x731 single]: Truth for verification of platform poses over time (10 ms sampling).
- pose0 [3x1 single]: Initial platform pose (m, m, rad).
- n 3068: number of events
- LidarCfg [1x1 struct]: Info about LiDAR installation.
    - Lx 0.4000: 
    - Ly 0: 
    - Alpha 0: 
- Walls [2x38 single]: Truth for verification of walls.
- Landmarks [2x26 single]: Truth for verification of landmarks.