# Stereo Visual Dataset

Factor graphs for stereo visual Bundle Adjustment experiments, built from KITTI odometry sequences.
Ground-truth trajectories are stored in TUM format under `gt/`.

## Folder Structure

```
stereo_visual/
├── gt/                  # Ground-truth trajectories (TUM format)
│   ├── 00.tum
│   ├── ...
│   └── 10.tum
├── multirobot/          # Multi-robot Bundle Adjustment datasets
│   ├── 00.g2o
│   ├── ...
│   └── 10.g2o
└── rolling_shutter/     # Rolling shutter simulation datasets
    ├── kitti_06_0.g2o
    ├── kitti_06_0.0001.g2o
    ├── kitti_06_0.001.g2o
    ├── kitti_06_0.01.g2o
    └── kitti_06_0.1.g2o
```

## `multirobot/`

G2O files produced by running ORB-SLAM on KITTI odometry sequences. Each file contains the
initial pose estimates, stereo landmark observations, and camera parameters for the full
sequence. The trajectory can be split across any number of robots at runtime via the
`num_robots` argument of `stereo_visual`.

## `rolling_shutter/`

G2O files for KITTI sequence 06 with a simulated rolling shutter effect. The suffix in
each filename is the **readout time** (in seconds): the time elapsed between the exposure
of the first and last row of the image sensor. A readout time of `0` corresponds to a
global shutter (no distortion). The values used correspond to those evaluated in the paper.

| File | Readout time |
|------|-------------|
| `kitti_06_0.g2o` | 0 s (global shutter) |
| `kitti_06_0.0001.g2o` | 0.0001 s |
| `kitti_06_0.001.g2o` | 0.001 s |
| `kitti_06_0.01.g2o` | 0.01 s |
| `kitti_06_0.1.g2o` | 0.1 s |
