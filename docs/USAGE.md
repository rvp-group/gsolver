# gsolver — Usage Guide

Detailed build, dataset, experiment, and API reference for **gsolver**.
For a quick overview and the fastest path to a first result, see the [main README](../README.md).

> Use the outline button (top-right of this file on GitHub) to jump between sections.

## 📁 Project Structure

```
gsolver/
├── gsolver/                    # Core C++ library
│   ├── include/gsolver/        # Public headers
│   │   ├── core/                # GBP solver, messages
│   │   ├── graph/               # Factor graph, nodes
│   │   └── maths/               # Geometry, lie algebra
│   └── src/                     # Implementation
├── python/                      # Python bindings (nanobind)
├── examples/
│   ├── cpp/                     # C++ experiments
│   │   ├── apps/                # Experiment executables
│   │   └── common/              # Shared utilities
│   ├── python/                  # Python experiments
│   │   ├── apps/                # Experiment scripts
│   │   └── common/              # Shared utilities
│   ├── config/                  # Experiment parameters
│   └── data/                    # Datasets and generators
└── cmake/                       # CMake configuration
```

## 🛠️ Building

### Docker (Recommended)

The easiest way to get started is via Docker, which handles all dependencies and provides X11 forwarding for visualization.

**Prerequisites:**
- [Docker](https://docs.docker.com/engine/install/) installed
- (Optional) [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) for GPU acceleration

**Build the image and start the container:**
```bash
./docker-run.sh
```

This script builds the `gsolver` Docker image (first run only) and launches the container with:
- the repository mounted at `/workspace`
- X11 forwarding configured for visualization
- GPU passthrough if the NVIDIA runtime is available

Once inside the container, build the project:
```bash
mkdir build && cd build
cmake .. -DGSOLVER_BUILD_EXAMPLES=ON -DGSOLVER_BUILD_PYTHON=ON
make -j$(nproc)
```

All subsequent commands in this guide assume you are running inside the container from `/workspace`.

### Native Build

#### Prerequisites

- **C++17** compatible compiler (GCC 9+, Clang 10+)
- **CMake** 3.16+
- **OpenMP** (for parallel GBP)
- **yaml-cpp** (for configuration files)
- **Python** 3.8+ (for Python bindings)

On Ubuntu/Debian:
```bash
sudo apt-get install build-essential cmake libomp-dev libyaml-cpp-dev python3-dev python3-venv
```

#### C++ Library

```bash
# Clone the repository
git clone git@github.com:rvp-group/gsolver.git
cd gsolver

# Create build directory
mkdir build && cd build

# Configure with examples and Python bindings
cmake .. -DGSOLVER_BUILD_EXAMPLES=ON -DGSOLVER_BUILD_PYTHON=ON

# Build
make -j$(nproc)
```

**CMake Options:**
| Option | Default | Description |
|--------|---------|-------------|
| `GSOLVER_BUILD_EXAMPLES` | OFF | Build C++ example executables |
| `GSOLVER_BUILD_PYTHON` | OFF | Build Python bindings |
| `GSOLVER_BUILD_TESTS` | OFF | Build unit tests (not yet provided) |

#### Python Package

**Option 1: Install from source (recommended for development)**
```bash
pip install -e .
```

**Option 2: Build wheel**
```bash
pip install build
python -m build
pip install dist/gsolver-*.whl
```

**Dependencies for examples:**
```bash
pip install numpy pyyaml rerun-sdk
```

## 📊 Generating Synthetic Datasets

The library includes a data generator for creating synthetic trajectories with configurable noise levels.

### Generate All Datasets

```bash
# Build the data generator first
cd build && cmake .. -DGSOLVER_BUILD_EXAMPLES=ON && make data_generator

# Run the generator script
cd ../examples/data/generators
./generate_all_datasets.sh
```

This creates:
- `datasets/synthetic/gt/` - Ground truth trajectories (TUM format)
- `datasets/synthetic/pgo/` - Pose Graph Optimization datasets (g2o format)
- `datasets/synthetic/prior/` - Prior factor experiment datasets

### Generate Individual Dataset

```bash
./build/examples/cpp/data_generator \
    --shape helix \
    --frequency 1 \
    --noise-position 0.01 \
    --noise-orientation 0.001 \
    --output examples/data/datasets/my_trajectory.g2o
```

### Learn GP Hyperparameters

To compute optimal `qc_diag` values for a trajectory, provide a ground-truth TUM file
(`timestamp tx ty tz qx qy qz qw`, one pose per line):

```bash
./build/examples/cpp/gp_hyperparam_trainer examples/data/datasets/synthetic/gt/sphere_100hz.tum
```

Output:
```
Suggested qc_diag for config file:
  qc_diag: [0.00514381, 0.00430491, 0.00550036, 0, 0, 0.000175324]
```

Add these values to `examples/config/experiment_params.yaml`.

## 🚀 Running Experiments

### Available Experiments

| Experiment | Description |
|------------|-------------|
| `pgo_experiment` | Pose Graph Optimization with GP priors |
| `prior_experiment` | GP prior factor evaluation |
| `hyperparam_ablation_experiment` | Qc hyperparameter sensitivity analysis |
| `charuco_experiment_ba` | Bundle Adjustment on Charuco datasets |
| `charuco_experiment_islam` | Incremental SLAM on Charuco datasets |
| `stereo_visual` | Stereo visual Bundle Adjustment (single-robot and multi-robot) |

### C++ Experiments

**Pose Graph Optimization:**
```bash
./build/examples/cpp/pgo_experiment \
    examples/data/datasets/synthetic/pgo/trajectories_1hz/noise_ig1.0_n0.001_helix.g2o \
    ./tmp/pgo_result.tum \
    helix \
    -visualize
```

**Prior Experiment:**
```bash
./build/examples/cpp/prior_experiment \
    examples/data/datasets/synthetic/prior/trajectories_1hz/noise_ig1.0_n0.001_helix.g2o \
    ./tmp/prior_result.tum \
    helix \
    -visualize
```

**Bundle Adjustment (Charuco):** the third argument is the initial-guess perturbation std (`0.0` = none); the config is fixed to `printing_room`.
```bash
./build/examples/cpp/charuco_experiment_ba \
    examples/data/datasets/charuco/printing_room_0.g2o \
    ./tmp/ba_result.tum \
    0.0 \
    -visualize
```

**Incremental SLAM (Charuco):** takes only input and output (config is fixed to `printing_room`).
```bash
./build/examples/cpp/charuco_experiment_islam \
    examples/data/datasets/charuco/printing_room_0.g2o \
    ./tmp/islam_result.tum \
    -visualize
```

**Hyperparameter Ablation:**
```bash
./build/examples/cpp/hyperparam_ablation_experiment \
    examples/data/datasets/synthetic/pgo/trajectories_1hz/noise_ig1.0_n0.001_helix.g2o \
    ./tmp/ablation_results/ \
    helix \
    -visualize
```

**Stereo Visual Bundle Adjustment — Multi-Robot:**
```bash
./build/examples/cpp/stereo_visual \
    examples/data/datasets/stereo_visual/multirobot/06.g2o \
    examples/data/datasets/stereo_visual/gt/06.tum \
    ./tmp/stereo_visual_result.tum \
    kitti_05 \
    8 \
    -visualize
```

**Stereo Visual Bundle Adjustment — Rolling Shutter:**
```bash
./build/examples/cpp/stereo_visual \
    examples/data/datasets/stereo_visual/rolling_shutter/kitti_06_0.1.g2o \
    examples/data/datasets/stereo_visual/gt/06.tum \
    ./tmp/stereo_visual_result.tum \
    kitti_05 \
    -visualize
```

### Python Experiments

**Pose Graph Optimization:**
```bash
python examples/python/apps/pgo_experiment.py \
    examples/data/datasets/synthetic/pgo/trajectories_1hz/noise_ig1.0_n0.001_helix.g2o \
    ./tmp/pgo_result.tum \
    helix \
    -visualize
```

**Prior Experiment:**
```bash
python examples/python/apps/prior_experiment.py \
    examples/data/datasets/synthetic/prior/trajectories_1hz/noise_ig1.0_n0.001_helix.g2o \
    ./tmp/prior_result.tum \
    helix \
    -visualize
```

**Bundle Adjustment (Charuco):** the third argument is the initial-guess perturbation std (`0.0` = none); the config is fixed to `printing_room`.
```bash
python examples/python/apps/charuco_experiment_ba.py \
    examples/data/datasets/charuco/printing_room_0.g2o \
    ./tmp/ba_result.tum \
    0.0 \
    -visualize
```

**Incremental SLAM (Charuco):** takes only input and output (config is fixed to `printing_room`).
```bash
python examples/python/apps/charuco_experiment_islam.py \
    examples/data/datasets/charuco/printing_room_0.g2o \
    ./tmp/islam_result.tum \
    -visualize
```

### Command Line Arguments

Most experiments share the pattern below, with the per-experiment exceptions noted underneath:

```bash
<experiment> <input.g2o> <output.tum> <trajectory-type> [-visualize]
```

| Argument | Description |
|----------|-------------|
| `input.g2o` | Input factor graph in g2o format |
| `output.tum` | Output trajectory in TUM format |
| `trajectory-type` | Config key from `experiment_params.yaml`: `helix`, `sphere`, `printing_room`, `kitti_05`, … |
| `-visualize` | Optional: enable Rerun 3D visualization |

Exact positional arguments per experiment:

| Experiment | Positional arguments |
|------------|----------------------|
| `pgo_experiment`, `prior_experiment` | `<input.g2o> <output.tum> <trajectory-type> [-visualize]` |
| `hyperparam_ablation_experiment` | `<input.g2o> <output_folder> <trajectory-type> [-visualize]` (output is a folder) |
| `charuco_experiment_ba` | `<input.g2o> <output.tum> <noise_level> [-visualize]` — config fixed to `printing_room`; `noise_level` is the initial-guess perturbation std (`0.0` = none) |
| `charuco_experiment_islam` | `<input.g2o> <output.tum> [-visualize]` — config fixed to `printing_room` |
| `stereo_visual` | `<input.g2o> <gt.tum> <output.tum> <trajectory-type> [num_robots] [-visualize]` |
| `gp_hyperparam_trainer` | `<input.tum>` |

### Configuration

Experiment parameters are in `examples/config/experiment_params.yaml`:

```yaml
helix:
  description: "Helical trajectory"
  qc_diag: [0.00514381, 0.00430491, 0.00550036, 0, 0, 0.000175324]
  num_iterations: 100
```

- `qc_diag`: GP motion prior hyperparameters `[tx, ty, tz, rx, ry, rz]`
- `num_iterations`: Number of GBP message-passing iterations

## 📖 API

The snippets below build a tiny two-state graph, run GBP for a fixed number of
iterations, and read back the optimized pose. The available variable and factor
types are `SE3PoseVel` / `SE3Pose` / `SE3Point` and `SE3PoseVelPrior`,
`SE3PoseVelPoseVel`, `SE3PoseVelGP`, `SE3PoseVelPose`, `SE3PoseVelPoint` (plus
`SE3PoseVelStereoPoint` in C++). For complete, runnable programs see
[`examples/cpp/apps/`](../examples/cpp/apps/) and
[`examples/python/apps/`](../examples/python/apps/), with shared graph-building
helpers in `examples/{cpp,python}/common/factor_graph_builder`.

### C++ API

```cpp
#include <gsolver/core/gbp_solver.h>
#include <gsolver/graph/factor_graph.h>
#include <gsolver/graph/types/variables/se3_posevel.h>
#include <gsolver/graph/types/factors/se3_posevel_posevel.h>
#include <gsolver/graph/types/factors/se3_posevel_prior.h>

using namespace gsolver;

FactorGraph graph;

// Two pose + velocity states, initialized at the identity
SE3PoseVelValue mu;
mu.pose = Eigen::Isometry3d::Identity();
mu.vel  = Eigen::Vector<double, 6>::Zero();
Eigen::Matrix<double, 12, 12> sigma = Eigen::Matrix<double, 12, 12>::Identity();

auto x0 = std::make_shared<SE3PoseVel>("x0", mu, sigma, 0.0);
auto x1 = std::make_shared<SE3PoseVel>("x1", mu, sigma, 1.0);
graph.addVariableNode(x0);
graph.addVariableNode(x1);

// Anchor the first state with a prior at the origin
Eigen::Matrix<double, 6, 6> prior_cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e-3;
auto prior = std::make_shared<SE3PoseVelPrior>("prior_x0", Eigen::Isometry3d::Identity(), prior_cov);
graph.addFactorNode(prior, std::vector<std::string>{"x0"});

// Relative motion measurement: 1 m along x between x0 and x1
Eigen::Isometry3d z = Eigen::Isometry3d::Identity();
z.translation().x() = 1.0;
Eigen::Matrix<double, 6, 6> odom_cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e-2;
auto odom = std::make_shared<SE3PoseVelPoseVel>("odom_x0_x1", z, odom_cov);
graph.addFactorNode(odom, std::vector<std::string>{"x0", "x1"});

// Run Gaussian Belief Propagation for a fixed number of iterations
GbpSolver solver(SolverScheduleType::SYNCHRONOUS);
for (int i = 0; i < 50; ++i)
  solver.performIteration(graph);

// Read the optimized state (translation -> (1, 0, 0))
auto x1_node = std::static_pointer_cast<SE3PoseVel>(graph.getVariableNode("x1"));
Eigen::Isometry3d x1_pose = x1_node->mu_.pose;
```

### Python API

```python
import numpy as np
from gsolver import (
    FactorGraph, GbpSolver, SolverScheduleType,
    SE3PoseVel, SE3PoseVelValue, SE3PoseVelPrior, SE3PoseVelPoseVel,
)

fg = FactorGraph()

# Two pose + velocity states, initialized at the identity
def identity_state():
    s = SE3PoseVelValue()
    s.pose = np.eye(4)
    s.vel = np.zeros(6)
    return s

sigma12 = np.eye(12)
fg.add_variable(SE3PoseVel("x0", identity_state(), sigma12, 0.0))
fg.add_variable(SE3PoseVel("x1", identity_state(), sigma12, 1.0))

# Anchor the first state with a prior at the origin
fg.add_factor(SE3PoseVelPrior("prior_x0", np.eye(4), np.eye(6) * 1e-3), ["x0"])

# Relative motion measurement: 1 m along x between x0 and x1
z = np.eye(4); z[0, 3] = 1.0
fg.add_factor(SE3PoseVelPoseVel("odom_x0_x1", z, np.eye(6) * 1e-2), ["x0", "x1"])

# Run Gaussian Belief Propagation for a fixed number of iterations
solver = GbpSolver(SolverScheduleType.SYNCHRONOUS)
for _ in range(50):
    solver.perform_iteration(fg)

# Read the optimized state (translation -> (1, 0, 0))
optimized_pose = fg.get_variable("x1").mu.pose
```

## 🔬 Visualization

The library integrates with [Rerun](https://rerun.io/) for real-time 3D visualization.

1. Install the Rerun viewer:
   ```bash
   pip install rerun-sdk
   rerun
   ```

2. Run any experiment with `-visualize` flag

3. The viewer will show:
   - Trajectory as 3D points and coordinate frames
   - Factor graph structure
   - Optimization progress (if enabled)

## 📝 File Formats

### Input: g2o Format

Standard g2o format with SE3 poses and edges:
```
VERTEX_SE3:QUAT id x y z qx qy qz qw
EDGE_SE3:QUAT id1 id2 dx dy dz dqx dqy dqz dqw info_11 info_12 ... info_66
```

### Output: TUM Format

Standard TUM trajectory format:
```
timestamp tx ty tz qx qy qz qw
```

Additional datasets are documented in
[`examples/data/datasets/stereo_visual/README.md`](../examples/data/datasets/stereo_visual/README.md).
