# gsolver

**G-solver: Gaussian Belief Propagation (GBP) with Gaussian Processes (GP) for Distributed and Continuous-Time SLAM**

A C++/Python library implementing Gaussian Belief Propagation (GBP) for Simultaneous Localization and Mapping (SLAM) with continuous-time Gaussian Process (GP) motion priors.

## 📄 Paper

> **Breaking Time: A Fully Gaussian Framework for Distributed and Continuous-Time SLAM**
>
> *Accepted to IEEE Robotics and Automation Letters (RA-L) on May 24, 2026.*

Repository: https://github.com/rvp-group/gsolver (SSH: git@github.com:rvp-group/gsolver.git)

If you use this code in your research, please cite our paper.

## 🎯 Features

- **Gaussian Belief Propagation Solver**: Efficient message-passing inference on factor graphs
- **Continuous-Time GP Motion Priors**: Smooth trajectory estimation with learnable hyperparameters
- **SE(3) Pose Optimization**: Full 6-DoF pose estimation with velocity states
- **Bundle Adjustment**: Joint optimization of poses and landmarks
- **Incremental SLAM**: Online operation with new measurements
- **Real-time Visualization**: Integration with [Rerun](https://rerun.io/) for 3D visualization
- **Python Bindings**: Full Python API via nanobind

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

### Prerequisites

- **C++17** compatible compiler (GCC 9+, Clang 10+)
- **CMake** 3.16+
- **OpenMP** (for parallel GBP)
- **yaml-cpp** (for configuration files)
- **Python** 3.8+ (for Python bindings)

On Ubuntu/Debian:
```bash
sudo apt-get install build-essential cmake libomp-dev libyaml-cpp-dev python3-dev
```

### C++ Library

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

### Python Package

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
- `datasets/gt/` - Ground truth trajectories (TUM format)
- `datasets/pgo/` - Pose Graph Optimization datasets (g2o format)
- `datasets/prior/` - Prior factor experiment datasets

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

To compute optimal `qc_diag` values for a trajectory:

```bash
./build/examples/cpp/gp_hyperparam_trainer examples/data/datasets/gt/helix_1hz.g2o
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

### C++ Experiments

**Pose Graph Optimization:**
```bash
./build/examples/cpp/pgo_experiment \
    examples/data/datasets/pgo/trajectories_1hz/noise_ig1.0_n0.001_helix.g2o \
    ./tmp/pgo_result.tum \
    helix \
    -visualize
```

**Prior Experiment:**
```bash
./build/examples/cpp/prior_experiment \
    examples/data/datasets/prior/trajectories_1hz/noise_ig1.0_n0.001_helix.g2o \
    ./tmp/prior_result.tum \
    helix \
    -visualize
```

**Bundle Adjustment (Charuco):**
```bash
./build/examples/cpp/charuco_experiment_ba \
    examples/data/datasets/charuco/printing_room_0.g2o \
    ./tmp/ba_result.tum \
    printing_room \
    -visualize
```

**Incremental SLAM (Charuco):**
```bash
./build/examples/cpp/charuco_experiment_islam \
    examples/data/datasets/charuco/printing_room_0.g2o \
    ./tmp/islam_result.tum \
    printing_room \
    -visualize
```

**Hyperparameter Ablation:**
```bash
./build/examples/cpp/hyperparam_ablation_experiment \
    examples/data/datasets/pgo/trajectories_1hz/noise_ig1.0_n0.001_helix.g2o \
    ./tmp/ablation_results/ \
    helix \
    -visualize
```

### Python Experiments

**Pose Graph Optimization:**
```bash
python examples/python/apps/pgo_experiment.py \
    examples/data/datasets/pgo/trajectories_1hz/noise_ig1.0_n0.001_helix.g2o \
    ./tmp/pgo_result.tum \
    helix \
    -visualize
```

**Prior Experiment:**
```bash
python examples/python/apps/prior_experiment.py \
    examples/data/datasets/prior/trajectories_1hz/noise_ig1.0_n0.001_helix.g2o \
    ./tmp/prior_result.tum \
    helix \
    -visualize
```

**Bundle Adjustment (Charuco):**
```bash
python examples/python/apps/charuco_experiment_ba.py \
    examples/data/datasets/charuco/printing_room_0.g2o \
    ./tmp/ba_result.tum \
    printing_room \
    -visualize
```

**Incremental SLAM (Charuco):**
```bash
python examples/python/apps/charuco_experiment_islam.py \
    examples/data/datasets/charuco/printing_room_0.g2o \
    ./tmp/islam_result.tum \
    printing_room \
    -visualize
```

### Command Line Arguments

All experiments follow the same pattern:

```bash
<experiment> <input.g2o> <output.tum> <trajectory-type> [-visualize]
```

| Argument | Description |
|----------|-------------|
| `input.g2o` | Input factor graph in g2o format |
| `output.tum` | Output trajectory in TUM format |
| `trajectory-type` | Config key: `helix`, `sphere`, `printing_room`, etc. |
| `-visualize` | Optional: Enable Rerun 3D visualization |

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

## 📖 API Quick Start

### C++ API

```cpp
#include <gsolver/core/gbp_solver.h>
#include <gsolver/graph/factor_graph.h>
#include <gsolver/graph/types/variables/se3_posevel.h>
#include <gsolver/graph/types/factors/odometry_se3.h>

using namespace gsolver;

// Create factor graph
auto graph = std::make_shared<FactorGraph>();

// Add pose variables
auto pose0 = std::make_shared<SE3PoseVel>("pose_0", timestamp0, initial_mu0, initial_omega0);
auto pose1 = std::make_shared<SE3PoseVel>("pose_1", timestamp1, initial_mu1, initial_omega1);
graph->addVariableNode(pose0);
graph->addVariableNode(pose1);

// Add odometry factor
auto odom = std::make_shared<OdometrySE3>("odom_0_1", measurement, information);
odom->setVariables({pose0, pose1});
graph->addFactorNode(odom);

// Solve with GBP
GbpSolver solver(graph);
solver.setScheduleType(SolverScheduleType::PARALLEL);
solver.setNumIterations(100);
solver.solve();

// Get optimized poses
Eigen::Matrix<double, 12, 1> optimized_pose = pose0->mu();
```

### Python API

```python
from gsolver import FactorGraph, GbpSolver, SolverScheduleType
from gsolver import SE3PoseVel, OdometrySE3

# Create factor graph
graph = FactorGraph()

# Add pose variables
pose0 = SE3PoseVel("pose_0", timestamp0, initial_mu0, initial_omega0)
pose1 = SE3PoseVel("pose_1", timestamp1, initial_mu1, initial_omega1)
graph.add_variable_node(pose0)
graph.add_variable_node(pose1)

# Add odometry factor
odom = OdometrySE3("odom_0_1", measurement, information)
odom.set_variables([pose0, pose1])
graph.add_factor_node(odom)

# Solve with GBP
solver = GbpSolver(graph)
solver.schedule_type = SolverScheduleType.PARALLEL
solver.num_iterations = 100
solver.solve()

# Get optimized poses
optimized_pose = pose0.mu
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

## 📜 License

BSD 3-Clause License

Copyright (c) 2026, Robots Vision and Perception Group

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## 🙏 Acknowledgements

- [Eigen](https://eigen.tuxfamily.org/) - Linear algebra library
- [nanobind](https://github.com/wjakob/nanobind) - Python bindings
- [Rerun](https://rerun.io/) - Visualization
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) - Configuration parsing

