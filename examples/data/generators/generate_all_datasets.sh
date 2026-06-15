#!/bin/bash
# =============================================================================
# Generate All Datasets
# =============================================================================
#
# This script generates all synthetic datasets for gsolver experiments.
# It creates ground truth trajectories and noisy factor graphs for benchmarking.
#
# Output structure:
#   datasets/
#   └── synthetic/
#       ├── gt/                    # Ground truth trajectories
#       │   ├── helix_1hz.tum
#       │   ├── helix_100hz.tum
#       │   ├── sphere_1hz.tum
#       │   └── sphere_100hz.tum
#       ├── prior/                 # Prior factor experiments
#       │   ├── configs_1hz/       # Configuration files
#       │   └── trajectories_1hz/  # Generated .g2o files
#       └── pgo/                   # Pose graph optimization experiments
#           ├── configs_1hz/
#           └── trajectories_1hz/
#
# Usage:
#   ./generate_all_datasets.sh [--quick]
#
# Options:
#   --quick  Generate only a subset of noise levels for quick testing
#
# Requirements:
#   - data_generator executable (build with CMake first)
#   - Python 3 with PyYAML
# =============================================================================

set -e  # Exit on error

# =============================================================================
# Configuration
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/../../../build"
EXECUTABLE="${BUILD_DIR}/examples/cpp/data_generator"
OUTPUT_DIR="${SCRIPT_DIR}/../datasets/synthetic"

# Trajectory types (no torus)
SHAPES=("helix" "sphere")

# Frequencies to generate
FREQUENCIES=(1 100)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# =============================================================================
# Helper Functions
# =============================================================================

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_section() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE} $1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

# =============================================================================
# Pre-flight Checks
# =============================================================================

check_executable() {
    if [[ ! -x "$EXECUTABLE" ]]; then
        log_error "data_generator executable not found at: $EXECUTABLE"
        log_info "Please build the project first:"
        log_info "  cd ${BUILD_DIR} && make data_generator"
        exit 1
    fi
    log_info "Using executable: $EXECUTABLE"
}

check_python() {
    if ! command -v python3 &> /dev/null; then
        log_error "python3 not found"
        exit 1
    fi
    
    if ! python3 -c "import yaml" &> /dev/null; then
        log_error "PyYAML not installed. Run: pip3 install pyyaml"
        exit 1
    fi
}

# =============================================================================
# Dataset Generation Functions
# =============================================================================

generate_ground_truth() {
    local hz=$1
    log_section "Generating Ground Truth Trajectories (${hz}Hz)"
    
    local config_dir="${OUTPUT_DIR}/gt/configs_${hz}hz"
    local output_dir="${OUTPUT_DIR}/gt"
    
    # Generate config
    python3 "${SCRIPT_DIR}/src/generate_configs.py" "$config_dir" "$hz" -gt
    
    # Create output directory
    mkdir -p "$output_dir"
    
    # Generate trajectories for each shape
    local config_file="${config_dir}/gt_${hz}hz.yaml"
    for shape in "${SHAPES[@]}"; do
        local output_file="${output_dir}/${shape}_${hz}hz"
        log_info "Generating: ${shape}_${hz}hz.tum"
        "$EXECUTABLE" "$config_file" "$shape" "$output_file"
    done
}

generate_prior_datasets() {
    local hz=$1
    log_section "Generating Prior Factor Datasets (${hz}Hz)"
    
    local config_dir="${OUTPUT_DIR}/prior/configs_${hz}hz"
    local output_dir="${OUTPUT_DIR}/prior/trajectories_${hz}hz"
    
    # Generate configs
    python3 "${SCRIPT_DIR}/src/generate_configs.py" "$config_dir" "$hz" -prior
    
    # Create output directory
    mkdir -p "$output_dir"
    
    # Generate datasets for each config and shape
    local count=0
    for config_file in "$config_dir"/*.yaml; do
        local base_name=$(basename "$config_file" .yaml)
        
        for shape in "${SHAPES[@]}"; do
            local output_file="${output_dir}/${base_name}_${shape}"
            "$EXECUTABLE" "$config_file" "$shape" "$output_file"
            count=$((count + 1))
        done
    done
    
    log_info "Generated $count prior datasets"
}

generate_pgo_datasets() {
    local hz=$1
    log_section "Generating PGO Datasets (${hz}Hz)"
    
    local config_dir="${OUTPUT_DIR}/pgo/configs_${hz}hz"
    local output_dir="${OUTPUT_DIR}/pgo/trajectories_${hz}hz"
    
    # Generate configs
    python3 "${SCRIPT_DIR}/src/generate_configs.py" "$config_dir" "$hz" -pgo
    
    # Create output directory
    mkdir -p "$output_dir"
    
    # Generate datasets for each config and shape
    local count=0
    for config_file in "$config_dir"/*.yaml; do
        local base_name=$(basename "$config_file" .yaml)
        
        for shape in "${SHAPES[@]}"; do
            local output_file="${output_dir}/${base_name}_${shape}"
            "$EXECUTABLE" "$config_file" "$shape" "$output_file"
            count=$((count + 1))
        done
    done
    
    log_info "Generated $count PGO datasets"
}

# =============================================================================
# Main
# =============================================================================

main() {
    log_section "gsolver Dataset Generator"
    
    # Parse arguments
    local quick_mode=false
    if [[ "$1" == "--quick" ]]; then
        quick_mode=true
        log_warn "Quick mode: generating reduced dataset"
    fi
    
    # Pre-flight checks
    check_executable
    check_python
    
    # Create output directory
    mkdir -p "$OUTPUT_DIR"
    log_info "Output directory: $OUTPUT_DIR"
    
    # Generate ground truth at both 1Hz and 100Hz
    for hz in "${FREQUENCIES[@]}"; do
        generate_ground_truth "$hz"
    done
    
    # Generate prior and PGO datasets only at 1Hz
    generate_prior_datasets 1
    generate_pgo_datasets 1
    
    log_section "Dataset Generation Complete"
    log_info "Datasets saved to: $OUTPUT_DIR"
    
    # Print summary
    echo ""
    log_info "Summary:"
    echo "  Ground truth: ${#SHAPES[@]} shapes × ${#FREQUENCIES[@]} frequencies (1Hz and 100Hz)"
    echo "  Prior datasets: 49 noise levels × ${#SHAPES[@]} shapes × 1Hz only"
    echo "  PGO datasets: 49 noise levels × ${#SHAPES[@]} shapes × 1Hz only"
}

main "$@"
