#!/usr/bin/env python3
"""
Generate YAML configuration files for synthetic dataset generation.

This script creates YAML configuration files with various noise levels
for benchmarking pose graph optimization algorithms. It supports:
- Ground truth trajectories (no noise)
- Prior-based experiments (absolute pose measurements)
- PGO experiments (relative odometry measurements)

Usage:
    python3 generate_configs.py <output_folder> <hz> <-gt|-prior|-pgo>

Arguments:
    output_folder: Directory to save configuration files
    hz: Sampling frequency (e.g., 1, 100)
    -gt: Generate ground truth config (no noise)
    -prior: Generate prior experiment configs (varying noise)
    -pgo: Generate PGO experiment configs (varying noise)

Example:
    python3 generate_configs.py gt_configs_100hz 100 -gt
    python3 generate_configs.py prior/noisy_configs_1hz 1 -prior
"""

import os
import argparse
import yaml


# =============================================================================
# Trajectory Parameters (shared across all configs)
# =============================================================================

TRAJECTORY_PARAMS = {
    "helix": {
        "path_length": 200,
        "radius": 10,
        "height": 10,
        "num_circles": 3,
        "circle_steps": 200
    },
    "sphere": {
        "num_points": 600,
        "radius": 10
    }
}

# Noise levels for benchmarking (translation, rotation pairs)
# Covers range from no noise to significant noise
NOISE_LEVELS = [
    (0.0, 0.0),        # No noise
    (1e-4, 1e-5),      # Very low
    (1e-3, 1e-4),      # Low
    (1e-2, 1e-3),      # Medium-low
    (1e-1, 1e-2),      # Medium
    (1.0, 1e-1),       # High
    (1.5, 0.15)        # Very high
]


# =============================================================================
# Configuration Generators
# =============================================================================

def create_base_config(hz: int, random_seed: int = 42) -> dict:
    """Create base configuration with trajectory parameters.
    
    Args:
        hz: Sampling frequency in Hz
        random_seed: Random seed for reproducibility
        
    Returns:
        Base configuration dictionary
    """
    return {
        "gt": False,
        "random_seed": random_seed,
        "trajectory": {
            "hz": hz,
            **TRAJECTORY_PARAMS
        },
        "variable_node": {
            "std_dev_data": {
                "add_noise": False,
                "translation": 0.0,
                "rotation": 0.0
            }
        },
        "factor_node": {
            "prior": {
                "add_factors": False,
                "std_dev_data": {
                    "add_noise": False,
                    "translation": 0.0,
                    "rotation": 0.0
                }
            },
            "pose_pose": {
                "add_factors": False,
                "percentage_of_interconnections": 50.0,
                "std_dev_data": {
                    "add_noise": False,
                    "translation": 0.0,
                    "rotation": 0.0
                }
            }
        }
    }


def generate_gt_config(output_folder: str, hz: int, random_seed: int = 42) -> None:
    """Generate ground truth configuration (no noise).
    
    Args:
        output_folder: Output directory
        hz: Sampling frequency
        random_seed: Random seed
    """
    os.makedirs(output_folder, exist_ok=True)
    
    config = create_base_config(hz, random_seed)
    config["gt"] = True
    
    filename = f"gt_{hz}hz.yaml"
    filepath = os.path.join(output_folder, filename)
    
    with open(filepath, "w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    
    print(f"Generated: {filepath}")


def generate_prior_configs(output_folder: str, hz: int, random_seed: int = 42) -> None:
    """Generate prior experiment configurations with varying noise levels.
    
    Creates a grid of configurations varying:
    - Initial guess noise (variable node)
    - Prior measurement noise (factor node)
    
    Args:
        output_folder: Output directory
        hz: Sampling frequency
        random_seed: Random seed
    """
    os.makedirs(output_folder, exist_ok=True)
    
    for ig_trans, ig_rot in NOISE_LEVELS:
        for meas_trans, meas_rot in NOISE_LEVELS:
            config = create_base_config(hz, random_seed)
            
            # Initial guess noise
            config["variable_node"]["std_dev_data"]["add_noise"] = True
            config["variable_node"]["std_dev_data"]["translation"] = ig_trans
            config["variable_node"]["std_dev_data"]["rotation"] = ig_rot
            
            # Prior factor noise
            config["factor_node"]["prior"]["add_factors"] = True
            config["factor_node"]["prior"]["std_dev_data"]["add_noise"] = True
            config["factor_node"]["prior"]["std_dev_data"]["translation"] = meas_trans
            config["factor_node"]["prior"]["std_dev_data"]["rotation"] = meas_rot
            
            filename = f"noise_ig{ig_trans}_n{meas_trans}.yaml"
            filepath = os.path.join(output_folder, filename)
            
            with open(filepath, "w") as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)
            
            print(f"Generated: {filepath}")


def generate_pgo_configs(output_folder: str, hz: int, random_seed: int = 42) -> None:
    """Generate PGO experiment configurations with varying noise levels.
    
    Creates a grid of configurations varying:
    - Initial guess noise (variable node)
    - Odometry measurement noise (pose-pose factor)
    
    Args:
        output_folder: Output directory
        hz: Sampling frequency
        random_seed: Random seed
    """
    os.makedirs(output_folder, exist_ok=True)
    
    for ig_trans, ig_rot in NOISE_LEVELS:
        for meas_trans, meas_rot in NOISE_LEVELS:
            config = create_base_config(hz, random_seed)
            
            # Initial guess noise
            config["variable_node"]["std_dev_data"]["add_noise"] = True
            config["variable_node"]["std_dev_data"]["translation"] = ig_trans
            config["variable_node"]["std_dev_data"]["rotation"] = ig_rot
            
            # Pose-pose factor noise
            config["factor_node"]["pose_pose"]["add_factors"] = True
            config["factor_node"]["pose_pose"]["std_dev_data"]["add_noise"] = True
            config["factor_node"]["pose_pose"]["std_dev_data"]["translation"] = meas_trans
            config["factor_node"]["pose_pose"]["std_dev_data"]["rotation"] = meas_rot
            
            filename = f"noise_ig{ig_trans}_n{meas_trans}.yaml"
            filepath = os.path.join(output_folder, filename)
            
            with open(filepath, "w") as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)
            
            print(f"Generated: {filepath}")


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Generate YAML configuration files for synthetic datasets",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    %(prog)s gt_configs_100hz 100 -gt
    %(prog)s prior/noisy_configs_1hz 1 -prior
    %(prog)s pgo/noisy_configs_1hz 1 -pgo
        """
    )
    parser.add_argument("output_folder", help="Output directory for config files")
    parser.add_argument("hz", type=int, help="Sampling frequency in Hz")
    
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument("-gt", action="store_true", help="Generate ground truth config")
    mode_group.add_argument("-prior", action="store_true", help="Generate prior experiment configs")
    mode_group.add_argument("-pgo", action="store_true", help="Generate PGO experiment configs")
    
    parser.add_argument("--seed", type=int, default=42, help="Random seed (default: 42)")
    
    args = parser.parse_args()
    
    if args.gt:
        generate_gt_config(args.output_folder, args.hz, args.seed)
    elif args.prior:
        generate_prior_configs(args.output_folder, args.hz, args.seed)
    elif args.pgo:
        generate_pgo_configs(args.output_folder, args.hz, args.seed)


if __name__ == "__main__":
    main()
