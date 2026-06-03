"""
Configuration loading utilities for experiment parameters.

Loads parameters from the shared config at examples/config/experiment_params.yaml
(same config used by C++ experiments).
"""

from pathlib import Path
from typing import Dict, Any, List, Optional, Union
import numpy as np
import yaml


# Path to the shared config directory (at examples/ level, shared with C++)
CONFIG_DIR = Path(__file__).parent.parent.parent / "config"
EXPERIMENT_PARAMS_FILE = CONFIG_DIR / "experiment_params.yaml"


def load_experiment_params(trajectory_type: Optional[str] = None, config_file: Optional[Path] = None) -> Dict[str, Any]:
    """
    Load experiment parameters from YAML config file.

    Args:
        trajectory_type: Name of the trajectory (e.g., 'sphere', 'helix', 'printing_room').
                        If None, returns the full config dict.
        config_file: Path to config file. Defaults to experiment_params.yaml.

    Returns:
        If trajectory_type is specified: Dict with 'qc_diag', 'num_iterations', 'description'.
        If trajectory_type is None: Full config dict with all trajectory types.
    """
    if config_file is None:
        config_file = EXPERIMENT_PARAMS_FILE

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    if trajectory_type is None:
        return config

    # Get params for specific trajectory type
    if trajectory_type in config:
        params = config[trajectory_type]
    else:
        print(f"Warning: Unknown trajectory type '{trajectory_type}', using default parameters")
        params = config.get('default', {})

    # Use default values for missing keys
    default = config.get('default', {})

    return {
        'qc_diag': np.array(params.get('qc_diag', default.get('qc_diag', [1.0]*6))),
        'num_iterations': params.get('num_iterations', default.get('num_iterations', 100)),
        'description': params.get('description', ''),
    }


def get_qc_diag(trajectory_type: str, config: Optional[Dict[str, Any]] = None) -> List[float]:
    """
    Get Qc diagonal parameters for a given trajectory type.
    
    Args:
        trajectory_type: Name of the trajectory (e.g., 'sphere', 'helix', 'printing_room')
        config: Pre-loaded config dict. If None, loads from default file.
        
    Returns:
        List of 6 floats representing Qc diagonal [tx, ty, tz, rx, ry, rz].
    """
    if config is None:
        config = load_experiment_params()
    
    if trajectory_type in config:
        return config[trajectory_type].get('qc_diag', config['default']['qc_diag'])
    else:
        print(f"Warning: Unknown trajectory type '{trajectory_type}', using default parameters")
        return config['default']['qc_diag']


def get_num_iterations(trajectory_type: str, config: Optional[Dict[str, Any]] = None) -> int:
    """
    Get number of GBP iterations for a given trajectory type.
    
    Args:
        trajectory_type: Name of the trajectory (e.g., 'sphere', 'helix', 'printing_room')
        config: Pre-loaded config dict. If None, loads from default file.
        
    Returns:
        Number of iterations.
    """
    if config is None:
        config = load_experiment_params()
    
    if trajectory_type in config:
        return config[trajectory_type].get('num_iterations', config['default']['num_iterations'])
    else:
        return config['default']['num_iterations']
