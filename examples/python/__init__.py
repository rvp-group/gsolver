"""
gsolver Python Examples

This package contains example applications demonstrating the gsolver library.

Structure:
    apps/       - Standalone experiment scripts (pgo, prior, charuco, etc.)
    common/     - Shared utilities (I/O, data types, visualization)
    config/     - Configuration files and loaders

Usage:
    # Run experiments from the apps/ folder
    cd examples/python
    python -m apps.pgo_experiment <args>
    
    # Or import utilities in your own scripts
    from examples.python.common import parse_g2o_file, write_tum, init_rerun
    from examples.python.config import get_qc_diag, get_num_iterations
"""
