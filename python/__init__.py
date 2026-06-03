"""
gsolver - Gaussian Belief Propagation SLAM Library
====================================================

A C++ library for solving SLAM problems using Gaussian Belief Propagation,
with Python bindings.

Example:
    >>> import gsolver
    >>> 
    >>> # Create factor graph
    >>> fg = gsolver.FactorGraph()
    >>> 
    >>> # Add variables and factors...
    >>> 
    >>> # Solve
    >>> solver = gsolver.GbpSolver(gsolver.SolverScheduleType.SYNCHRONOUS)
    >>> solver.solve(fg)
"""

from gsolver._gsolver import *

__version__ = "0.1.0"
