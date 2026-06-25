# =============================================================================
# gsolver Python Bindings
# =============================================================================

cmake_minimum_required(VERSION 3.16)

# nanobind is already available from parent CMakeLists.txt via FetchContent
# No find_package needed - it's already populated

# Collect all binding source files
set(BINDING_SOURCES
    # Main module
    gsolver_module.cpp
    
    # Core module
    core/bind_gbp_solver.cpp
    
    # Maths module
    maths/bind_geometry3d.cpp
    
    # Graph module - base classes
    graph/bind_variable_node_base.cpp
    graph/bind_factor_node_base.cpp
    graph/bind_factor_graph.cpp
    graph/bind_edge_base.cpp
    graph/bind_edge.cpp
    
    # Graph module - variable types
    graph/types/variables/bind_se3_point.cpp
    graph/types/variables/bind_se3_pose.cpp
    graph/types/variables/bind_se3_posevel.cpp
    
    # Graph module - factor types
    graph/types/factors/bind_se3_posevel_prior.cpp
    graph/types/factors/bind_se3_posevel_posevel.cpp
    graph/types/factors/bind_se3_posevel_gp.cpp
    graph/types/factors/bind_se3_posevel_pose.cpp
    graph/types/factors/bind_se3_posevel_point.cpp
)

# Create the Python module
nanobind_add_module(_gsolver
    STABLE_ABI
    NB_STATIC
    ${BINDING_SOURCES}
)

# Link against the core library and Eigen
target_link_libraries(_gsolver
    PRIVATE
        gsolver::gsolver
        Eigen3::Eigen
)

# Install the compiled module to the Python package
# scikit-build-core will handle this during pip install
install(TARGETS _gsolver
    LIBRARY DESTINATION .
)

# Install the Python package initializer (re-exports the compiled _gsolver symbols).
# DESTINATION "." resolves to the wheel install-dir ("gsolver"), so this lands as
# gsolver/__init__.py — without it, `from gsolver import ...` fails after pip install.
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
    DESTINATION .
)
