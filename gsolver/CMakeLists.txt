# =============================================================================
# gsolver Library
# =============================================================================

# -----------------------------------------------------------------------------
# Source Files
# -----------------------------------------------------------------------------
set(GSOLVER_SOURCES
    # Core
    src/core/gbp_solver.cpp
    
    # Graph - non-template implementations only
    # Note: variable_node.cpp, factor_node.cpp, factor_graph.cpp contain template
    # definitions and are #included by instantiations.cpp
    
    # Variable types
    src/graph/types/variables/se3_point.cpp
    src/graph/types/variables/se3_pose.cpp
    src/graph/types/variables/se3_posevel.cpp
    
    # Factor types
    src/graph/types/factors/se3_point_equality.cpp
    src/graph/types/factors/se3_posevel_gp.cpp
    src/graph/types/factors/se3_posevel_point.cpp
    src/graph/types/factors/se3_posevel_stereo_point.cpp
    src/graph/types/factors/se3_posevel_pose.cpp
    src/graph/types/factors/se3_posevel_posevel.cpp
    src/graph/types/factors/se3_posevel_prior.cpp
    
    # Explicit template instantiations (must be last)
    src/graph/types/instantiations.cpp
)

# -----------------------------------------------------------------------------
# Library Target
# -----------------------------------------------------------------------------
add_library(gsolver STATIC ${GSOLVER_SOURCES})

# Create namespaced alias for use with target_link_libraries
add_library(gsolver::gsolver ALIAS gsolver)

# -----------------------------------------------------------------------------
# Include Directories
# -----------------------------------------------------------------------------
target_include_directories(gsolver
    PUBLIC
        # Build interface: use source directory
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        # Install interface: use installed include directory
        $<INSTALL_INTERFACE:include>
    PRIVATE
        # For including template .cpp files in instantiations.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# -----------------------------------------------------------------------------
# Link Dependencies
# -----------------------------------------------------------------------------
target_link_libraries(gsolver
    PUBLIC
        Eigen3::Eigen
        OpenMP::OpenMP_CXX
    PRIVATE
        yaml-cpp
        rerun_sdk
)

# -----------------------------------------------------------------------------
# Compile Features & Options
# -----------------------------------------------------------------------------
target_compile_features(gsolver PUBLIC cxx_std_17)

# Enable Position Independent Code for shared library linking (e.g., Python bindings)
set_target_properties(gsolver PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_options(gsolver
    PRIVATE
        $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Wpedantic>
        $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra -Wpedantic>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# -----------------------------------------------------------------------------
# Install Rules
# -----------------------------------------------------------------------------
include(GNUInstallDirs)

# Install the library
install(TARGETS gsolver
    EXPORT gsolverTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install headers (only .h and .hpp files, not .cpp)
install(DIRECTORY include/gsolver
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING 
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN "*.cpp" EXCLUDE
)

# Export targets for find_package support
install(EXPORT gsolverTargets
    FILE gsolverTargets.cmake
    NAMESPACE gsolver::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gsolver
)

# Generate package config files
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/gsolverConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/gsolverConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gsolver
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/gsolverConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/gsolverConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/gsolverConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gsolver
)