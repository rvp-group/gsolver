cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(gsolver
    VERSION 0.1.0
    DESCRIPTION "Gaussian Belief Propagation SLAM Solver"
    LANGUAGES CXX
)

# =============================================================================
# Project Options
# =============================================================================
option(GSOLVER_BUILD_EXAMPLES "Build example executables" OFF)
option(GSOLVER_BUILD_PYTHON   "Build Python bindings"     OFF)
option(GSOLVER_BUILD_TESTS    "Build tests"               OFF)

# =============================================================================
# Global Settings
# =============================================================================
# Require out-of-source builds
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not allowed. Please use a separate build directory.")
endif()

# Set default build type if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============================================================================
# Dependencies
# =============================================================================
include(FetchContent)

# --- Eigen3 ---
message(STATUS "Fetching Eigen3...")
FetchContent_Declare(
    Eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH MD5=4c527a9171d71a72a9d4186e65bea559
)
FetchContent_GetProperties(Eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(Eigen)
    add_subdirectory(${eigen_SOURCE_DIR} ${eigen_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# --- Rerun SDK ---
message(STATUS "Fetching Rerun SDK...")
FetchContent_Declare(
    rerun_sdk
    URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)

# --- System Dependencies ---
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)

# =============================================================================
# Core Library
# =============================================================================
add_subdirectory(gsolver)

# =============================================================================
# Optional Components
# =============================================================================

# --- Examples ---
if(GSOLVER_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# --- Python Bindings ---
if(GSOLVER_BUILD_PYTHON)
    # Find Python
    find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
    
    # Fetch nanobind
    message(STATUS "Fetching nanobind...")
    FetchContent_Declare(
        nanobind
        GIT_REPOSITORY https://github.com/wjakob/nanobind.git
        GIT_TAG v2.7.0
    )
    FetchContent_MakeAvailable(nanobind)
    
    add_subdirectory(python)
endif()

# --- Tests ---
if(GSOLVER_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "=== gsolver Configuration Summary ===")
message(STATUS "  Version:         ${PROJECT_VERSION}")
message(STATUS "  Build type:      ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard:    ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build examples:  ${GSOLVER_BUILD_EXAMPLES}")
message(STATUS "  Build Python:    ${GSOLVER_BUILD_PYTHON}")
message(STATUS "  Build tests:     ${GSOLVER_BUILD_TESTS}")
message(STATUS "")