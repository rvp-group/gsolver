# =============================================================================
# gsolver C++ Examples
# =============================================================================

# Common include directories for all experiments
set(EXAMPLES_COMMON_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR})

# -----------------------------------------------------------------------------
# Data Generator (for synthetic datasets)
# -----------------------------------------------------------------------------
add_executable(data_generator ../data/generators/src/data_generator.cpp)
target_link_libraries(data_generator PRIVATE gsolver::gsolver yaml-cpp)
target_include_directories(data_generator PRIVATE ${EXAMPLES_COMMON_INCLUDES})

# -----------------------------------------------------------------------------
# PGO Experiment
# -----------------------------------------------------------------------------
add_executable(pgo_experiment apps/pgo_experiment.cpp)
target_link_libraries(pgo_experiment PRIVATE gsolver::gsolver yaml-cpp rerun_sdk)
target_include_directories(pgo_experiment PRIVATE ${EXAMPLES_COMMON_INCLUDES})

# -----------------------------------------------------------------------------
# Prior Experiment
# -----------------------------------------------------------------------------
add_executable(prior_experiment apps/prior_experiment.cpp)
target_link_libraries(prior_experiment PRIVATE gsolver::gsolver yaml-cpp rerun_sdk)
target_include_directories(prior_experiment PRIVATE ${EXAMPLES_COMMON_INCLUDES})

# -----------------------------------------------------------------------------
# Hyperparam Ablation Experiment
# -----------------------------------------------------------------------------
add_executable(hyperparam_ablation_experiment apps/hyperparam_ablation_experiment.cpp)
target_link_libraries(hyperparam_ablation_experiment PRIVATE gsolver::gsolver yaml-cpp rerun_sdk)
target_include_directories(hyperparam_ablation_experiment PRIVATE ${EXAMPLES_COMMON_INCLUDES})

# -----------------------------------------------------------------------------
# Charuco Experiments
# -----------------------------------------------------------------------------
add_executable(charuco_experiment_islam apps/charuco_experiment_islam.cpp)
target_link_libraries(charuco_experiment_islam PRIVATE gsolver::gsolver yaml-cpp rerun_sdk)
target_include_directories(charuco_experiment_islam PRIVATE ${EXAMPLES_COMMON_INCLUDES})

add_executable(charuco_experiment_ba apps/charuco_experiment_ba.cpp)
target_link_libraries(charuco_experiment_ba PRIVATE gsolver::gsolver yaml-cpp rerun_sdk)
target_include_directories(charuco_experiment_ba PRIVATE ${EXAMPLES_COMMON_INCLUDES})

# -----------------------------------------------------------------------------
# GP Hyperparameter Trainer
# -----------------------------------------------------------------------------
add_executable(gp_hyperparam_trainer apps/gp_hyperparam_trainer.cpp)
target_link_libraries(gp_hyperparam_trainer PRIVATE gsolver::gsolver yaml-cpp)
target_include_directories(gp_hyperparam_trainer PRIVATE ${EXAMPLES_COMMON_INCLUDES})
