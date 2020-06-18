cmake_minimum_required(VERSION 3.5.0)
project(tesseract_motion_planners VERSION 0.1.0 LANGUAGES CXX)

find_package(ompl REQUIRED)
find_package(trajopt REQUIRED)
find_package(trajopt_sco REQUIRED)
find_package(descartes_light REQUIRED)
find_package(descartes_samplers REQUIRED)
find_package(descartes_opw REQUIRED)
find_package(tesseract REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED)

set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/* /*/include/descartes_light/* /*/include/opw_kinematics/* /*/include/trajopt/* /*/include/trajopt_sco/*)

add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})

# Create interface for core
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE tesseract::tesseract tesseract::tesseract_common tesseract::tesseract_command_language trajopt::trajopt console_bridge)
tesseract_target_compile_options(${PROJECT_NAME}_core INTERFACE)
tesseract_clang_tidy(${PROJECT_NAME}_core)
tesseract_code_coverage(${PROJECT_NAME}_core ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(${PROJECT_NAME}_core INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_core SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS})

# Trajopt Planner
add_library(${PROJECT_NAME}_trajopt
  src/trajopt/trajopt_planner_config.cpp
  src/trajopt/trajopt_motion_planner.cpp
  src/trajopt/trajopt_planner_universal_config.cpp
  src/trajopt/trajopt_utils.cpp
  src/trajopt/profile/trajopt_default_plan_profile.cpp
  src/trajopt/profile/trajopt_default_composite_profile.cpp)
target_link_libraries(${PROJECT_NAME}_trajopt PUBLIC ${Boost_LIBRARIES} ${PROJECT_NAME}_core trajopt::trajopt trajopt::trajopt_utils trajopt::trajopt_sco)
tesseract_target_compile_options(${PROJECT_NAME}_trajopt PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_trajopt)
tesseract_code_coverage(${PROJECT_NAME}_trajopt ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(${PROJECT_NAME}_trajopt PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_trajopt SYSTEM PUBLIC
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS})

## OMPL Freespace Planning Interface
#set(OMPL_SRC src/ompl/ompl_motion_planner.cpp
#  src/ompl/continuous_motion_validator.cpp
#  src/ompl/discrete_motion_validator.cpp
#  src/ompl/weighted_real_vector_state_sampler.cpp
#  src/ompl/ompl_motion_planner_status_category.cpp
#  src/ompl/ompl_planner_configurator.cpp
#  src/ompl/config/ompl_planner_config.cpp
#  src/ompl/config/ompl_planner_freespace_config.cpp
#  src/ompl/utils.cpp
#  src/ompl/state_collision_validator.cpp
#  src/ompl/compound_state_validator.cpp)

#if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
#  list(APPEND OMPL_SRC src/ompl/config/ompl_planner_constrained_config.cpp)
#endif()

#add_library(${PROJECT_NAME}_ompl SHARED ${OMPL_SRC})
#target_link_libraries(${PROJECT_NAME}_ompl PUBLIC ${PROJECT_NAME}_core ${OMPL_LIBRARIES})
#tesseract_target_compile_options(${PROJECT_NAME}_ompl PUBLIC)
#tesseract_clang_tidy(${PROJECT_NAME}_ompl)
#tesseract_code_coverage(${PROJECT_NAME}_ompl ALL EXCLUDE ${COVERAGE_EXCLUDE})
#target_include_directories(${PROJECT_NAME}_ompl PUBLIC
#    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#    "$<INSTALL_INTERFACE:include>")
#target_include_directories(${PROJECT_NAME}_ompl SYSTEM PUBLIC
#    ${OMPL_INCLUDE_DIRS})

# Descartes Planner
add_library(${PROJECT_NAME}_descartes SHARED
  src/descartes/descartes_motion_planner.cpp
  src/descartes/descartes_motion_planner_default_config.cpp
  src/descartes/descartes_collision.cpp
  src/descartes/descartes_collision_edge_evaluator.cpp
  src/descartes/descartes_robot_sampler.cpp
  src/descartes/descartes_robot_positioner_sampler.cpp
  src/descartes/descartes_external_positioner_sampler.cpp
  src/descartes/descartes_motion_planner_status_category.cpp
  src/descartes/profile/descartes_default_plan_profile.cpp)
target_link_libraries(${PROJECT_NAME}_descartes PUBLIC ${PROJECT_NAME}_core descartes::descartes_light descartes::descartes_samplers descartes::descartes_opw)
tesseract_target_compile_options(${PROJECT_NAME}_descartes PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_descartes)
tesseract_code_coverage(${PROJECT_NAME}_descartes ALL EXCLUDE ${COVERAGE_EXCLUDE})
target_include_directories(${PROJECT_NAME}_descartes PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

tesseract_configure_package(${PROJECT_NAME}_core
                            ${PROJECT_NAME}_trajopt
                            #${PROJECT_NAME}_ompl
                            ${PROJECT_NAME}_descartes
                            )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

#if (TESSERACT_ENABLE_TESTING)
#  enable_testing()
#  tesseract_add_run_tests_target()
  add_subdirectory(test)
#endif()
