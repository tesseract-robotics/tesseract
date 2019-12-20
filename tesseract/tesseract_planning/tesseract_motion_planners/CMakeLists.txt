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
find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED)

# Create interface for core
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE tesseract::tesseract tesseract::tesseract_common console_bridge)
tesseract_target_compile_options(${PROJECT_NAME}_core INTERFACE)
tesseract_clang_tidy(${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_core INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_core SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS})

# Trajopt Planner
add_library(${PROJECT_NAME}_trajopt
  SHARED
    src/trajopt/config/utils.cpp
    src/trajopt/config/trajopt_planner_config.cpp
    src/trajopt/config/trajopt_planner_default_config.cpp
    src/trajopt/config/trajopt_planner_freespace_config.cpp
    src/trajopt/trajopt_motion_planner.cpp
)
target_link_libraries(${PROJECT_NAME}_trajopt PUBLIC ${Boost_LIBRARIES} ${PROJECT_NAME}_core trajopt::trajopt trajopt::trajopt_utils trajopt::trajopt_sco)
tesseract_target_compile_options(${PROJECT_NAME}_trajopt PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_trajopt)
target_include_directories(${PROJECT_NAME}_trajopt PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_trajopt SYSTEM PUBLIC
    ${Boost_INCLUDE_DIRS})

# OMPL Freespace Planning Interface
add_library(${PROJECT_NAME}_ompl_freespace SHARED
  src/ompl/ompl_freespace_planner.cpp
  src/ompl/conversions.cpp
  src/ompl/continuous_motion_validator.cpp
  src/ompl/discrete_motion_validator.cpp
  src/ompl/weighted_real_vector_state_sampler.cpp
  src/ompl/ompl_freespace_planner_status_category.cpp
)
target_link_libraries(${PROJECT_NAME}_ompl_freespace PUBLIC ${PROJECT_NAME}_core ${OMPL_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_ompl_freespace PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_ompl_freespace)
target_include_directories(${PROJECT_NAME}_ompl_freespace PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_ompl_freespace SYSTEM PUBLIC
    ${OMPL_INCLUDE_DIRS})

# OMPL Hybrid Planner
add_library(${PROJECT_NAME}_ompl_trajopt_freespace SHARED src/hybrid/ompl_trajopt_freespace_planner.cpp)
target_link_libraries(${PROJECT_NAME}_ompl_trajopt_freespace PUBLIC ${PROJECT_NAME}_ompl_freespace ${PROJECT_NAME}_trajopt)
tesseract_target_compile_options(${PROJECT_NAME}_ompl_trajopt_freespace PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_ompl_trajopt_freespace)
target_include_directories(${PROJECT_NAME}_ompl_trajopt_freespace PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

# Descartes Planner
add_library(${PROJECT_NAME}_descartes SHARED
  src/descartes/descartes_motion_planner.cpp
  src/descartes/descartes_collision.cpp
  src/descartes/descartes_collision_edge_evaluator.cpp
  src/descartes/descartes_robot_sampler.cpp
  src/descartes/descartes_robot_positioner_sampler.cpp
  src/descartes/descartes_external_positioner_sampler.cpp
  src/descartes/descartes_motion_planner_status_category.cpp)
target_link_libraries(${PROJECT_NAME}_descartes PUBLIC ${PROJECT_NAME}_core descartes::descartes_light descartes::descartes_samplers descartes::descartes_opw)
tesseract_target_compile_options(${PROJECT_NAME}_descartes PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_descartes)
target_include_directories(${PROJECT_NAME}_descartes PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

# Descartes Hybrid Planner
add_library(${PROJECT_NAME}_descartes_trajopt_array SHARED src/hybrid/descartes_trajopt_array_planner.cpp)
target_link_libraries(${PROJECT_NAME}_descartes_trajopt_array PUBLIC ${PROJECT_NAME}_descartes ${PROJECT_NAME}_trajopt)
tesseract_target_compile_options(${PROJECT_NAME}_descartes_trajopt_array PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_descartes_trajopt_array)
target_include_directories(${PROJECT_NAME}_descartes_trajopt_array PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

tesseract_configure_package(${PROJECT_NAME}_core
                            ${PROJECT_NAME}_trajopt
                            ${PROJECT_NAME}_ompl_freespace
                            ${PROJECT_NAME}_descartes
                            ${PROJECT_NAME}_descartes_trajopt_array)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h*"
  PATTERN ".svn" EXCLUDE
)

if (TESSERACT_ENABLE_TESTING)
  enable_testing()
  tesseract_add_run_tests_target()
  add_subdirectory(test)
endif()
