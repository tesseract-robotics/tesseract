find_package(fcl 0.6 REQUIRED)

include_directories(BEFORE ${FCL_INCLUDE_DIRS})
link_directories(BEFORE ${FCL_LIBRARY_DIRS})

# Create target for FCL implementation
add_library(${PROJECT_NAME}_fcl
  src/fcl_discrete_managers.cpp
  src/fcl_utils.cpp
  src/fcl_collision_object_wrapper.cpp)
target_link_libraries(
  ${PROJECT_NAME}_fcl
  PUBLIC ${PROJECT_NAME}_core
         Eigen3::Eigen
         tesseract::tesseract_geometry
         fcl
         console_bridge::console_bridge
         octomap
         octomath)
target_compile_options(${PROJECT_NAME}_fcl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_fcl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_fcl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_fcl ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_fcl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_fcl
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_fcl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                      "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

install_targets(TARGETS ${PROJECT_NAME}_fcl)
