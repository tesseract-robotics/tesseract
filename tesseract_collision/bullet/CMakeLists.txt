find_bullet()

# Create target for Bullet implementation
add_library(
  ${PROJECT_NAME}_bullet
  src/bullet_cast_bvh_manager.cpp
  src/bullet_cast_simple_manager.cpp
  src/bullet_discrete_bvh_manager.cpp
  src/bullet_discrete_simple_manager.cpp
  src/bullet_utils.cpp
  src/convex_hull_utils.cpp
  src/tesseract_compound_collision_algorithm.cpp
  src/tesseract_compound_compound_collision_algorithm.cpp
  src/tesseract_collision_configuration.cpp
  src/tesseract_convex_convex_algorithm.cpp
  src/tesseract_gjk_pair_detector.cpp)
target_link_libraries(
  ${PROJECT_NAME}_bullet
  PUBLIC ${PROJECT_NAME}_core
         Eigen3::Eigen
         Bullet3::Bullet
         tesseract::tesseract_geometry
         console_bridge::console_bridge
         octomap
         octomath)
target_compile_options(${PROJECT_NAME}_bullet PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_bullet PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_bullet PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_bullet PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_bullet ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_bullet
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_bullet PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:include>")

# Create target for Bullet implementation
add_library(${PROJECT_NAME}_bullet_factories src/bullet_factories.cpp)
target_link_libraries(${PROJECT_NAME}_bullet_factories PUBLIC ${PROJECT_NAME}_bullet)
target_compile_options(${PROJECT_NAME}_bullet_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_bullet_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_bullet_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_bullet_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_bullet_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_bullet_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_bullet_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                          "$<INSTALL_INTERFACE:include>")

# Convex decomposition libraries
if(${HACD_LIBRARY})
  add_library(${PROJECT_NAME}_hacd_convex_decomposition src/convex_decomposition_hacd.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_hacd_convex_decomposition
    PUBLIC ${PROJECT_NAME}_bullet
           Eigen3::Eigen
           Bullet3::Bullet
           tesseract::tesseract_geometry
           console_bridge::console_bridge
           ${HACD_LIBRARY})
  target_compile_options(${PROJECT_NAME}_hacd_convex_decomposition PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
  target_compile_options(${PROJECT_NAME}_hacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_hacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
  target_cxx_version(${PROJECT_NAME}_hacd_convex_decomposition PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_clang_tidy(${PROJECT_NAME}_hacd_convex_decomposition ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_code_coverage(
    ${PROJECT_NAME}_hacd_convex_decomposition
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  target_include_directories(
    ${PROJECT_NAME}_hacd_convex_decomposition PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                     "$<INSTALL_INTERFACE:include>")
  install_targets(TARGETS ${PROJECT_NAME}_hacd_convex_decomposition)
endif()

if(NOT MSVC)
  # Create target for creating convex hulls from meshes
  add_executable(create_convex_hull src/create_convex_hull.cpp)
  target_link_libraries(
    create_convex_hull
    PUBLIC ${PROJECT_NAME}_core
           ${PROJECT_NAME}_bullet
           Bullet3::Bullet
           Boost::boost
           Boost::program_options
           Eigen3::Eigen
           tesseract::tesseract_common
           tesseract::tesseract_geometry
           console_bridge::console_bridge
           octomap
           octomath)
  target_compile_options(create_convex_hull PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                    ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(create_convex_hull PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_cxx_version(create_convex_hull PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_clang_tidy(create_convex_hull ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_include_directories(create_convex_hull PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

  list(APPEND PACKAGE_LIBRARIES create_convex_hull)

  install_targets(TARGETS create_convex_hull)
endif()

# Add factory library so contact_managers_factory can find these factories by defauult
set(CONTACT_MANAGERS_PLUGINS ${CONTACT_MANAGERS_PLUGINS} "${PROJECT_NAME}_bullet_factories" PARENT_SCOPE)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

install_targets(TARGETS ${PROJECT_NAME}_bullet ${PROJECT_NAME}_bullet_factories)
