find_bullet()

# Create target for Bullet implementation
add_library(
  collision_bullet
  src/bullet_cast_bvh_manager.cpp
  src/bullet_cast_simple_manager.cpp
  src/bullet_collision_shape_cache.cpp
  src/bullet_discrete_bvh_manager.cpp
  src/bullet_discrete_simple_manager.cpp
  src/bullet_utils.cpp
  src/convex_hull_utils.cpp
  src/tesseract_compound_collision_algorithm.cpp
  src/tesseract_compound_compound_collision_algorithm.cpp
  src/tesseract_collision_configuration.cpp
  src/tesseract_convex_convex_algorithm.cpp
  src/tesseract_gjk_pair_detector.cpp)
add_library(tesseract::collision_bullet ALIAS collision_bullet)
set_target_properties(collision_bullet PROPERTIES OUTPUT_NAME tesseract_collision_bullet)
target_link_libraries(
  collision_bullet
  PUBLIC collision
         Eigen3::Eigen
         Bullet3::Bullet
         tesseract::geometry
         console_bridge::console_bridge
         octomap
         octomath)
target_compile_options(collision_bullet PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(collision_bullet PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(collision_bullet PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(collision_bullet PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(collision_bullet ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  collision_bullet
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(collision_bullet PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                   "$<INSTALL_INTERFACE:include>")

# Create target for Bullet implementation
add_library(collision_bullet_factories src/bullet_factories.cpp)
add_library(tesseract::collision_bullet_factories ALIAS collision_bullet_factories)
set_target_properties(collision_bullet_factories PROPERTIES OUTPUT_NAME tesseract_collision_bullet_factories)
target_link_libraries(collision_bullet_factories PUBLIC collision_bullet)
target_compile_options(collision_bullet_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(collision_bullet_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(collision_bullet_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(collision_bullet_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(collision_bullet_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  collision_bullet_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(collision_bullet_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                             "$<INSTALL_INTERFACE:include>")

# Convex decomposition libraries
if(${HACD_LIBRARY})
  add_library(collision_hacd_convex_decomposition src/convex_decomposition_hacd.cpp)
  add_library(tesseract::collision_hacd_convex_decomposition ALIAS collision_hacd_convex_decomposition)
  set_target_properties(collision_hacd_convex_decomposition PROPERTIES OUTPUT_NAME
                                                                       tesseract_collision_hacd_convex_decomposition)
  target_link_libraries(
    collision_hacd_convex_decomposition
    PUBLIC collision_bullet
           Eigen3::Eigen
           Bullet3::Bullet
           tesseract::geometry
           console_bridge::console_bridge
           ${HACD_LIBRARY})
  target_compile_options(collision_hacd_convex_decomposition PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
  target_compile_options(collision_hacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(collision_hacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
  target_cxx_version(collision_hacd_convex_decomposition PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_clang_tidy(collision_hacd_convex_decomposition ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_code_coverage(
    collision_hacd_convex_decomposition
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  target_include_directories(
    collision_hacd_convex_decomposition PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                               "$<INSTALL_INTERFACE:include>")
  install_targets(TARGETS collision_hacd_convex_decomposition COMPONENT bullet)
endif()

if(NOT MSVC)
  # Create target for creating convex hulls from meshes
  add_executable(create_convex_hull src/create_convex_hull.cpp)
  target_link_libraries(
    create_convex_hull
    PUBLIC collision
           collision_bullet
           tesseract::common
           tesseract::geometry
           Bullet3::Bullet
           Boost::boost
           Boost::program_options
           Eigen3::Eigen
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

  install_targets(TARGETS create_convex_hull COMPONENT bullet)
endif()

# Add factory library so contact_managers_factory can find these factories by defauult
set(CONTACT_MANAGERS_PLUGINS ${CONTACT_MANAGERS_PLUGINS} "tesseract_collision_bullet_factories" PARENT_SCOPE)

# Mark cpp header files for installation
install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT collision_bullet
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

# Plugin should not be export only installed
install(
  TARGETS collision_bullet_factories
  COMPONENT collision_bullet
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

configure_component(
  COMPONENT collision_bullet
  NAMESPACE tesseract
  TARGETS collision_bullet
  DEPENDENCIES "tesseract COMPONENTS collision"
  CFG_EXTRAS cmake/bullet-extras.cmake)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT collision_bullet
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Collision bullet components"
    COMPONENT_DEPENDS collision
    LINUX_DEPENDS "libbullet-dev" "libbullet-extras-dev"
    WINDOWS_DEPENDS "libbullet-dev" "libbullet-extras-dev")
endif()
