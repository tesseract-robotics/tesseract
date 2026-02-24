find_package(fcl 0.6 REQUIRED)

# Create target for FCL implementation
add_library(
  collision_fcl
  src/fcl_collision_geometry_cache.cpp
  src/fcl_discrete_managers.cpp
  src/fcl_utils.cpp
  src/fcl_collision_object_wrapper.cpp)
add_library(tesseract::collision_fcl ALIAS collision_fcl)
set_target_properties(collision_fcl PROPERTIES OUTPUT_NAME tesseract_collision_fcl)
target_link_libraries(
  collision_fcl
  PUBLIC collision
         Eigen3::Eigen
         tesseract::geometry
         fcl
         console_bridge::console_bridge
         octomap
         octomath)
target_compile_options(collision_fcl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(collision_fcl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(collision_fcl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(collision_fcl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(collision_fcl ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  collision_fcl
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(collision_fcl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:include>")

add_library(collision_fcl_factories src/fcl_factories.cpp)
set_target_properties(collision_fcl_factories PROPERTIES OUTPUT_NAME tesseract_collision_fcl_factories)
target_link_libraries(collision_fcl_factories PUBLIC collision_fcl)
target_compile_options(collision_fcl_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(collision_fcl_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(collision_fcl_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(collision_fcl_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(collision_fcl_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  collision_fcl_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(collision_fcl_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")

# Add factory library so contact_managers_factory can find these factories by defauult
set(CONTACT_MANAGERS_PLUGINS ${CONTACT_MANAGERS_PLUGINS} "tesseract_collision_fcl_factories" PARENT_SCOPE)

# Mark cpp header files for installation
install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT collision_fcl
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

# Plugin should not be export only installed
install(
  TARGETS collision_fcl_factories
  COMPONENT collision_fcl
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

configure_component(
  COMPONENT collision_fcl
  NAMESPACE tesseract
  TARGETS collision_fcl
  DEPENDENCIES "tesseract COMPONENTS collision" "fcl 0.6")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT collision_fcl
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Collision FCL components"
    COMPONENT_DEPENDS collision
    LINUX_DEPENDS "libfcl-dev (>= 0.6.0) | fcl (>= 0.6.0) | ${TESSERACT_PACKAGE_PREFIX}fcl"
    WINDOWS_DEPENDS "libfcl-dev (>= 0.6.0) | fcl (>= 0.6.0) | ${TESSERACT_PACKAGE_PREFIX}fcl")
endif()
