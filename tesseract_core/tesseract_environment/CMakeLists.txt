cmake_minimum_required(VERSION 3.5.0)
project(tesseract_environment)

# define library version (update: apparently you can also do it in project()!)
set(TESSERACT_ENVIRONMENT_VERSION_MAJOR 0 CACHE STRING "major version" FORCE)
set(TESSERACT_ENVIRONMENT_VERSION_MINOR 1 CACHE STRING "minor version" FORCE)
set(TESSERACT_ENVIRONMENT_VERSION_PATCH 0 CACHE STRING "patch version" FORCE)
set(TESSERACT_ENVIRONMENT_VERSION ${TESSERACT_ENVIRONMENT_VERSION_MAJOR}.${TESSERACT_ENVIRONMENT_VERSION_MINOR}.${TESSERACT_ENVIRONMENT_VERSION_PATCH} CACHE STRING "version" FORCE)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_scene_graph REQUIRED)

set(TESSERACT_ENVIRONMENT_PKGCONFIG_REQUIRES Eigen3 orocos_kdl console_bridge tesseract_collision tesseract_geometry tesseract_scene_graph)

add_library(${PROJECT_NAME}_kdl SHARED src/kdl/kdl_env.cpp)
target_link_libraries(${PROJECT_NAME}_kdl tesseract_geometry tesseract_collision_fcl tesseract_collision_bullet tesseract_scene_graph Eigen3::Eigen ${BULLET_LIBRARIES} ${orocos_kdl_LIBRARIES} ${console_bridge_LIBRARIES})
target_compile_options(${PROJECT_NAME}_kdl PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(${PROJECT_NAME}_kdl PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}_kdl PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_kdl SYSTEM INTERFACE
    ${orocos_kdl_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS})

list (APPEND TESSERACT_ENVIRONMENT_LIBRARIES ${PROJECT_NAME}_kdl)

install(TARGETS ${TESSERACT_ENVIRONMENT_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/${PROJECT_NAME}-config.cmake.in ${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${PROJECT_NAME}-config-version.cmake
  VERSION ${TESSERACT_ENVIRONMENT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

set(TESSERACT_ENVIRONMENT_PKGCONFIG_DESCRIPTION "Libraries for building environments")
configure_file(cmake/${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
install(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

option(ENABLE_TESTS "Enable tests" OFF)
if (${ENABLE_TESTS})
  enable_testing()
  add_subdirectory(test)
endif()
