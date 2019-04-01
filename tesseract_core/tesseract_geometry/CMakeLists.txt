cmake_minimum_required(VERSION 3.5.0)
project(tesseract_geometry)

# define library version (update: apparently you can also do it in project()!)
set(TESSERACT_GEOMETRY_VERSION_MAJOR 0 CACHE STRING "major version" FORCE)
set(TESSERACT_GEOMETRY_VERSION_MINOR 1 CACHE STRING "minor version" FORCE)
set(TESSERACT_GEOMETRY_VERSION_PATCH 0 CACHE STRING "patch version" FORCE)
set(TESSERACT_GEOMETRY_VERSION ${TESSERACT_GEOMETRY_VERSION_MAJOR}.${TESSERACT_GEOMETRY_VERSION_MINOR}.${TESSERACT_GEOMETRY_VERSION_PATCH} CACHE STRING "version" FORCE)

find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)

set(TESSERACT_GEOMETRY_PKGCONFIG_REQUIRES Eigen3 console_bridge)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen console_bridge)
target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)

include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/${PROJECT_NAME}-config.cmake.in ${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${PROJECT_NAME}-config-version.cmake
  VERSION ${TESSERACT_GEOMETRY_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

set(TESSERACT_GEOMETRY_PKGCONFIG_DESCRIPTION "Library for geometric shapes")
configure_file(cmake/${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
install(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
