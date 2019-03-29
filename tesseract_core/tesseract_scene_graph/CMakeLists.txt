cmake_minimum_required(VERSION 3.5.0)
project(tesseract_scene_graph)

# define library version (update: apparently you can also do it in project()!)
set(TESSERACT_SCENE_GRAPH_VERSION_MAJOR 0 CACHE STRING "major version" FORCE)
set(TESSERACT_SCENE_GRAPH_VERSION_MINOR 1 CACHE STRING "minor version" FORCE)
set(TESSERACT_SCENE_GRAPH_VERSION_PATCH 0 CACHE STRING "patch version" FORCE)
set(TESSERACT_SCENE_GRAPH_VERSION ${TESSERACT_SCENE_GRAPH_VERSION_MAJOR}.${TESSERACT_SCENE_GRAPH_VERSION_MINOR}.${TESSERACT_SCENE_GRAPH_VERSION_PATCH} CACHE STRING "version" FORCE)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(console_bridge REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(TinyXML REQUIRED)
find_package(tesseract_geometry REQUIRED)

find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  find_package(PkgConfig REQUIRED)
  # assimp is required, so REQUIRE the second attempt
  pkg_check_modules(ASSIMP REQUIRED assimp)
endif()

set(TESSERACT_SCENE_GRAPH_PKGCONFIG_REQUIRES Eigen3 urdfdom urdfdom_headers orocos_kdl TinyXML ASSIMP console_bridge tesseract_geometry)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE tesseract_geometry Eigen3::Eigen ${Boost_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${ASSIMP_LIBRARIES} ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES} ${console_bridge_LIBRARIES})
target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_definitions(${PROJECT_NAME} INTERFACE Boost_USE_STATIC_LIBS=OFF Boost_USE_MULTITHREADED=ON Boost_USE_STATIC_RUNTIME=OFF)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE
    ${Boost_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES cmake/FindTinyXML.cmake DESTINATION lib/cmake/${PROJECT_NAME})

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/${PROJECT_NAME}-config.cmake.in ${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${PROJECT_NAME}-config-version.cmake
  VERSION ${TESSERACT_SCENE_GRAPH_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

set(TESSERACT_SCENE_GRAPH_PKGCONFIG_DESCRIPTION "Libraries for building scene graph")
configure_file(cmake/${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
install(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

option(ENABLE_TESTS "Enable tests" OFF)
if (${ENABLE_TESTS})
  enable_testing()
  add_subdirectory(test)
endif()
