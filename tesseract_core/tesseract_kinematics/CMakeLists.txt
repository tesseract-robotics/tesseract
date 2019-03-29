cmake_minimum_required(VERSION 3.5.0)
project(tesseract_kinematics)

# define library version (update: apparently you can also do it in project()!)
set(TESSERACT_KINEMATICS_VERSION_MAJOR 0 CACHE STRING "major version" FORCE)
set(TESSERACT_KINEMATICS_VERSION_MINOR 1 CACHE STRING "minor version" FORCE)
set(TESSERACT_KINEMATICS_VERSION_PATCH 0 CACHE STRING "patch version" FORCE)
set(TESSERACT_KINEMATICS_VERSION ${TESSERACT_KINEMATICS_VERSION_MAJOR}.${TESSERACT_KINEMATICS_VERSION_MINOR}.${TESSERACT_KINEMATICS_VERSION_PATCH} CACHE STRING "version" FORCE)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_geometry REQUIRED) #This should not be need I think there is something wrong with how I am exporting targets

set(TESSERACT_KINEMATICS_PKGCONFIG_REQUIRES Eigen3 console_bridge orocos_kdl tesseract_scene_graph)

add_library(${PROJECT_NAME}_kdl SHARED
  src/kdl/kdl_fwd_kin_chain.cpp
  src/kdl/kdl_fwd_kin_tree.cpp
)
target_link_libraries(${PROJECT_NAME}_kdl tesseract_scene_graph tesseract_geometry Eigen3::Eigen ${orocos_kdl_LIBRARIES} ${console_bridge_LIBRARIES})
target_compile_options(${PROJECT_NAME}_kdl PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(${PROJECT_NAME}_kdl PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}_kdl PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_kdl SYSTEM PUBLIC
    ${orocos_kdl_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS})

list (APPEND TESSERACT_KINEMATICS_LIBRARIES ${PROJECT_NAME}_kdl)

# Mark executables and/or libraries for installation
install(TARGETS ${TESSERACT_KINEMATICS_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

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
  VERSION ${TESSERACT_KINEMATICS_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

set(TESSERACT_KINEMATICS_PKGCONFIG_DESCRIPTION "Libraries for performing kinematics")
configure_file(cmake/${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
install(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

option(ENABLE_TESTS "Enable tests" OFF)
if (${ENABLE_TESTS})
  enable_testing()
  add_subdirectory(test)
endif()
