cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# All of these are used by all components so to avoid having them in each they are include here
find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_state_solver REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(boost_plugin_loader REQUIRED)
find_package(yaml-cpp REQUIRED)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                  ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                    ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/test/*
    /*/install/*
    /*/devel/*
    /*/gtest/*
    /*/bullet/LinearMath/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Define compile-time default variables
if(MSVC)
  set(TESSERACT_KINEMATICS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_KINEMATICS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

# Variable for kinematics plugins
set(KINEMATICS_PLUGINS "")

# Core
set(SUPPORTED_COMPONENTS core)
add_subdirectory(core)

# IKFast
option(TESSERACT_BUILD_IKFAST "Build IKFast components" ON)
if(TESSERACT_BUILD_IKFAST)
  message("Building IKFast components")
  list(APPEND SUPPORTED_COMPONENTS ikfast)
  add_subdirectory(ikfast)
endif()

# KDL
option(TESSERACT_BUILD_KDL "Build KDL components" ON)
if(TESSERACT_BUILD_KDL)
  message("Building KDL components")
  list(APPEND SUPPORTED_COMPONENTS kdl)
  add_subdirectory(kdl)
endif()

# OPW
option(TESSERACT_BUILD_OPW "Build OPW components" ON)
if(TESSERACT_BUILD_OPW)
  message("Building OPW components")
  list(APPEND SUPPORTED_COMPONENTS opw)
  add_subdirectory(opw)
endif()

# UR
option(TESSERACT_BUILD_UR "Build UR components" ON)
if(TESSERACT_BUILD_UR)
  message("Building UR components")
  list(APPEND SUPPORTED_COMPONENTS ur)
  add_subdirectory(ur)
endif()

# Add compiler definition to core so it can find all plugins produced
string(
  REPLACE ";"
          ":"
          KINEMATICS_PLUGINS_STRING
          "${KINEMATICS_PLUGINS}")
target_compile_definitions(${PROJECT_NAME}_core PRIVATE TESSERACT_KINEMATICS_PLUGINS="${KINEMATICS_PLUGINS_STRING}")

# Testing
if((TESSERACT_ENABLE_TESTING OR TESSERACT_KINEMATICS_ENABLE_TESTING)
   AND TESSERACT_BUILD_IKFAST
   AND TESSERACT_BUILD_KDL
   AND TESSERACT_BUILD_OPW
   AND TESSERACT_BUILD_UR)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

configure_package(COMPONENT core SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS})

if(TESSERACT_PACKAGE)
  cpack_component_package(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    COMPONENT_DEPENDS ${SUPPORTED_COMPONENTS})

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
