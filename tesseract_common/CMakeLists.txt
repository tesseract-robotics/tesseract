cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

include(cmake/tesseract_macros.cmake)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

if(NOT APPLE)
  find_package(
    Boost REQUIRED
    COMPONENTS system
               filesystem
               serialization
               stacktrace_noop
    OPTIONAL_COMPONENTS stacktrace_backtrace stacktrace_basic)
else()
  find_package(
    Boost REQUIRED
    COMPONENTS system
               filesystem
               serialization
               stacktrace_noop)
endif()
find_package(Eigen3 REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(yaml-cpp REQUIRED)

if(TARGET Boost::stacktrace_backtrace)
  find_file(BACKTRACE_INCLUDE_FILE backtrace.h PATHS ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
  if(BACKTRACE_INCLUDE_FILE)
    set(TESSERACT_BACKTRACE_DEFINITION BOOST_STACKTRACE_USE_BACKTRACE
                                       BOOST_STACKTRACE_BACKTRACE_INCLUDE_FILE="${BACKTRACE_INCLUDE_FILE}")
  else()
    set(TESSERACT_BACKTRACE_DEFINITION BOOST_STACKTRACE_USE_BACKTRACE)
  endif()
  set(TESSERACT_BACKTRACE_COMPONENT stacktrace_backtrace)
  set(TESSERACT_BACKTRACE_LIB Boost::stacktrace_backtrace)
elseif(TARGET Boost::stacktrace_basic)
  set(TESSERACT_BACKTRACE_DEFINITION "")
  set(TESSERACT_BACKTRACE_COMPONENT "stacktrace_basic")
  set(TESSERACT_BACKTRACE_LIB "Boost::stacktrace_basic")
elseif(TARGET Boost::stacktrace_noop)
  set(TESSERACT_BACKTRACE_DEFINITION BOOST_STACKTRACE_USE_NOOP)
  set(TESSERACT_BACKTRACE_COMPONENT "stacktrace_noop")
  set(TESSERACT_BACKTRACE_LIB "Boost::stacktrace_noop")
endif()

find_package(console_bridge REQUIRED)
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
    /*/gtest/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

add_library(
  ${PROJECT_NAME}
  src/allowed_collision_matrix.cpp
  src/any_poly.cpp
  src/calibration_info.cpp
  src/collision_margin_data.cpp
  src/contact_allowed_validator.cpp
  src/joint_state.cpp
  src/manipulator_info.cpp
  src/kinematic_limits.cpp
  src/eigen_serialization.cpp
  src/utils.cpp
  src/plugin_info.cpp
  src/profile_dictionary.cpp
  src/profile.cpp
  src/resource_locator.cpp
  src/types.cpp
  src/stopwatch.cpp
  src/timer.cpp
  src/yaml_utils.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
         tinyxml2::tinyxml2
         ${CMAKE_DL_LIBS}
         Boost::boost
         Boost::system
         Boost::filesystem
         Boost::serialization
         ${TESSERACT_BACKTRACE_LIB}
         console_bridge::console_bridge
         yaml-cpp)
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS} ${TESSERACT_BACKTRACE_DEFINITION})
target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME})
if(WIN32)
  target_link_libraries(${PROJECT_NAME} PUBLIC Bcrypt)
endif()

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

install(
  FILES "${CMAKE_CURRENT_LIST_DIR}/cmake/tesseract_macros.cmake"
        "${CMAKE_CURRENT_LIST_DIR}/cmake/FindTinyXML2.cmake"
        "${CMAKE_CURRENT_LIST_DIR}/cmake/Findtcmalloc.cmake"
        "${CMAKE_CURRENT_LIST_DIR}/cmake/Findtcmalloc_minimal.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

if(TESSERACT_ENABLE_TESTING OR TESSERACT_COMMON_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS
      "libboost-system-dev"
      "libboost-filesystem-dev"
      "libboost-serialization-dev"
      "libconsole-bridge-dev"
      "libtinyxml2-dev"
      "libeigen3-dev"
      "libyaml-cpp-dev"
    WINDOWS_DEPENDS
      "boost_system"
      "boost_filesystem"
      "boost_serialization"
      "console-bridge"
      "tinyxml2"
      "Eigen3"
      "yaml-cpp")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
