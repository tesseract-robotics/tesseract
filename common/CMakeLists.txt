if(NOT APPLE)
  find_package(Boost REQUIRED COMPONENTS system filesystem stacktrace_noop OPTIONAL_COMPONENTS stacktrace_backtrace
                                                                                               stacktrace_basic)
else()
  find_package(Boost REQUIRED COMPONENTS system filesystem stacktrace_noop)
endif()

find_package(TinyXML2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(boost_plugin_loader REQUIRED)

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

set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} "common" PARENT_SCOPE)

add_library(
  common
  src/allowed_collision_matrix.cpp
  src/any_poly.cpp
  src/calibration_info.cpp
  src/cereal_serialization.cpp
  src/collision_margin_data.cpp
  src/contact_allowed_validator.cpp
  src/joint_state.cpp
  src/manipulator_info.cpp
  src/kinematic_limits.cpp
  src/utils.cpp
  src/plugin_info.cpp
  src/ply_io.cpp
  src/profile_dictionary.cpp
  src/profile_plugin_factory.cpp
  src/profile.cpp
  src/resource_locator.cpp
  src/types.cpp
  src/stopwatch.cpp
  src/timer.cpp
  src/yaml_utils.cpp)
add_library(tesseract::common ALIAS common)
set_target_properties(common PROPERTIES OUTPUT_NAME tesseract_common)
target_link_libraries(
  common
  PUBLIC Eigen3::Eigen
         tinyxml2::tinyxml2
         boost_plugin_loader::boost_plugin_loader
         ${CMAKE_DL_LIBS}
         Boost::boost
         Boost::system
         Boost::filesystem
         ${TESSERACT_BACKTRACE_LIB}
         console_bridge::console_bridge
         cereal::cereal
         yaml-cpp)
target_compile_options(common PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(common PUBLIC ${TESSERACT_COMPILE_DEFINITIONS} ${TESSERACT_BACKTRACE_DEFINITION})
target_clang_tidy(common ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(common PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  common
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(common PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                         "$<INSTALL_INTERFACE:include>")

if(WIN32)
  target_link_libraries(common PUBLIC Bcrypt)
endif()

# configure_package(NAMESPACE tesseract TARGETS common)
configure_component(COMPONENT common NAMESPACE tesseract TARGETS common)

install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT common
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

install(
  FILES "${CMAKE_CURRENT_LIST_DIR}/cmake/tesseract_macros.cmake"
        "${CMAKE_CURRENT_LIST_DIR}/cmake/FindTinyXML2.cmake"
        "${CMAKE_CURRENT_LIST_DIR}/cmake/Findtcmalloc.cmake"
        "${CMAKE_CURRENT_LIST_DIR}/cmake/Findtcmalloc_minimal.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
  COMPONENT common)

if(TESSERACT_ENABLE_TESTING OR TESSERACT_COMMON_ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT common
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Common Components"
    LINUX_DEPENDS
      "libboost-system-dev"
      "libboost-filesystem-dev"
      "libconsole-bridge-dev"
      "libtinyxml2-dev"
      "libeigen3-dev"
      "libyaml-cpp-dev"
      "cereal"
    WINDOWS_DEPENDS
      "boost_system"
      "boost_filesystem"
      "console-bridge"
      "tinyxml2"
      "Eigen3"
      "yaml-cpp"
      "cereal")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
