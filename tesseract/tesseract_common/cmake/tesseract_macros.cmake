#
# @file tesseract_macros.cmae
# @brief Common Tesseract CMake Macros
#
# @author Levi Armstrong
# @date January 18, 2018
# @version TODO
# @bug No known bugs
#
# @copyright Copyright (c) 2017, Southwest Research Institute
#
# @par License
# Software License Agreement (Apache License)
# @par
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# @par
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

macro(tesseract_target_compile_options target)
  cmake_parse_arguments(ARG "INTERFACE;PUBLIC;PRIVATE" "" "" ${ARGN})

  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "tesseract_target_compile_options() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

  if (ARG_INTERFACE)
    target_compile_options(${target} INTERFACE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
    if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options("${target}" INTERFACE -std=c++11)
    else()
        target_compile_features("${target}" INTERFACE cxx_std_11)
    endif()
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options("${target}" INTERFACE -mno-avx)
    else()
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  elseif(ARG_PUBLIC)
    target_compile_options("${target}" PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
    if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options("${target}" PUBLIC -std=c++11)
    else()
        target_compile_features("${target}" PUBLIC cxx_std_11)
    endif()
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options("${target}" PUBLIC -mno-avx)
    else()
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  elseif(ARG_PRIVATE)
    target_compile_options("${target}" PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
    if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options("${target}" PRIVATE -std=c++11)
    else()
        target_compile_features("${target}" PRIVATE cxx_std_11)
    endif()
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options("${target}" PRIVATE -mno-avx)
    else()
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  endif()

endmacro()

macro(tesseract_configure_package)
  install(TARGETS ${ARGV}
          EXPORT ${PROJECT_NAME}-targets
          RUNTIME DESTINATION bin
          LIBRARY DESTINATION lib)
  install(EXPORT ${PROJECT_NAME}-targets NAMESPACE tesseract:: DESTINATION lib/cmake/${PROJECT_NAME})

  install(FILES package.xml DESTINATION share/${PROJECT_NAME})

  # Create cmake config files
  include(CMakePackageConfigHelpers)
  configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

  write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME})

  export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
endmacro()

macro(tesseract_gtest_discover_tests target)
  if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
      gtest_add_tests(${target} "" AUTO)
  else()
      gtest_discover_tests(${target})
  endif()
endmacro()
