#
# @file tesseract_macros.cmae @brief Common Tesseract CMake Macros
#
# @author Levi Armstrong @date October 15, 2019 @version TODO @bug No known bugs
#
# @copyright Copyright (c) 2019, Southwest Research Institute
#
# @par License Software License Agreement (Apache License) @par Licensed under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0 @par Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing permissions and limitations under the License.

macro(tesseract_variables)
  if(NOT DEFINED BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS ON)
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_EXAMPLES)
    set(TESSERACT_ENABLE_EXAMPLES ON)
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_CLANG_TIDY)
    set(TESSERACT_ENABLE_CLANG_TIDY OFF)
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_CODE_COVERAGE)
    set(TESSERACT_ENABLE_CODE_COVERAGE OFF)
  elseif(TESSERACT_ENABLE_CODE_COVERAGE)
    set(TESSERACT_ENABLE_TESTING ON)
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_TESTING)
    set(TESSERACT_ENABLE_TESTING OFF)
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_EXAMPLES)
    set(TESSERACT_ENABLE_EXAMPLES ON)
  endif()

  if(NOT DEFINED TESSERACT_WARNINGS_AS_ERRORS)
    set(TESSERACT_WARNINGS_AS_ERRORS ${TESSERACT_ENABLE_TESTING})
  endif()

  if(NOT DEFINED TESSERACT_ENABLE_RUN_TESTING)
    set(TESSERACT_ENABLE_RUN_TESTING OFF)
  endif()

  if(TESSERACT_ENABLE_TESTING_ALL)
    set(TESSERACT_ENABLE_TESTING ON)
    set(TESSERACT_ENABLE_CLANG_TIDY ON)
    set(TESSERACT_ENABLE_CODE_COVERAGE ON)
  endif()

  set(TESSERACT_COMPILE_DEFINITIONS "")
  set(TESSERACT_COMPILE_OPTIONS_PUBLIC "")
  set(TESSERACT_COMPILE_OPTIONS_PRIVATE "")
  if(NOT TESSERACT_WARNINGS_AS_ERRORS)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(TESSERACT_COMPILE_OPTIONS_PRIVATE
          -Wall
          -Wextra
          -Wconversion
          -Wsign-conversion
          -Wno-sign-compare
          -Wnon-virtual-dtor)
      exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
      if(NOT
         CMAKE_SYSTEM_NAME2
         MATCHES
         "aarch64"
         AND NOT
             CMAKE_SYSTEM_NAME2
             MATCHES
             "armv7l"
         AND NOT
             CMAKE_SYSTEM_NAME2
             MATCHES
             "unknown")
        set(TESSERACT_COMPILE_OPTIONS_PUBLIC -mno-avx)
      endif()
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      set(TESSERACT_COMPILE_OPTIONS_PRIVATE
          -Wall
          -Wextra
          -Wconversion
          -Wsign-conversion)
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      set(TESSERACT_COMPILE_DEFINITIONS "_USE_MATH_DEFINES=ON")
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    else()
      message(WARNING "${CMAKE_CXX_COMPILER_ID} Unsupported compiler detected.")
    endif()
  else()
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(TESSERACT_COMPILE_OPTIONS_PRIVATE
          -Werror=all
          -Werror=extra
          -Werror=conversion
          -Werror=sign-conversion
          -Wno-sign-compare
          -Werror=non-virtual-dtor)
      exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
      if(NOT
         CMAKE_SYSTEM_NAME2
         MATCHES
         "aarch64"
         AND NOT
             CMAKE_SYSTEM_NAME2
             MATCHES
             "armv7l"
         AND NOT
             CMAKE_SYSTEM_NAME2
             MATCHES
             "unknown")
        set(TESSERACT_COMPILE_OPTIONS_PUBLIC -mno-avx)
      endif()
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      set(TESSERACT_COMPILE_OPTIONS_PRIVATE
          -Werror=all
          -Werror=extra
          -Werror=conversion
          -Werror=sign-conversion)
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      set(TESSERACT_COMPILE_DEFINITIONS "_USE_MATH_DEFINES=ON")
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    else()
      message(WARNING "${CMAKE_CXX_COMPILER_ID} Unsupported compiler detected.")
    endif()
  endif()

  set(TESSERACT_CXX_VERSION 17)
endmacro()

macro(tesseract_cpack)
  set(oneValueArgs
      VERSION
      MAINTAINER
      DESCRIPTION
      LICENSE_FILE
      README_FILE)
  set(multiValueArgs LINUX_DEPENDS WINDOWS_DEPENDS)
  cmake_parse_arguments(
    ARG
    ""
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  set(CPACK_PACKAGE_VENDOR "ROS-Industrial")
  set(CPACK_RESOURCE_FILE_LICENSE ${ARG_LICENSE_FILE})
  set(CPACK_RESOURCE_FILE_README ${ARG_README_FILE})
  if(UNIX)
    string(
      REPLACE "_"
              "-"
              TESSERACT_PACKAGE_NAME
              ${PROJECT_NAME})
    set(CPACK_GENERATOR "DEB;TXZ")

    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
      set(DEB_ARCH "amd64")
    else()
      set(DEB_ARCH ${CMAKE_SYSTEM_PROCESSOR})
    endif()

    set(CPACK_PACKAGE_FILE_NAME "${TESSERACT_PACKAGE_PREFIX}${TESSERACT_PACKAGE_NAME}_${DEB_ARCH}_linux_${ARG_VERSION}")
    set(CPACK_DEBIAN_PACKAGE_NAME "${TESSERACT_PACKAGE_PREFIX}${TESSERACT_PACKAGE_NAME}")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${DEB_ARCH})
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER ${ARG_MAINTAINER})
    set(CPACK_DEBIAN_PACKAGE_DESCRIPTION ${ARG_DESCRIPTION})
    set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS=ON)
    string(
      REPLACE ";"
              ","
              CPACK_DEBIAN_PACKAGE_DEPENDS
              "${ARG_LINUX_DEPENDS}")
  elseif(WIN32)
    set(CPACK_GENERATOR "NuGet;TXZ")
    set(CPACK_PACKAGE_FILE_NAME
        "${TESSERACT_PACKAGE_PREFIX}${TESSERACT_PACKAGE_NAME}_${CMAKE_SYSTEM_PROCESSOR}_windows_${ARG_VERSION}")
    set(CPACK_NUGET_PACKAGE_NAME
        "${TESSERACT_PACKAGE_PREFIX}${TESSERACT_PACKAGE_NAME}_${CMAKE_SYSTEM_PROCESSOR}_windows")
    set(CPACK_NUGET_PACKAGE_DESCRIPTION ${ARG_DESCRIPTION})
    string(
      REPLACE ";"
              ","
              CPACK_NUGET_PACKAGE_DEPENDENCIES
              "${ARG_WINDOWS_DEPENDS}")
  endif()
  include(CPack)
endmacro()

macro(find_bullet)
  find_package(
    Bullet
    REQUIRED
    CONFIGS
    BulletConfig-float64.cmake
    BulletConfig.cmake)
  if(NOT
     ${BULLET_DEFINITIONS}
     MATCHES
     ".*-DBT_USE_DOUBLE_PRECISION.*")
    message(
      WARNING "Bullet does not appear to be build with double precision, current definitions: ${BULLET_DEFINITIONS}")
  endif()

  # Some Bullet installations (vcpkg) use absolute paths instead of relative to BULLET_ROOT_DIR in the CMake vars
  set(BULLET_INCLUDE_DIRS_ABS "")
  set(BULLET_LIBRARY_DIRS_ABS "")
  if(NOT IS_ABSOLUTE "${BULLET_INCLUDE_DIR}")
    foreach(dir IN LISTS BULLET_INCLUDE_DIRS)
      list(APPEND BULLET_INCLUDE_DIRS_ABS "${BULLET_ROOT_DIR}/${dir}")
    endforeach()
    foreach(dir IN LISTS BULLET_LIBRARY_DIRS)
      list(APPEND BULLET_LIBRARY_DIRS_ABS "${BULLET_ROOT_DIR}/${dir}")
    endforeach()
  else()
    set(BULLET_INCLUDE_DIRS_ABS ${BULLET_INCLUDE_DIRS})
    set(BULLET_LIBRARY_DIRS_ABS ${BULLET_LIBRARY_DIRS})
  endif()

  set(BULLET_LIBRARIES_ABS "")
  foreach(BULLET_LIB IN LISTS BULLET_LIBRARIES)
    find_library(BULLET_LIB_ABS_${BULLET_LIB} ${BULLET_LIB} PATHS ${BULLET_LIBRARY_DIRS_ABS} NO_DEFAULT_PATH REQUIRED)
    list(APPEND BULLET_LIBRARIES_ABS "${BULLET_LIB_ABS_${BULLET_LIB}}")
    message(STATUS "BULLET_LIB=${BULLET_LIB} BULLET_LIB_ABS=${BULLET_LIB_ABS_${BULLET_LIB}}")
  endforeach()
  message(STATUS "BULLET_LIBRARIES_ABS=${BULLET_LIBRARIES_ABS}")

  set(BULLET_DEFINITIONS_STRIPED "")
  foreach(DEF ${BULLET_DEFINITIONS})
    string(STRIP ${DEF} DEF)
    if(NOT
       "${DEF}"
       STREQUAL
       "")
      string(LENGTH ${DEF} DEF_LENGTH)
      string(
        SUBSTRING ${DEF}
                  2
                  ${DEF_LENGTH}
                  DEF)
      list(APPEND BULLET_DEFINITIONS_STRIPED ${DEF})
    endif()
  endforeach()

  if(NOT TARGET Bullet3::Bullet)
    add_library(Bullet3::Bullet INTERFACE IMPORTED)
    set_target_properties(Bullet3::Bullet PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${BULLET_INCLUDE_DIRS_ABS}")
    set_target_properties(Bullet3::Bullet PROPERTIES INTERFACE_LINK_LIBRARIES "${BULLET_LIBRARIES_ABS}")
    set_target_properties(Bullet3::Bullet PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${BULLET_DEFINITIONS_STRIPED}")
  endif()

  find_library(HACD_LIBRARY HACD HINTS ${BULLET_LIBRARY_DIRS_ABS})
  if(NOT HACD_LIBRARY)
    message(
      WARNING "HACD not found! Convex decomposition library will not be built. Install libbullet-extras-dev on Linux.")
  endif()
endmacro()
