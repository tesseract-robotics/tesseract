#
# @file tesseract_macros.cmae
# @brief Common Tesseract CMake Macros
#
# @author Levi Armstrong
# @date October 15, 2019
# @version TODO
# @bug No known bugs
#
# @copyright Copyright (c) 2019, Southwest Research Institute
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

macro(tesseract_variables)
  if (NOT DEFINED BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS ON)
  endif()

  if (NOT DEFINED TESSERACT_ENABLE_CLANG_TIDY)
    set(TESSERACT_ENABLE_CLANG_TIDY OFF)
  endif()

  if (NOT DEFINED TESSERACT_ENABLE_TESTING)
    set(TESSERACT_ENABLE_TESTING OFF)
  endif()

  if (NOT DEFINED TESSERACT_ENABLE_RUN_TESTING)
    set(TESSERACT_ENABLE_RUN_TESTING OFF)
  endif()

  if (TESSERACT_ENABLE_TESTING_ALL)
    set(TESSERACT_ENABLE_TESTING ON)
    set(TESSERACT_ENABLE_CLANG_TIDY ON)
  endif()

  set(TESSERACT_COMPILE_DEFINITIONS "")
  set(TESSERACT_COMPILE_OPTIONS_PUBLIC "")
  set(TESSERACT_COMPILE_OPTIONS_PRIVATE "")
  if (NOT TESSERACT_ENABLE_TESTING AND NOT TESSERACT_ENABLE_TESTING_ALL)
    set(TESSERACT_CLANG_TIDY_ARGS "-header-filter=.*" "-line-filter=[{'name':'EnvironmentMonitorDynamicReconfigureConfig.h','lines':[[9999999,9999999]]}, {'name':'.h'}, {'name':'.hpp'}]" "-checks=-*,clang-analyzer-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,modernize-*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects")
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(TESSERACT_COMPILE_OPTIONS_PRIVATE -Wall -Wextra -Wconversion -Wsign-conversion -Wno-sign-compare -Wnon-virtual-dtor)
      set(TESSERACT_COMPILE_OPTIONS_PUBLIC -mno-avx)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      set(TESSERACT_COMPILE_OPTIONS_PRIVATE -Wall -Wextra -Wconversion -Wsign-conversion)
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      set(TESSERACT_COMPILE_DEFINITIONS "_USE_MATH_DEFINES=ON")
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    else()
      message(WARNING "${CMAKE_CXX_COMPILER_ID} Unsupported compiler detected.")
    endif()
  else()
    set(TESSERACT_CLANG_TIDY_ARGS "-header-filter=.*" "-line-filter=[{'name':'EnvironmentMonitorDynamicReconfigureConfig.h','lines':[[9999999,9999999]]}, {'name':'.h'}, {'name':'.hpp'}]" "-checks=-*,clang-analyzer-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,modernize-*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects" "-warnings-as-errors=-*,clang-analyzer-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,modernize-*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects")
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(TESSERACT_COMPILE_OPTIONS_PRIVATE -Werror=all -Werror=extra -Werror=conversion -Werror=sign-conversion -Wno-sign-compare -Wnon-virtual-dtor)
      set(TESSERACT_COMPILE_OPTIONS_PUBLIC -mno-avx)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      set(TESSERACT_COMPILE_OPTIONS_PRIVATE -Werror=all -Werror=extra -Werror=conversion -Werror=sign-conversion)
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
