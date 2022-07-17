find_bullet()

# Third party vhacd
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/vhacd_common.cmake")

find_package(OpenMP REQUIRED)

option(NO_OPENCL "NO_OPENCL" OFF)
message("NO_OPENCL " ${NO_OPENCL})

if(NOT NO_OPENCL)
  find_package(OpenCL)
endif()

if(NOT TARGET OpenMP::OpenMP_CXX)
  find_package(Threads REQUIRED)
  add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
  # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
endif()

add_library(
  ${PROJECT_NAME}_vhacd
  ${VHACD_CPP_FILES}
  ${VHACD_C_FILES}
  ${VHACD_INC_FILES}
  ${VHACD_INL_FILES}
  ${VHACD_CL_FILES})
target_cxx_version(${PROJECT_NAME}_vhacd PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_compile_options(${PROJECT_NAME}_vhacd PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_vhacd PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_vhacd PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})

if(OpenCL_FOUND)
  target_include_directories(${PROJECT_NAME}_vhacd PRIVATE "${OpenCL_INCLUDE_DIR}")
  target_link_libraries(${PROJECT_NAME}_vhacd PRIVATE "${OpenCL_LIBRARY}")
  target_compile_definitions(${PROJECT_NAME}_vhacd PRIVATE -DOPENCL_FOUND=1)
  target_compile_definitions(${PROJECT_NAME}_vhacd PRIVATE -DOPENCL_CL_FILES="${PROJECT_CL_FILES}")
endif()

target_link_libraries(
  ${PROJECT_NAME}_vhacd
  PUBLIC OpenMP::OpenMP_CXX
         tesseract::tesseract_common
         Eigen3::Eigen
         Bullet3::Bullet)
target_include_directories(${PROJECT_NAME}_vhacd PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                        "$<INSTALL_INTERFACE:include>")

# Convex decomposition libraries
add_library(${PROJECT_NAME}_vhacd_convex_decomposition src/convex_decomposition_vhacd.cpp)
target_link_libraries(
  ${PROJECT_NAME}_vhacd_convex_decomposition
  PUBLIC ${PROJECT_NAME}_core
         ${PROJECT_NAME}_bullet
         ${PROJECT_NAME}_vhacd
         Eigen3::Eigen
         tesseract::tesseract_geometry
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_vhacd_convex_decomposition PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_vhacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_vhacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_vhacd_convex_decomposition PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_vhacd_convex_decomposition ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_vhacd_convex_decomposition
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_vhacd_convex_decomposition PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                    "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

install_targets(TARGETS ${PROJECT_NAME}_vhacd ${PROJECT_NAME}_vhacd_convex_decomposition)
