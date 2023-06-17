# Convex decomposition library
add_library(${PROJECT_NAME}_vhacd_convex_decomposition src/convex_decomposition_vhacd.cpp)
target_link_libraries(
  ${PROJECT_NAME}_vhacd_convex_decomposition
  PUBLIC ${PROJECT_NAME}_core
         ${PROJECT_NAME}_bullet
         Eigen3::Eigen
         tesseract::tesseract_geometry
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_vhacd_convex_decomposition PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_vhacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_vhacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_vhacd_convex_decomposition PUBLIC VERSION ${TESSERACT_CXX_VERSION})
# target_clang_tidy(${PROJECT_NAME}_vhacd_convex_decomposition ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})

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

install_targets(TARGETS ${PROJECT_NAME}_vhacd_convex_decomposition)
