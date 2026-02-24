add_executable(tesseract_collision_box_box_example box_box_example.cpp)
target_link_libraries(
  tesseract_collision_box_box_example
  collision_bullet
  collision_fcl
  tesseract::geometry
  console_bridge::console_bridge
  ${Boost_LIBRARIES}
  octomap
  octomath)
target_compile_options(tesseract_collision_box_box_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                   ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_cxx_version(tesseract_collision_box_box_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(tesseract_collision_box_box_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_compile_definitions(tesseract_collision_box_box_example PRIVATE DATA_DIR="${CMAKE_SOURCE_DIR}/test")

configure_component(
  COMPONENT collision_examples
  NAMESPACE tesseract
  TARGETS tesseract_collision_box_box_example
  DEPENDENCIES "tesseract COMPONENTS collision collision_bullet collision_fcl")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT collision_examples
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Collision Examples components"
    COMPONENT_DEPENDS collision collision_bullet collision_fcl)
endif()
