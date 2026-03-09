add_executable(tesseract_common_basic_property_tree_example property_tree_basic_example.cpp)
target_link_libraries(tesseract_common_basic_property_tree_example PRIVATE tesseract::common)
target_compile_options(tesseract_common_basic_property_tree_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                            ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_cxx_version(tesseract_common_basic_property_tree_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(tesseract_common_basic_property_tree_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})

add_executable(tesseract_common_advanced_property_tree_example property_tree_advanced_example.cpp)
target_link_libraries(tesseract_common_advanced_property_tree_example PRIVATE tesseract::common)
target_compile_options(tesseract_common_advanced_property_tree_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                               ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_cxx_version(tesseract_common_advanced_property_tree_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(tesseract_common_advanced_property_tree_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})

add_executable(tesseract_common_oneof_property_tree_example property_tree_oneof_example.cpp)
target_link_libraries(tesseract_common_oneof_property_tree_example PRIVATE tesseract::common)
target_compile_options(tesseract_common_oneof_property_tree_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                            ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_cxx_version(tesseract_common_oneof_property_tree_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(tesseract_common_oneof_property_tree_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})

add_executable(tesseract_common_polymorphic_property_tree_example property_tree_polymorphic_example.cpp)
target_link_libraries(tesseract_common_polymorphic_property_tree_example PRIVATE tesseract::common)
target_compile_options(tesseract_common_polymorphic_property_tree_example PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                                                  ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_cxx_version(tesseract_common_polymorphic_property_tree_example PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(tesseract_common_polymorphic_property_tree_example ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})

configure_component(
  COMPONENT common_examples
  NAMESPACE tesseract
  TARGETS tesseract_common_basic_property_tree_example
          tesseract_common_advanced_property_tree_example
          tesseract_common_oneof_property_tree_example
          tesseract_common_polymorphic_property_tree_example
  DEPENDENCIES "tesseract COMPONENTS common")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT common_examples
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Common PropertyTree Examples"
    COMPONENT_DEPENDS common)
endif()
