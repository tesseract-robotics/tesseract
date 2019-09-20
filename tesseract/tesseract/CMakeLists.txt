cmake_minimum_required(VERSION 3.5.0)
project(tesseract VERSION 0.1.0 LANGUAGES CXX)

find_package(Boost COMPONENTS system REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_urdf REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

# Create target
add_library(${PROJECT_NAME} SHARED src/tesseract.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract::tesseract_urdf tesseract::tesseract_scene_graph tesseract::tesseract_collision_bullet tesseract::tesseract_environment_kdl tesseract::tesseract_kinematics_kdl console_bridge)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
endif()
target_compile_options(${PROJECT_NAME} PUBLIC -mno-avx)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${Boost_INCLUDE_DIRS}
    $<TARGET_PROPERTY:tesseract::tesseract_common,INTERFACE_INCLUDE_DIRECTORIES>) #tesseract::tesseract_common Due to bug in catkin, there is an open PR

list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

# Mark executables and/or libraries for installation
install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE tesseract:: DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

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

#option(ENABLE_TESTS "Enable tests" OFF)
#if (${ENABLE_TESTS})
#  enable_testing()
#  add_custom_target(run_tests ALL
#      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
#      COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> -V)

#  add_subdirectory(test)
#endif()
