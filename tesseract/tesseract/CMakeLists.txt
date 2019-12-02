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

# Create target
add_library(${PROJECT_NAME} SHARED src/tesseract.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract::tesseract_urdf tesseract::tesseract_scene_graph tesseract::tesseract_collision_bullet tesseract::tesseract_environment_kdl tesseract::tesseract_kinematics_kdl console_bridge)
tesseract_target_compile_options(${PROJECT_NAME} PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${Boost_INCLUDE_DIRS}
    $<TARGET_PROPERTY:tesseract::tesseract_common,INTERFACE_INCLUDE_DIRECTORIES>) #tesseract::tesseract_common Due to bug in catkin, there is an open PR

tesseract_configure_package(${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

#option(ENABLE_TESTS "Enable tests" OFF)
#if (${ENABLE_TESTS})
#  enable_testing()
#  add_custom_target(run_tests ALL
#      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
#      COMMAND ${CMAKE_CTEST_COMMAND} -V -O "/tmp/${PROJECT_NAME}_ctest.log" -C $<CONFIGURATION>)

#  add_subdirectory(test)
#endif()
