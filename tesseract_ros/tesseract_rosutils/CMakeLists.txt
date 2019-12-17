cmake_minimum_required(VERSION 3.5.0)
project(tesseract_rosutils)

find_package(catkin REQUIRED COMPONENTS
  tesseract_msgs
  roscpp
  eigen_conversions
  geometry_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_geometry REQUIRED) # This should not be required, must be doing something wrong when creating targets
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_process_planners REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
#  LIBRARIES
  CATKIN_DEPENDS
    tesseract_msgs
    roscpp
    eigen_conversions
    roslib
    geometry_msgs
    std_msgs
  DEPENDS
    EIGEN3
    tesseract_scene_graph
    tesseract_geometry
    tesseract_visualization
    tesseract_collision
    tesseract_common
    tesseract_process_planners
    tesseract_motion_planners
)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE tesseract::tesseract_scene_graph tesseract::tesseract_geometry tesseract::tesseract_collision_core tesseract::tesseract_visualization tesseract::tesseract_common tesseract::tesseract_process_planners tesseract::tesseract_motion_planners_core ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME} INTERFACE)
tesseract_clang_tidy(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

if(CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)
  catkin_add_gtest(${PROJECT_NAME}_unit test/tesseract_rosutils_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_unit GTest::GTest GTest::Main tesseract::tesseract_scene_graph tesseract::tesseract_geometry tesseract::tesseract_collision_core tesseract::tesseract_visualization tesseract::tesseract_common tesseract::tesseract_process_planners tesseract::tesseract_motion_planners_core ${catkin_LIBRARIES})
  tesseract_target_compile_options(${PROJECT_NAME}_unit PRIVATE)
  tesseract_clang_tidy(${PROJECT_NAME}_unit)
  target_include_directories(${PROJECT_NAME}_unit PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
  target_include_directories(${PROJECT_NAME}_unit SYSTEM PRIVATE
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS})
endif()
