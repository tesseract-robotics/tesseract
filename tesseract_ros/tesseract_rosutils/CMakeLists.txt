cmake_minimum_required(VERSION 3.5.0)
project(tesseract_rosutils)

find_package(catkin REQUIRED COMPONENTS
  tesseract_msgs
  roscpp
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_geometry REQUIRED) # This should not be required, must be doing something wrong when creating targets
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_common REQUIRED)
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
  DEPENDS
    EIGEN3
    tesseract_scene_graph
    tesseract_geometry
    tesseract_visualization
    tesseract_collision
    tesseract_common
    tesseract_process_planners
)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE tesseract::tesseract_scene_graph tesseract::tesseract_geometry tesseract::tesseract_collision_core tesseract::tesseract_visualization tesseract::tesseract_common tesseract::tesseract_process_planners ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME} INTERFACE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)
endif()
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
