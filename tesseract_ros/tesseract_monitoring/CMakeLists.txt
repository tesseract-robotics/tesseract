cmake_minimum_required(VERSION 3.5.0)
project(tesseract_monitoring)

find_package(tesseract_collision REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(orocos_kdl REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tesseract_msgs
  tesseract_rosutils
  dynamic_reconfigure
  pluginlib
)

generate_dynamic_reconfigure_options(
  cfg/EnvironmentMonitorDynamicReconfigure.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_environment
  CATKIN_DEPENDS
    roscpp
    tesseract_msgs
    tesseract_rosutils
    pluginlib
    dynamic_reconfigure
  DEPENDS
    EIGEN3
    tesseract_collision
    tesseract_environment
    tesseract_kinematics
    tesseract_scene_graph
    tesseract_geometry
    orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Tesseract ROS Nodes
add_executable(${PROJECT_NAME}_contacts_node src/contact_monitor.cpp)
target_link_libraries(${PROJECT_NAME}_contacts_node PRIVATE tesseract_environment_kdl tesseract_scene_graph ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_contacts_node PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(${PROJECT_NAME}_contacts_node PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}_contacts_node PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_contacts_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_environment SHARED src/environment_monitor.cpp src/current_state_monitor.cpp)
target_link_libraries(${PROJECT_NAME}_environment PUBLIC tesseract_kinematics_core tesseract_kinematics_kdl tesseract_environment_kdl tesseract_collision_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME}_environment PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(${PROJECT_NAME}_environment PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}_environment PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_environment SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME}_environment ${PROJECT_NAME}_gencfg)

add_executable(demo_scene demos/demo_scene.cpp)
target_link_libraries(demo_scene PRIVATE ${PROJECT_NAME}_environment ${catkin_LIBRARIES})
target_compile_options(demo_scene PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(demo_scene PRIVATE cxx_std_11)
target_include_directories(demo_scene PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(demo_scene SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_environment_node src/environment_monitor_node.cpp)
target_link_libraries(${PROJECT_NAME}_environment_node PRIVATE ${PROJECT_NAME}_environment ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_environment_node PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(${PROJECT_NAME}_environment_node PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}_environment_node PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_environment_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_environment ${PROJECT_NAME}_contacts_node ${PROJECT_NAME}_environment_node demo_scene
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

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
