cmake_minimum_required(VERSION 2.8.3)
project(tesseract_monitoring)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tesseract_ros
  tesseract_collision
  tesseract_msgs
  tesseract_core
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/EnvironmentMonitorDynamicReconfigure.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_environment
  CATKIN_DEPENDS
    roscpp
    tesseract_ros
    tesseract_collision
    tesseract_msgs
    tesseract_core
    dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Tesseract ROS Nodes
add_executable(${PROJECT_NAME}_contacts_node src/contact_monitor.cpp)
target_link_libraries(${PROJECT_NAME}_contacts_node ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_contacts_node PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

add_library(${PROJECT_NAME}_environment src/environment_monitor.cpp src/current_state_monitor.cpp)
target_link_libraries(${PROJECT_NAME}_environment ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_options(${PROJECT_NAME}_environment PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
add_dependencies(${PROJECT_NAME}_environment ${PROJECT_NAME}_gencfg)

add_executable(demo_scene demos/demo_scene.cpp)
target_link_libraries(demo_scene ${PROJECT_NAME}_environment ${catkin_LIBRARIES})
target_compile_options(demo_scene PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

add_executable(${PROJECT_NAME}_environment_node src/environment_monitor_node.cpp)
target_link_libraries(${PROJECT_NAME}_environment_node ${PROJECT_NAME}_environment ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_environment_node PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

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
