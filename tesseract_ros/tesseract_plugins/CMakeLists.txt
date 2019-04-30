cmake_minimum_required(VERSION 3.5.0)
project(tesseract_plugins)

find_package(catkin REQUIRED COMPONENTS
  tesseract_msgs
  roscpp
  eigen_conversions
  pluginlib
)

find_package(Eigen3 REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_geometry REQUIRED) # This should not be required, must be doing something wrong when creating targets

catkin_package(
  INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS
    tesseract_msgs
    roscpp
    eigen_conversions
    pluginlib
  DEPENDS
    EIGEN3
    tesseract_collision
)

add_library(tesseract_collision_bullet_plugin src/plugins/bullet_plugin.cpp)
target_link_libraries(tesseract_collision_bullet_plugin PRIVATE tesseract_collision_core tesseract_collision_bullet Eigen3::Eigen ${catkin_LIBRARIES})
target_compile_options(tesseract_collision_bullet_plugin PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(tesseract_collision_bullet_plugin PRIVATE cxx_std_11)
target_include_directories(tesseract_collision_bullet_plugin SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})


add_library(tesseract_collision_fcl_plugin src/plugins/fcl_plugin.cpp)
target_link_libraries(tesseract_collision_fcl_plugin PRIVATE tesseract_collision_core tesseract_collision_fcl ${catkin_LIBRARIES})
target_compile_options(tesseract_collision_fcl_plugin PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(tesseract_collision_bullet_plugin PRIVATE cxx_std_11)
target_include_directories(tesseract_collision_bullet_plugin SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

# Mark executables and/or libraries for installation
install(TARGETS tesseract_collision_bullet_plugin tesseract_collision_fcl_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
# )

install(FILES tesseract_collision_bullet_plugin_description.xml tesseract_collision_fcl_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
