cmake_minimum_required(VERSION 2.8.3)
project(tesseract_planning)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tesseract_ros
  trajopt
)

find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${OMPL_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_ompl
    ${PROJECT_NAME}_trajopt
  CATKIN_DEPENDS
    roscpp
    tesseract_ros
    trajopt
  DEPENDS
    ompl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${OMPL_INCLUDE_DIRS}
)

# Trajopt Planner
add_library(${PROJECT_NAME}_trajopt src/trajopt/trajopt_planner.cpp)
target_link_libraries(${PROJECT_NAME}_trajopt ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_trajopt PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

# OMPL Planning Interface
add_library(${PROJECT_NAME}_ompl
  src/ompl/chain_ompl_interface.cpp
  src/ompl/conversions.cpp
  src/ompl/continuous_motion_validator.cpp
)
add_dependencies(${PROJECT_NAME}_ompl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_ompl ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
target_compile_options(${PROJECT_NAME}_ompl PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

# OMPL Planning Test/Example Program
add_executable(${PROJECT_NAME}_ompl_test src/ompl_planner_tests.cpp)
add_dependencies(${PROJECT_NAME}_ompl_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_ompl_test ${catkin_LIBRARIES} ${PROJECT_NAME}_ompl)
target_compile_options(${PROJECT_NAME}_ompl_test PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_ompl ${PROJECT_NAME}_trajopt
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
