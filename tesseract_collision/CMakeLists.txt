cmake_minimum_required(VERSION 2.8.3)
project(tesseract_collision)

add_compile_options(-std=c++11 -Wall -Wextra)
add_definitions(-DBT_USE_DOUBLE_PRECISION)

find_package(catkin REQUIRED COMPONENTS
  bullet3_ros
  roscpp
  geometric_shapes
  tesseract_core
  eigen_conversions
  pluginlib
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}_bullet
    ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS
    bullet3_ros
    roscpp
    geometric_shapes
    tesseract_core
    eigen_conversions
    pluginlib
  DEPENDS
    EIGEN3
    Boost
    OCTOMAP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${OCTOMAP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_bullet
  src/bullet/bullet_cast_managers.cpp
  src/bullet/bullet_discrete_managers.cpp
  src/bullet/bullet_utils.cpp
)

target_link_libraries(${PROJECT_NAME}_bullet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_library(${PROJECT_NAME}_bullet_plugin src/bullet/bullet_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_bullet_plugin ${PROJECT_NAME}_bullet ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_bullet
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

if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  find_package(OpenMP REQUIRED)
  if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  else(OPENMP_FOUND)
    message (STATUS "Not found OpenMP")
  endif()

  include_directories(
      ${catkin_INCLUDE_DIRS}
  )

  catkin_add_gtest(${PROJECT_NAME}_box_sphere_unit test/collision_box_sphere_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_box_sphere_unit ${PROJECT_NAME}_bullet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

  catkin_add_gtest(${PROJECT_NAME}_box_cylinder_unit test/collision_box_cylinder_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_box_cylinder_unit ${PROJECT_NAME}_bullet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

  catkin_add_gtest(${PROJECT_NAME}_box_cone_unit test/collision_box_cone_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_box_cone_unit ${PROJECT_NAME}_bullet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

  catkin_add_gtest(${PROJECT_NAME}_box_box_unit test/collision_box_box_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_box_box_unit ${PROJECT_NAME}_bullet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

  catkin_add_gtest(${PROJECT_NAME}_large_dataset_unit test/collision_large_dataset_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_large_dataset_unit ${PROJECT_NAME}_bullet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

  catkin_add_gtest(${PROJECT_NAME}_sphere_sphere_unit test/collision_sphere_sphere_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_sphere_sphere_unit ${PROJECT_NAME}_bullet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

  catkin_add_gtest(${PROJECT_NAME}_multi_threaded_unit test/collision_multi_threaded_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_multi_threaded_unit ${PROJECT_NAME}_bullet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})

#  catkin_add_gtest(${PROJECT_NAME}_convex_concave_unit test/convex_concave_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_convex_concave_unit ${PROJECT_NAME}_bullet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})
endif()
