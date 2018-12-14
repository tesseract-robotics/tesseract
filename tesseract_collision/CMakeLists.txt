cmake_minimum_required(VERSION 2.8.3)
project(tesseract_collision)

add_compile_options(-std=c++11 -Wall -Wextra)

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
find_package(PkgConfig REQUIRED)
find_package(ccd REQUIRED)

pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")
list(APPEND LIBFCL_LIBRARIES ${CCD_LIBRARIES})

catkin_package(
  INCLUDE_DIRS
    include
    ${CCD_INCLUDE_DIRS}
    ${LIBFCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_bullet
    ${PROJECT_NAME}_fcl
    ${OCTOMAP_LIBRARIES}
    ${LIBFCL_LIBRARIES}
  CATKIN_DEPENDS
    bullet3_ros
    roscpp
    geometric_shapes
    tesseract_core
    pluginlib
  DEPENDS
    EIGEN3
    Boost
    OCTOMAP
    LIBFCL
    ccd
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBFCL_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${OCTOMAP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_bullet
  src/bullet/bullet_cast_bvh_manager.cpp
  src/bullet/bullet_cast_simple_manager.cpp
  src/bullet/bullet_discrete_bvh_manager.cpp
  src/bullet/bullet_discrete_simple_manager.cpp
  src/bullet/bullet_utils.cpp
)

target_link_libraries(${PROJECT_NAME}_bullet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})
target_compile_options(${PROJECT_NAME}_bullet PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

add_library(${PROJECT_NAME}_bullet_plugin src/bullet/bullet_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_bullet_plugin ${PROJECT_NAME}_bullet ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_bullet_plugin PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

add_library(${PROJECT_NAME}_fcl
  src/fcl/fcl_discrete_managers.cpp
  src/fcl/fcl_utils.cpp
)

target_link_libraries(${PROJECT_NAME}_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES})
target_compile_options(${PROJECT_NAME}_fcl PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

add_library(${PROJECT_NAME}_fcl_plugin src/fcl/fcl_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_fcl_plugin ${PROJECT_NAME}_fcl ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_fcl_plugin PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

add_executable(create_convex_hull src/create_convex_hull.cpp)
target_link_libraries(create_convex_hull ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})
target_compile_options(create_convex_hull PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${PROJECT_NAME}_bullet_plugin ${PROJECT_NAME}_fcl_plugin create_convex_hull
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

install(FILES tesseract_collision_bullet_plugin_description.xml tesseract_collision_fcl_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
  target_link_libraries(${PROJECT_NAME}_box_sphere_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${GTEST_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS})
  target_compile_options(${PROJECT_NAME}_box_sphere_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
  catkin_add_gtest(${PROJECT_NAME}_box_cylinder_unit test/collision_box_cylinder_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_box_cylinder_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS})

  catkin_add_gtest(${PROJECT_NAME}_box_cone_unit test/collision_box_cone_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_box_cone_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS})
  target_compile_options(${PROJECT_NAME}_box_cone_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  catkin_add_gtest(${PROJECT_NAME}_box_box_unit test/collision_box_box_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_box_box_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS})
  target_compile_options(${PROJECT_NAME}_box_box_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  catkin_add_gtest(${PROJECT_NAME}_large_dataset_unit test/collision_large_dataset_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_large_dataset_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS})
  target_compile_options(${PROJECT_NAME}_large_dataset_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  catkin_add_gtest(${PROJECT_NAME}_sphere_sphere_unit test/collision_sphere_sphere_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_sphere_sphere_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS})
  target_compile_options(${PROJECT_NAME}_sphere_sphere_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  catkin_add_gtest(${PROJECT_NAME}_mesh_mesh_unit test/collision_mesh_mesh_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_mesh_mesh_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS})
  target_compile_options(${PROJECT_NAME}_mesh_mesh_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  catkin_add_gtest(${PROJECT_NAME}_multi_threaded_unit test/collision_multi_threaded_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_multi_threaded_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS})
  target_compile_options(${PROJECT_NAME}_multi_threaded_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  catkin_add_gtest(${PROJECT_NAME}_octomap_sphere_unit test/collision_octomap_sphere_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_octomap_sphere_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS})
  target_compile_options(${PROJECT_NAME}_octomap_sphere_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  catkin_add_gtest(${PROJECT_NAME}_clone_unit test/collision_clone_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_clone_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS})
  target_compile_options(${PROJECT_NAME}_clone_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  catkin_add_gtest(${PROJECT_NAME}_box_box_cast_unit test/collision_box_box_cast_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_box_box_cast_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS})
  target_compile_options(${PROJECT_NAME}_box_box_cast_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

  catkin_add_gtest(${PROJECT_NAME}_compound_compound_unit test/collision_compound_compound_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_compound_compound_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS})
  target_compile_options(${PROJECT_NAME}_compound_compound_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

#  catkin_add_gtest(${PROJECT_NAME}_convex_concave_unit test/convex_concave_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_convex_concave_unit ${PROJECT_NAME}_bullet ${PROJECT_NAME}_fcl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_INCLUDE_DIRS})
endif()
