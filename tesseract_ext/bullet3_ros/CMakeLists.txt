cmake_minimum_required(VERSION 2.4.3)
project(bullet3_ros)

add_compile_options(-std=c++11 -w)

find_package(catkin REQUIRED COMPONENTS)

# This causes accuracy issues in the EPA algorithm and need to create an issue.
#set(BUILD_PYBULLET ON CACHE BOOL "Enable Bullet Python")
#set(BUILD_PYBULLET_NUMPY OFF CACHE BOOL "Disable Bullet Python Numpy")
#set(USE_DOUBLE_PRECISION ON CACHE BOOL "Enable Bullet Double Precision")

set(BUILD_UNIT_TESTS OFF CACHE BOOL "Disable Bullet Unit Tests" FORCE)

file(MAKE_DIRECTORY "${CMAKE_INSTALL_PREFIX}/include/bullet")
catkin_package(
  INCLUDE_DIRS bullet3/src bullet3/Extras ${CMAKE_INSTALL_PREFIX}/include/bullet
  LIBRARIES BulletCollision BulletDynamics LinearMath ConvexDecomposition HACD
)

add_subdirectory(bullet3)
