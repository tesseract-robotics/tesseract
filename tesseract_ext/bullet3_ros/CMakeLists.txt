cmake_minimum_required(VERSION 2.4.3)
project(bullet3_ros)

add_compile_options(-std=c++11 -w)

find_package(catkin REQUIRED COMPONENTS)

set(BUILD_PYBULLET ON CACHE BOOL "Enable Bullet Python")
set(BUILD_PYBULLET_NUMPY OFF CACHE BOOL "Disable Bullet Python Numpy")
set(USE_DOUBLE_PRECISION ON CACHE BOOL "Enable Bullet Double Precision")

catkin_package(
  INCLUDE_DIRS bullet3/src bullet3/Extras
  LIBRARIES BulletCollision BulletDynamics LinearMath ConvexDecomposition
)

add_subdirectory(bullet3)
