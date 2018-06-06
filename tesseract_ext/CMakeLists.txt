cmake_minimum_required(VERSION 2.8.3)
project(tesseract_ext)

add_compile_options(-std=c++11 -w)

find_package(catkin REQUIRED COMPONENTS)

set(BUILD_PYBULLET, ON)
set(BUILD_PYBULLET_NUMPY, OFF)
set(USE_DOUBLE_PRECISION, ON)

catkin_package(
  INCLUDE_DIRS bullet3/src bullet3/Extras
  LIBRARIES BulletCollision BulletDynamics LinearMath ConvexDecomposition
)

add_subdirectory(bullet3)

