cmake_minimum_required(VERSION 2.4.3)

add_compile_options(-std=c++11 -w)

# This causes accuracy issues in the EPA algorithm and need to create an issue.
#set(BUILD_PYBULLET ON CACHE BOOL "Enable Bullet Python")
#set(BUILD_PYBULLET_NUMPY OFF CACHE BOOL "Disable Bullet Python Numpy")
#set(USE_DOUBLE_PRECISION ON CACHE BOOL "Enable Bullet Double Precision")

set(BUILD_UNIT_TESTS OFF CACHE BOOL "Disable Bullet Unit Tests" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "Enable Bullet Shared Libs" FORCE)

add_subdirectory(bullet3)

install(FILES package.xml DESTINATION share/bullet3)
