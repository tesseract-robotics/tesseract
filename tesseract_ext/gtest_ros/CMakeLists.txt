cmake_minimum_required(VERSION 2.4.3)

add_compile_options(-std=c++11 -w)

set(BUILD_SHARED_LIBS ON CACHE BOOL "Enable Bullet Shared Libs" FORCE)

add_subdirectory(gtest)

install(FILES package.xml DESTINATION share/gtest)
