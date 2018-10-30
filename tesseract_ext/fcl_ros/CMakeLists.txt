cmake_minimum_required(VERSION 2.8.12)

add_compile_options(-std=c++11 -w)

set(FCL_BUILD_TESTS OFF CACHE BOOL "Disable FCL tests")
add_subdirectory(fcl)

install(FILES package.xml DESTINATION share/fcl_ros)
