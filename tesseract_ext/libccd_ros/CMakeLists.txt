cmake_minimum_required(VERSION 2.8.12)

add_compile_options(-std=c++11 -w)

set(ENABLE_DOUBLE_PRECISION ON CACHE BOOL "Enable Double Precision")
add_subdirectory(libccd)

configure_file(${CMAKE_BINARY_DIR}/libccd/ccd-config.cmake ${CMAKE_BINARY_DIR}/ccd-config.cmake COPYONLY)
configure_file(${CMAKE_BINARY_DIR}/libccd/ccd-config-version.cmake ${CMAKE_BINARY_DIR}/ccd-config-version.cmake COPYONLY)
configure_file(${CMAKE_BINARY_DIR}/libccd/ccd.pc ${CMAKE_BINARY_DIR}/ccd.pc COPYONLY)

install(FILES package.xml DESTINATION share/libccd_ros)
