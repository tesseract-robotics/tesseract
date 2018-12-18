cmake_minimum_required(VERSION 2.8.3)
project(tesseract_scene_graph)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(Eigen3 REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED)

include_directories(
  include
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION include/${PROJECT_NAME})

#if (CATKIN_ENABLE_TESTING)

#  find_package(gtest REQUIRED)

#  include_directories(${GTEST_INCLUDE_DIRS})

  add_executable(${PROJECT_NAME}_unit test/tesseract_scene_graph_unit.cpp)
  target_link_libraries(${PROJECT_NAME}_unit ${Boost_LIBRARIES}) #${GTEST_BOTH_LIBRARIES}

#endif()
