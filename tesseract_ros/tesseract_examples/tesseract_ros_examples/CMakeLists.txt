cmake_minimum_required(VERSION 2.8.3)
project(tesseract_ros_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tesseract_rosutils
  octomap_ros
  pcl_conversions
)

find_package(trajopt REQUIRED)
find_package(tesseract REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

catkin_package()

add_library(${PROJECT_NAME}_basic_cartesian_example src/basic_cartesian_example.cpp)
target_link_libraries(${PROJECT_NAME}_basic_cartesian_example tesseract::tesseract tesseract::tesseract_motion_planners_trajopt jsoncpp_lib ${PCL_LIBRARIES} ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_basic_cartesian_example PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_basic_cartesian_example)
foreach(DEF ${PCL_DEFINITIONS})
  string(STRIP ${DEF} DEF)
  if (NOT "${DEF}" STREQUAL "")
    string(SUBSTRING "${DEF}" 0 2 DEF_PREFIX)
    if ("${DEF_PREFIX}" STREQUAL "-m")
      string(REPLACE " " ";" DEF ${DEF})
      foreach(OPTION_DEF ${DEF})
        target_compile_options(${PROJECT_NAME}_basic_cartesian_example PUBLIC ${OPTION_DEF})
      endforeach()
    else()
      target_compile_definitions(${PROJECT_NAME}_basic_cartesian_example PUBLIC ${DEF})
    endif()
  endif()
endforeach()
target_include_directories(${PROJECT_NAME}_basic_cartesian_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_basic_cartesian_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_basic_cartesian_example_node src/basic_cartesian_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_basic_cartesian_example_node ${PROJECT_NAME}_basic_cartesian_example ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_basic_cartesian_example_node PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_basic_cartesian_example_node)
target_include_directories(${PROJECT_NAME}_basic_cartesian_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_basic_cartesian_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_glass_up_right_example src/glass_up_right_example.cpp)
target_link_libraries(${PROJECT_NAME}_glass_up_right_example tesseract::tesseract tesseract::tesseract_motion_planners_trajopt jsoncpp_lib ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_glass_up_right_example PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_glass_up_right_example)
target_include_directories(${PROJECT_NAME}_glass_up_right_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_glass_up_right_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_glass_up_right_example_node src/glass_up_right_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_glass_up_right_example_node ${PROJECT_NAME}_glass_up_right_example ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_glass_up_right_example_node )
tesseract_clang_tidy(${PROJECT_NAME}_glass_up_right_example_node)
target_include_directories(${PROJECT_NAME}_glass_up_right_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_glass_up_right_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_puzzle_piece_example src/puzzle_piece_example.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_example tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_puzzle_piece_example PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_puzzle_piece_example)
target_include_directories(${PROJECT_NAME}_puzzle_piece_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_puzzle_piece_example_node src/puzzle_piece_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_example_node ${PROJECT_NAME}_puzzle_piece_example ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_puzzle_piece_example_node PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_puzzle_piece_example_node)
target_include_directories(${PROJECT_NAME}_puzzle_piece_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_car_seat_example src/car_seat_example.cpp)
target_link_libraries(${PROJECT_NAME}_car_seat_example tesseract::tesseract tesseract::tesseract_motion_planners_trajopt jsoncpp_lib ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_car_seat_example PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_car_seat_example)
target_include_directories(${PROJECT_NAME}_car_seat_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_car_seat_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_car_seat_example_node src/car_seat_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_car_seat_example_node ${PROJECT_NAME}_car_seat_example ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_car_seat_example_node PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_car_seat_example_node)
target_include_directories(${PROJECT_NAME}_car_seat_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_car_seat_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example src/puzzle_piece_auxillary_axes_example.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example)
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node src/puzzle_piece_auxillary_axes_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node)
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_pick_and_place_example src/pick_and_place_example.cpp)
target_link_libraries(${PROJECT_NAME}_pick_and_place_example tesseract::tesseract tesseract::tesseract_motion_planners_trajopt jsoncpp_lib ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_pick_and_place_example PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_pick_and_place_example)
target_include_directories(${PROJECT_NAME}_pick_and_place_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_pick_and_place_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_pick_and_place_example_node src/pick_and_place_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_pick_and_place_example_node ${PROJECT_NAME}_pick_and_place_example ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_pick_and_place_example_node PRIVATE)
tesseract_clang_tidy(${PROJECT_NAME}_pick_and_place_example_node)
target_include_directories(${PROJECT_NAME}_pick_and_place_example_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_pick_and_place_example_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}_basic_cartesian_example
    ${PROJECT_NAME}_basic_cartesian_example_node
    ${PROJECT_NAME}_glass_up_right_example
    ${PROJECT_NAME}_glass_up_right_example_node
    ${PROJECT_NAME}_puzzle_piece_example
    ${PROJECT_NAME}_puzzle_piece_example_node
    ${PROJECT_NAME}_car_seat_example
    ${PROJECT_NAME}_car_seat_example_node
    ${PROJECT_NAME}_pick_and_place_example
    ${PROJECT_NAME}_pick_and_place_example_node
    ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example
    ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.test"
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(GTest REQUIRED)

    add_rostest_gtest(${PROJECT_NAME}_basic_cartesian_example_unit test/basic_cartesian_example_unit.test test/basic_cartesian_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_basic_cartesian_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_basic_cartesian_example ${catkin_LIBRARIES})
    tesseract_target_compile_options(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE)
    tesseract_clang_tidy(${PROJECT_NAME}_basic_cartesian_example_unit)
    target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_car_seat_example_unit test/car_seat_example_unit.test test/car_seat_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_car_seat_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_car_seat_example ${catkin_LIBRARIES})
    tesseract_target_compile_options(${PROJECT_NAME}_car_seat_example_unit PRIVATE)
    tesseract_clang_tidy(${PROJECT_NAME}_car_seat_example_unit)
    target_include_directories(${PROJECT_NAME}_car_seat_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_car_seat_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_glass_up_right_example_unit test/glass_up_right_example_unit.test test/glass_up_right_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_glass_up_right_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_glass_up_right_example ${catkin_LIBRARIES})
    tesseract_target_compile_options(${PROJECT_NAME}_glass_up_right_example_unit PRIVATE)
    tesseract_clang_tidy(${PROJECT_NAME}_glass_up_right_example_unit)
    target_include_directories(${PROJECT_NAME}_glass_up_right_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_glass_up_right_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_puzzle_piece_example_unit test/puzzle_piece_example_unit.test test/puzzle_piece_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_puzzle_piece_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_puzzle_piece_example ${catkin_LIBRARIES})
    tesseract_target_compile_options(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE)
    tesseract_clang_tidy(${PROJECT_NAME}_puzzle_piece_example_unit)
    target_include_directories(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_puzzle_piece_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit test/puzzle_piece_auxillary_axes_example_unit.test test/puzzle_piece_auxillary_axes_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example ${catkin_LIBRARIES})
    tesseract_target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit PRIVATE)
    tesseract_clang_tidy(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit)
    target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_pick_and_place_example_unit test/pick_and_place_example_unit.test test/pick_and_place_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_pick_and_place_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_pick_and_place_example ${catkin_LIBRARIES})
    tesseract_target_compile_options(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE)
    tesseract_clang_tidy(${PROJECT_NAME}_pick_and_place_example_unit)
    target_include_directories(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_pick_and_place_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})
endif()

