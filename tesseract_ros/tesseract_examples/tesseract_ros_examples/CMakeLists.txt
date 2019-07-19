cmake_minimum_required(VERSION 2.8.3)
project(tesseract_ros_examples)

add_compile_options(-std=c++11 -Wall -Wextra)

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
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

catkin_package()

add_library(${PROJECT_NAME}_basic_cartesian_example src/basic_cartesian_example.cpp)
target_link_libraries(${PROJECT_NAME}_basic_cartesian_example tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_basic_cartesian_example PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_basic_cartesian_example PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_basic_cartesian_example PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_basic_cartesian_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_basic_cartesian_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_basic_cartesian_example_node src/basic_cartesian_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_basic_cartesian_example_node ${PROJECT_NAME}_basic_cartesian_example ${catkin_LIBRARIES})
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_basic_cartesian_example_node PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_basic_cartesian_example_node PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_basic_cartesian_example_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_basic_cartesian_example_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_glass_up_right_example src/glass_up_right_example.cpp)
target_link_libraries(${PROJECT_NAME}_glass_up_right_example tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_glass_up_right_example PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_glass_up_right_example PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_glass_up_right_example PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_glass_up_right_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_glass_up_right_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_glass_up_right_example_node src/glass_up_right_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_glass_up_right_example_node ${PROJECT_NAME}_glass_up_right_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_glass_up_right_example_node PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_glass_up_right_example_node PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_glass_up_right_example_node PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_glass_up_right_example_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_glass_up_right_example_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_puzzle_piece_example src/puzzle_piece_example.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_example tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_puzzle_piece_example PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_puzzle_piece_example PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_puzzle_piece_example PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_puzzle_piece_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_puzzle_piece_example_node src/puzzle_piece_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_example_node ${PROJECT_NAME}_puzzle_piece_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_puzzle_piece_example_node PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_puzzle_piece_example_node PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_puzzle_piece_example_node PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_puzzle_piece_example_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_example_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_car_seat_example src/car_seat_example.cpp)
target_link_libraries(${PROJECT_NAME}_car_seat_example tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_car_seat_example PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_car_seat_example PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_car_seat_example PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_car_seat_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_car_seat_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_car_seat_example_node src/car_seat_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_car_seat_example_node ${PROJECT_NAME}_car_seat_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_car_seat_example_node PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_car_seat_example_node PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_car_seat_example_node PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_car_seat_example_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_car_seat_example_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example src/puzzle_piece_auxillary_axes_example.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node src/puzzle_piece_auxillary_axes_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_pick_and_place_example src/pick_and_place_example.cpp)
target_link_libraries(${PROJECT_NAME}_pick_and_place_example tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_pick_and_place_example PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_pick_and_place_example PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_pick_and_place_example PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_pick_and_place_example PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_pick_and_place_example SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_pick_and_place_example_node src/pick_and_place_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_pick_and_place_example_node ${PROJECT_NAME}_pick_and_place_example ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_pick_and_place_example_node PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_pick_and_place_example_node PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_pick_and_place_example_node PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_pick_and_place_example_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_pick_and_place_example_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

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

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(GTest REQUIRED)

    add_rostest_gtest(${PROJECT_NAME}_basic_cartesian_example_unit test/basic_cartesian_example_unit.test test/basic_cartesian_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_basic_cartesian_example_unit ${PROJECT_NAME}_basic_cartesian_example ${GTEST_BOTH_LIBRARIES} ${catkin_LIBRARIES})
    if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE -std=c++11)
    else()
        target_compile_features(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE cxx_std_11)
    endif()
    target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit SYSTEM PUBLIC
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${GTEST_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_car_seat_example_unit test/car_seat_example_unit.test test/car_seat_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_car_seat_example_unit ${PROJECT_NAME}_car_seat_example ${GTEST_BOTH_LIBRARIES} ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_car_seat_example_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
    if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options(${PROJECT_NAME}_car_seat_example_unit PRIVATE -std=c++11)
    else()
        target_compile_features(${PROJECT_NAME}_car_seat_example_unit PRIVATE cxx_std_11)
    endif()
    target_include_directories(${PROJECT_NAME}_car_seat_example_unit PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_car_seat_example_unit SYSTEM PUBLIC
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_glass_up_right_example_unit test/glass_up_right_example_unit.test test/glass_up_right_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_glass_up_right_example_unit ${PROJECT_NAME}_glass_up_right_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_glass_up_right_example_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
    if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options(${PROJECT_NAME}_glass_up_right_example_unit PRIVATE -std=c++11)
    else()
        target_compile_features(${PROJECT_NAME}_glass_up_right_example_unit PRIVATE cxx_std_11)
    endif()
    target_include_directories(${PROJECT_NAME}_glass_up_right_example_unit PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_glass_up_right_example_unit SYSTEM PUBLIC
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_puzzle_piece_example_unit test/puzzle_piece_example_unit.test test/puzzle_piece_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_puzzle_piece_example_unit ${PROJECT_NAME}_puzzle_piece_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
    if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE -std=c++11)
    else()
        target_compile_features(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE cxx_std_11)
    endif()
    target_include_directories(${PROJECT_NAME}_puzzle_piece_example_unit PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_puzzle_piece_example_unit SYSTEM PUBLIC
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit test/puzzle_piece_auxillary_axes_example_unit.test test/puzzle_piece_auxillary_axes_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
    if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit PRIVATE -std=c++11)
    else()
        target_compile_features(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit PRIVATE cxx_std_11)
    endif()
    target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit SYSTEM PUBLIC
        ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(${PROJECT_NAME}_pick_and_place_example_unit test/pick_and_place_example_unit.test test/pick_and_place_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_pick_and_place_example_unit ${PROJECT_NAME}_pick_and_place_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
    if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE -std=c++11)
    else()
        target_compile_features(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE cxx_std_11)
    endif()
    target_include_directories(${PROJECT_NAME}_pick_and_place_example_unit PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_pick_and_place_example_unit SYSTEM PUBLIC
        ${catkin_INCLUDE_DIRS})
endif()

