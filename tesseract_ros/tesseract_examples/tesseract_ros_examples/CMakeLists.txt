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

add_executable(${PROJECT_NAME}_basic_cartesian_plan src/basic_cartesian_plan.cpp)
target_link_libraries(${PROJECT_NAME}_basic_cartesian_plan tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_basic_cartesian_plan PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_basic_cartesian_plan PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_basic_cartesian_plan PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_basic_cartesian_plan PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_basic_cartesian_plan SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_glass_up_right_plan src/glass_up_right_plan.cpp)
target_link_libraries(${PROJECT_NAME}_glass_up_right_plan tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_glass_up_right_plan PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_glass_up_right_plan PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_glass_up_right_plan PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_glass_up_right_plan PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_glass_up_right_plan SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_puzzle_piece_plan src/puzzle_piece_plan.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_plan tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_puzzle_piece_plan PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_puzzle_piece_plan PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_puzzle_piece_plan PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_puzzle_piece_plan PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_plan SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_car_seat_demo src/car_seat_demo.cpp)
target_link_libraries(${PROJECT_NAME}_car_seat_demo tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_car_seat_demo PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_car_seat_demo PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_car_seat_demo PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_car_seat_demo PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_car_seat_demo SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_puzzle_piece_auxillary_axes_plan src/puzzle_piece_auxillary_axes_plan.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_plan tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_plan PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_plan PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_puzzle_piece_auxillary_axes_plan PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_plan PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_plan SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_pick_and_place_plan src/pick_and_place_plan.cpp)
target_link_libraries(${PROJECT_NAME}_pick_and_place_plan tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_pick_and_place_plan PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_pick_and_place_plan PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_pick_and_place_plan PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_pick_and_place_plan PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_pick_and_place_plan SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}_basic_cartesian_plan ${PROJECT_NAME}_glass_up_right_plan ${PROJECT_NAME}_puzzle_piece_plan ${PROJECT_NAME}_car_seat_demo ${PROJECT_NAME}_pick_and_place_plan ${PROJECT_NAME}_puzzle_piece_auxillary_axes_plan
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

