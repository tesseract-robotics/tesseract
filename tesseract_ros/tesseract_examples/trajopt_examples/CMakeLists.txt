cmake_minimum_required(VERSION 2.8.3)
project(trajopt_examples)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tesseract_rosutils
  octomap_ros
  pcl_conversions
)

find_package(trajopt REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_planning REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

catkin_package()

add_executable(${PROJECT_NAME}_basic_cartesian_plan src/basic_cartesian_plan.cpp)
target_link_libraries(${PROJECT_NAME}_basic_cartesian_plan tesseract_environment_kdl tesseract_kinematics_kdl tesseract_collision_bullet tesseract_scene_graph tesseract_planning_trajopt ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_basic_cartesian_plan PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(${PROJECT_NAME}_basic_cartesian_plan PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}_basic_cartesian_plan PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_basic_cartesian_plan SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_glass_up_right_plan src/glass_up_right_plan.cpp)
target_link_libraries(${PROJECT_NAME}_glass_up_right_plan tesseract_environment_kdl tesseract_kinematics_kdl tesseract_collision_bullet tesseract_scene_graph tesseract_planning_trajopt ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_glass_up_right_plan PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(${PROJECT_NAME}_glass_up_right_plan PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}_glass_up_right_plan PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_glass_up_right_plan SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_puzzle_piece_plan src/puzzle_piece_plan.cpp)
target_link_libraries(${PROJECT_NAME}_puzzle_piece_plan tesseract_environment_kdl tesseract_kinematics_kdl tesseract_collision_bullet tesseract_scene_graph tesseract_planning_trajopt ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_puzzle_piece_plan PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(${PROJECT_NAME}_puzzle_piece_plan PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}_puzzle_piece_plan PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_puzzle_piece_plan SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

#add_executable(${PROJECT_NAME}_car_seat_demo src/car_seat_demo.cpp)
#target_link_libraries(${PROJECT_NAME}_car_seat_demo ${catkin_LIBRARIES})
#target_compile_options(${PROJECT_NAME}_car_seat_demo PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

#add_executable(${PROJECT_NAME}_puzzle_piece_auxillary_axes_plan src/puzzle_piece_auxillary_axes_plan.cpp)
#target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_plan ${catkin_LIBRARIES})
#target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_plan PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

#add_executable(${PROJECT_NAME}_pick_and_place_plan src/pick_and_place_plan.cpp)
#target_link_libraries(${PROJECT_NAME}_pick_and_place_plan ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}_basic_cartesian_plan ${PROJECT_NAME}_glass_up_right_plan ${PROJECT_NAME}_puzzle_piece_plan #${PROJECT_NAME}_car_seat_demo ${PROJECT_NAME}_puzzle_piece_auxillary_axes_plan
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

