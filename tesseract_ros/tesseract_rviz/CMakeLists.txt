cmake_minimum_required(VERSION 3.5.0)
project(tesseract_rviz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  tesseract_msgs
  tesseract_rosutils
  rviz
  pluginlib
  interactive_markers
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED thread date_time system filesystem)
find_package(tesseract REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(Eigen3 REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

# Ogre
find_package(PkgConfig REQUIRED)
pkg_check_modules(OGRE OGRE)
link_directories(${OGRE_LIBRARY_DIRS} )

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt4_wrap_cpp(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt5_wrap_cpp(${ARGN})
  endmacro()
endif()

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS
    include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${QT_INCLUDE_DIR}
    ${OGRE_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_render_tools
    ${PROJECT_NAME}_state_plugin_core
  CATKIN_DEPENDS
    roscpp
    message_generation
    tesseract_msgs
    tesseract_rosutils
    rviz
    pluginlib
    interactive_markers
  DEPENDS
    Boost
    OGRE
    QT
    tesseract
    tesseract_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${QT_INCLUDE_DIR}
  SYSTEM ${OGRE_INCLUDE_DIRS}
)

# Rviz Render Tools
qt_wrap_cpp(${PROJECT_NAME}_render_tools_cpp_MOCS
  include/tesseract_rviz/render_tools/trajectory_panel.h
  include/tesseract_rviz/render_tools/visualization_widget.h
  include/tesseract_rviz/render_tools/link_widget.h
  include/tesseract_rviz/render_tools/joint_widget.h
  include/tesseract_rviz/render_tools/joint_state_monitor_widget.h
  include/tesseract_rviz/render_tools/environment_widget.h
  include/tesseract_rviz/render_tools/trajectory_monitor_widget.h
  include/tesseract_rviz/render_tools/manipulation_widget.h
  include/tesseract_rviz/interactive_marker/integer_action.h
  include/tesseract_rviz/interactive_marker/interactive_marker.h
  include/tesseract_rviz/property/button_property.h
  )

add_library(${PROJECT_NAME}_render_tools SHARED
  src/render_tools/visualization_widget.cpp
  src/render_tools/joint_widget.cpp
  src/render_tools/link_widget.cpp
  src/render_tools/trajectory_panel.cpp
  src/render_tools/joint_state_monitor_widget.cpp
  src/render_tools/environment_widget.cpp
  src/render_tools/trajectory_monitor_widget.cpp
  src/render_tools/manipulation_widget.cpp
  src/interactive_marker/integer_action.cpp
  src/interactive_marker/interactive_marker.cpp
  src/interactive_marker/interactive_marker_control.cpp
  src/markers/marker_base.cpp
  src/markers/marker_selection_handler.cpp
  src/markers/utils.cpp
  src/markers/arrow_marker.cpp
  src/markers/sphere_marker.cpp
  src/markers/cube_marker.cpp
  src/markers/cylinder_marker.cpp
  src/markers/triangle_list_marker.cpp
  src/markers/text_view_facing_marker.cpp
  src/property/button_property.cpp
  ${${PROJECT_NAME}_render_tools_cpp_MOCS}
)

target_link_libraries(${PROJECT_NAME}_render_tools PUBLIC
  tesseract::tesseract
  ${catkin_LIBRARIES}
  ${OGRE_LIBRARIES}
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
)

tesseract_target_compile_options(${PROJECT_NAME}_render_tools PUBLIC)
target_include_directories(${PROJECT_NAME}_render_tools PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_render_tools SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${QT_INCLUDE_DIR}
    ${OGRE_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

#add_library(${PROJECT_NAME}_scene_plugin_core
#  src/tesseract_scene_plugin/tesseract_scene_display.cpp
#  include/tesseract_rviz/tesseract_scene_plugin/tesseract_scene_display.h)
#target_link_libraries(${PROJECT_NAME}_scene_plugin_core ${PROJECT_NAME}_render_tools ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

#add_library(${PROJECT_NAME}_scene_plugin src/tesseract_scene_plugin/plugin_init.cpp)
#target_link_libraries(${PROJECT_NAME}_scene_plugin ${PROJECT_NAME}_scene_plugin_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Trajectory State Disply
qt_wrap_cpp(${PROJECT_NAME}_state_plugin_core_cpp_MOCS include/tesseract_rviz/tesseract_state_plugin/tesseract_state_display.h)

add_library(${PROJECT_NAME}_state_plugin_core SHARED src/tesseract_state_plugin/tesseract_state_display.cpp ${${PROJECT_NAME}_state_plugin_core_cpp_MOCS})
target_link_libraries(${PROJECT_NAME}_state_plugin_core PUBLIC
    ${PROJECT_NAME}_render_tools
    ${catkin_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_state_plugin_core PUBLIC)
target_include_directories(${PROJECT_NAME}_state_plugin_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_state_plugin_core SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${QT_INCLUDE_DIR}
    ${OGRE_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_state_plugin SHARED src/tesseract_state_plugin/plugin_init.cpp)
target_link_libraries(${PROJECT_NAME}_state_plugin PRIVATE ${PROJECT_NAME}_state_plugin_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_state_plugin PRIVATE)
target_include_directories(${PROJECT_NAME}_state_plugin PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_state_plugin SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

## Trajectory Display
qt_wrap_cpp(${PROJECT_NAME}_trajectory_plugin_core_cpp_MOCS include/tesseract_rviz/tesseract_trajectory_plugin/tesseract_trajectory_display.h)

add_library(${PROJECT_NAME}_trajectory_plugin_core SHARED src/tesseract_trajectory_plugin/tesseract_trajectory_display.cpp ${${PROJECT_NAME}_trajectory_plugin_core_cpp_MOCS})
target_link_libraries(${PROJECT_NAME}_trajectory_plugin_core PUBLIC
    ${PROJECT_NAME}_render_tools
    ${PROJECT_NAME}_state_plugin_core
    tesseract::tesseract
    ${catkin_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_trajectory_plugin_core PUBLIC)
target_include_directories(${PROJECT_NAME}_trajectory_plugin_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_trajectory_plugin_core SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${QT_INCLUDE_DIR}
    ${OGRE_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_trajectory_plugin SHARED src/tesseract_trajectory_plugin/plugin_init.cpp)
target_link_libraries(${PROJECT_NAME}_trajectory_plugin PRIVATE
    ${PROJECT_NAME}_trajectory_plugin_core
    tesseract::tesseract
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_trajectory_plugin PRIVATE)
target_include_directories(${PROJECT_NAME}_trajectory_plugin PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_trajectory_plugin SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

## Manipulation Display
qt_wrap_cpp(${PROJECT_NAME}_manipulation_plugin_core_cpp_MOCS include/tesseract_rviz/tesseract_manipulation_plugin/tesseract_manipulation_display.h)

add_library(${PROJECT_NAME}_manipulation_plugin_core SHARED src/tesseract_manipulation_plugin/tesseract_manipulation_display.cpp ${${PROJECT_NAME}_manipulation_plugin_core_cpp_MOCS})
target_link_libraries(${PROJECT_NAME}_manipulation_plugin_core PUBLIC
    ${PROJECT_NAME}_render_tools
    tesseract::tesseract
    ${catkin_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES}
    ${EIGEN3_INCLUDE_DIRS})
tesseract_target_compile_options(${PROJECT_NAME}_manipulation_plugin_core PUBLIC)
target_include_directories(${PROJECT_NAME}_manipulation_plugin_core PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_manipulation_plugin_core SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${QT_INCLUDE_DIR}
    ${OGRE_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_manipulation_plugin SHARED src/tesseract_manipulation_plugin/plugin_init.cpp)
target_link_libraries(${PROJECT_NAME}_manipulation_plugin PRIVATE
    ${PROJECT_NAME}_manipulation_plugin_core
    tesseract::tesseract
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_manipulation_plugin PRIVATE)
target_include_directories(${PROJECT_NAME}_manipulation_plugin PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_manipulation_plugin SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_test src/test.cpp)
target_link_libraries(${PROJECT_NAME}_test PRIVATE ${PROJECT_NAME}_trajectory_plugin_core ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_test PRIVATE)
target_include_directories(${PROJECT_NAME}_test PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_test SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})

# Files generated by MOC, RCC, and UIC may produce clang-tidy warnings.
# We generate a dummy .clang-tidy file in the binary directory that disables
# all clang-tidy checks except one that will never match.  This one check is
# necessary; clang-tidy reports an error when no checks are enabled.
# Since the Qt code generators will generate source files in the binary tree,
# clang-tidy will load the configuration from this dummy file when the sources
# are built.
#file(WRITE "${tesseract_rviz_BINARY_DIR}/.clang-tidy" "
#---
#Checks: '-*,llvm-twine-local'
#...
#")


# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_render_tools ${PROJECT_NAME}_state_plugin_core ${PROJECT_NAME}_state_plugin ${PROJECT_NAME}_trajectory_plugin_core ${PROJECT_NAME}_trajectory_plugin ${PROJECT_NAME}_manipulation_plugin_core ${PROJECT_NAME}_manipulation_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark plugin XML files for installation
install(FILES tesseract_rviz_state_plugin_description.xml tesseract_rviz_trajectory_plugin_description.xml tesseract_rviz_manipulation_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ogre_media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
