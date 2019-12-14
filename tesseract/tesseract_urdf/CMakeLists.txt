cmake_minimum_required(VERSION 3.5.0)
project(tesseract_urdf VERSION 0.1.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(PCL REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_collision REQUIRED)

add_library(${PROJECT_NAME} SHARED src/urdf_parser.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract::tesseract_geometry tesseract::tesseract_scene_graph tesseract::tesseract_collision_core console_bridge ${Boost_LIBRARIES} ${TinyXML2_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME} PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME})

# PCL does not support c++11 on Xenial so cannot include point cloud parsing from urdf
# Boost version number is in XYYYZZ format such that:
#   (BOOST_VERSION % 100) is the sub-minor version
#   ((BOOST_VERSION / 100) % 1000) is the minor version
#   (BOOST_VERSION / 100000) is the major version.
if(Boost_VERSION VERSION_GREATER "106000")
    target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PUBLIC TESSERACT_PARSE_POINT_CLOUDS="ON")
    foreach(DEF ${PCL_DEFINITIONS})
      string(STRIP ${DEF} DEF)
      if (NOT "${DEF}" STREQUAL "")
         string(SUBSTRING "${DEF}" 0 2 DEF_PREFIX)
         if ("${DEF_PREFIX}" STREQUAL "-m")
             string(REPLACE " " ";" DEF ${DEF})
             foreach(OPTION_DEF ${DEF})
                 target_compile_options(${PROJECT_NAME} PUBLIC ${OPTION_DEF})
             endforeach()
         else()
            target_compile_definitions(${PROJECT_NAME} PUBLIC ${DEF})
         endif()
      endif()
    endforeach()
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${TinyXML2_INCLUDE_DIRS}
    $<TARGET_PROPERTY:tesseract::tesseract_common,INTERFACE_INCLUDE_DIRECTORIES>) #tesseract::tesseract_common Due to bug in catkin, there is an open PR

tesseract_configure_package(${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES cmake/FindTinyXML2.cmake DESTINATION lib/cmake/${PROJECT_NAME})

# ADD Examples
add_subdirectory(examples)

if (TESSERACT_ENABLE_TESTING)
  enable_testing()
  tesseract_add_run_tests_target()
  add_subdirectory(test)
endif()
