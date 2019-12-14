cmake_minimum_required(VERSION 3.5.0)
project(tesseract_scene_graph VERSION 0.1.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(TinyXML REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)

add_library(${PROJECT_NAME} SHARED src/graph.cpp src/resource_locator.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract::tesseract_geometry orocos-kdl console_bridge ${Boost_LIBRARIES} ${TinyXML_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME} PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME} PUBLIC Boost_USE_STATIC_LIBS=OFF Boost_USE_MULTITHREADED=ON Boost_USE_STATIC_RUNTIME=OFF)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS} # Orocos did not export target correctly
    $<TARGET_PROPERTY:tesseract::tesseract_common,INTERFACE_INCLUDE_DIRECTORIES>) #tesseract::tesseract_common Due to bug in catkin, there is an open PR

tesseract_configure_package(${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES cmake/FindTinyXML.cmake DESTINATION lib/cmake/${PROJECT_NAME})

# ADD Examples
add_subdirectory(examples)

if (TESSERACT_ENABLE_TESTING)
  enable_testing()
  tesseract_add_run_tests_target()
  add_subdirectory(test)
endif()
