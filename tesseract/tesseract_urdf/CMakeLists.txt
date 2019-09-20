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

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

add_library(${PROJECT_NAME} SHARED src/urdf_parser.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract::tesseract_geometry tesseract::tesseract_scene_graph tesseract::tesseract_collision_core console_bridge ${Boost_LIBRARIES} ${TinyXML2_LIBRARIES} ${PCL_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_definitions(${PROJECT_NAME} PUBLIC Boost_USE_STATIC_LIBS=OFF Boost_USE_MULTITHREADED=ON Boost_USE_STATIC_RUNTIME=OFF)
foreach(DEF ${PCL_DEFINITIONS})
  string(STRIP ${DEF} DEF)
  if (NOT "${DEF}" STREQUAL "")
    target_compile_definitions(${PROJECT_NAME} PUBLIC ${DEF})
  endif()
endforeach()

if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
endif()
target_compile_options(${PROJECT_NAME} PUBLIC -mno-avx)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${TinyXML2_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    $<TARGET_PROPERTY:tesseract::tesseract_common,INTERFACE_INCLUDE_DIRECTORIES>) #tesseract::tesseract_common Due to bug in catkin, there is an open PR

list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE tesseract:: DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES cmake/FindTinyXML2.cmake DESTINATION lib/cmake/${PROJECT_NAME})

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)

# ADD Examples
add_subdirectory(examples)

if (ENABLE_TESTS)
  enable_testing()
  add_custom_target(run_tests ALL
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> -V)

  add_subdirectory(test)
endif()
