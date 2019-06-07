cmake_minimum_required(VERSION 3.5.0)
project(tesseract_scene_graph VERSION 0.1.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(console_bridge REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(TinyXML REQUIRED)
find_package(tesseract_geometry REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  find_package(PkgConfig REQUIRED)
  # assimp is required, so REQUIRE the second attempt
  pkg_check_modules(ASSIMP REQUIRED assimp)
endif()

add_library(${PROJECT_NAME} SHARED src/graph.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC tesseract_geometry orocos-kdl console_bridge ${Boost_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${ASSIMP_LIBRARIES} ${TinyXML_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_definitions(${PROJECT_NAME} PUBLIC Boost_USE_STATIC_LIBS=OFF Boost_USE_MULTITHREADED=ON Boost_USE_STATIC_RUNTIME=OFF)
if(CXX_FEATURE_FOUND EQUAL "-1")
    set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES)
else()
    target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}) # Orocos did not export target correctly

list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES cmake/FindTinyXML.cmake DESTINATION lib/cmake/${PROJECT_NAME})

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

option(ENABLE_TESTS "Enable tests" OFF)
if (${ENABLE_TESTS})
  enable_testing()
  add_custom_target(run_tests ALL
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> -V)

  add_subdirectory(test)
endif()
