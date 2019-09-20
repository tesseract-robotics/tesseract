cmake_minimum_required(VERSION 3.5.0)
project(tesseract_collision VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread program_options REQUIRED)
find_package(octomap REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(ccd REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(Bullet REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")
list(APPEND LIBFCL_LIBRARIES ${CCD_LIBRARIES})

# Create interface for core
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE tesseract::tesseract_geometry ${Boost_LIBRARIES} ${BULLET_LIBRARIES})
target_compile_options(${PROJECT_NAME}_core INTERFACE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_core INTERFACE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_11)
endif()
target_compile_options(${PROJECT_NAME}_core INTERFACE -mno-avx)
target_include_directories(${PROJECT_NAME}_core INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_core SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    $<TARGET_PROPERTY:tesseract::tesseract_common,INTERFACE_INCLUDE_DIRECTORIES>) #tesseract::tesseract_common Due to bug in catkin, there is an open PR

# Create target for Bullet implementation
add_library(${PROJECT_NAME}_bullet SHARED
  src/bullet/bullet_cast_bvh_manager.cpp
  src/bullet/bullet_cast_simple_manager.cpp
  src/bullet/bullet_discrete_bvh_manager.cpp
  src/bullet/bullet_discrete_simple_manager.cpp
  src/bullet/bullet_utils.cpp
)
target_link_libraries(${PROJECT_NAME}_bullet PUBLIC ${PROJECT_NAME}_core tesseract::tesseract_geometry console_bridge ${OCTOMAP_LIBRARIES})
target_compile_options(${PROJECT_NAME}_bullet PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_bullet PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_bullet PUBLIC cxx_std_11)
endif()
target_compile_options(${PROJECT_NAME}_bullet PUBLIC -mno-avx)
target_include_directories(${PROJECT_NAME}_bullet PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_bullet SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS})

# Create target for FCL implementation
add_library(${PROJECT_NAME}_fcl SHARED src/fcl/fcl_discrete_managers.cpp src/fcl/fcl_utils.cpp)
target_link_libraries(${PROJECT_NAME}_fcl PUBLIC ${PROJECT_NAME}_core tesseract::tesseract_geometry console_bridge ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES})
target_compile_options(${PROJECT_NAME}_fcl PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_fcl PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_fcl PUBLIC cxx_std_11)
endif()
target_compile_options(${PROJECT_NAME}_fcl PUBLIC -mno-avx)
target_include_directories(${PROJECT_NAME}_fcl PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_fcl SYSTEM PUBLIC
    ${LIBFCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS})

# Create target for creating convex hull's from meshes
add_executable(create_convex_hull src/create_convex_hull.cpp)
target_link_libraries(create_convex_hull PUBLIC tesseract::tesseract_geometry console_bridge ${BULLET_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})
target_compile_options(create_convex_hull PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(create_convex_hull PUBLIC -std=c++11)
else()
    target_compile_features(create_convex_hull PUBLIC cxx_std_11)
endif()
target_compile_options(create_convex_hull PUBLIC -mno-avx)
target_include_directories(create_convex_hull PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(create_convex_hull SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${LIBFCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS})

list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_core)
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_bullet)
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_fcl)

# Mark executables and/or libraries for installation
install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE tesseract:: DESTINATION lib/cmake/${PROJECT_NAME})
install(TARGETS create_convex_hull DESTINATION bin)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

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
