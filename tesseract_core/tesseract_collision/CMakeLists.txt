cmake_minimum_required(VERSION 3.5.0)
project(tesseract_collision)

# define library version (update: apparently you can also do it in project()!)
set(TESSERACT_COLLISION_VERSION_MAJOR 0 CACHE STRING "major version" FORCE)
set(TESSERACT_COLLISION_VERSION_MINOR 1 CACHE STRING "minor version" FORCE)
set(TESSERACT_COLLISION_VERSION_PATCH 0 CACHE STRING "patch version" FORCE)
set(TESSERACT_COLLISION_VERSION ${TESSERACT_COLLISION_VERSION_MAJOR}.${TESSERACT_COLLISION_VERSION_MINOR}.${TESSERACT_COLLISION_VERSION_PATCH} CACHE STRING "version" FORCE)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread program_options REQUIRED)
find_package(octomap REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(ccd REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(Bullet REQUIRED)

pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")
list(APPEND LIBFCL_LIBRARIES ${CCD_LIBRARIES})

set(TESSERACT_COLLISION_PKGCONFIG_REQUIRES Eigen3 console_bridge Boost octomap PkgConfig ccd fcl Bullet tesseract_geometry)

add_library(${PROJECT_NAME}_bullet SHARED
  src/bullet/bullet_cast_bvh_manager.cpp
  src/bullet/bullet_cast_simple_manager.cpp
  src/bullet/bullet_discrete_bvh_manager.cpp
  src/bullet/bullet_discrete_simple_manager.cpp
  src/bullet/bullet_utils.cpp
)

target_link_libraries(${PROJECT_NAME}_bullet tesseract_geometry Eigen3::Eigen ${BULLET_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${console_bridge_LIBRARIES})
target_compile_options(${PROJECT_NAME}_bullet PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(${PROJECT_NAME}_bullet PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}_bullet PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_bullet SYSTEM PUBLIC
    ${BULLET_INCLUDE_DIRS}
    ${LIBFCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_fcl SHARED src/fcl/fcl_discrete_managers.cpp src/fcl/fcl_utils.cpp)
target_link_libraries(${PROJECT_NAME}_fcl tesseract_geometry Eigen3::Eigen ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ${console_bridge_LIBRARIES})
target_compile_options(${PROJECT_NAME}_fcl PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(${PROJECT_NAME}_fcl PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}_fcl PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_fcl SYSTEM PUBLIC
    ${BULLET_INCLUDE_DIRS}
    ${LIBFCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS})

add_executable(create_convex_hull src/create_convex_hull.cpp)
target_link_libraries(create_convex_hull tesseract_geometry Eigen3::Eigen ${BULLET_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${console_bridge_LIBRARIES})
target_compile_options(create_convex_hull PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
target_compile_features(create_convex_hull PRIVATE cxx_std_11)
target_include_directories(create_convex_hull PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(create_convex_hull SYSTEM PUBLIC
    ${BULLET_INCLUDE_DIRS}
    ${LIBFCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS})

list (APPEND TESSERACT_COLLISION_LIBRARIES ${PROJECT_NAME}_bullet)
list (APPEND TESSERACT_COLLISION_LIBRARIES ${PROJECT_NAME}_fcl)

# Mark executables and/or libraries for installation
install(TARGETS ${TESSERACT_COLLISION_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})
install(TARGETS create_convex_hull DESTINATION bin)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/${PROJECT_NAME}-config.cmake.in ${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${PROJECT_NAME}-config-version.cmake
  VERSION ${TESSERACT_COLLISION_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

set(TESSERACT_COLLISION_PKGCONFIG_DESCRIPTION "Libraries for performing collision checking")
configure_file(cmake/${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
install(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

option(ENABLE_TESTS "Enable tests" OFF)
if (${ENABLE_TESTS})
  enable_testing()
  add_subdirectory(test)
endif()
