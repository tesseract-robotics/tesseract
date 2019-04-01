cmake_minimum_required(VERSION 3.5.0)
project(tesseract_support)

# define library version (update: apparently you can also do it in project()!)
set(TESSERACT_SUPPORT_VERSION_MAJOR 0 CACHE STRING "major version" FORCE)
set(TESSERACT_SUPPORT_VERSION_MINOR 1 CACHE STRING "minor version" FORCE)
set(TESSERACT_SUPPORT_VERSION_PATCH 0 CACHE STRING "patch version" FORCE)
set(TESSERACT_SUPPORT_VERSION ${TESSERACT_SUPPORT_VERSION_MAJOR}.${TESSERACT_SUPPORT_VERSION_MINOR}.${TESSERACT_SUPPORT_VERSION_PATCH} CACHE STRING "version" FORCE)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE)
target_compile_definitions(${PROJECT_NAME} INTERFACE TESSERACT_SUPPORT_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})

foreach(dir urdf meshes)
   install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

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
  VERSION ${TESSERACT_SUPPORT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

set(TESSERACT_SUPPORT_PKGCONFIG_DESCRIPTION "Contains resource files for running tests throught tesseract")
configure_file(cmake/${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
install(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
