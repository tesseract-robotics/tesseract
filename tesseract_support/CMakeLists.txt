cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version})

find_package(tesseract_common REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE)
target_compile_definitions(${PROJECT_NAME}
                           INTERFACE TESSERACT_SUPPORT_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME})

foreach(dir urdf meshes)
  install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

if(TESSERACT_PACKAGE)
  tesseract_cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER <https://github.com/ros-industrial-consortium/tesseract>
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
    WINDOWS_DEPENDS "${TESSERACT_PACKAGE_PREFIX}tesseract-common")
endif()
