cmake_minimum_required(VERSION 3.18.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

include(common/cmake/tesseract_macros.cmake)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/common/cmake/")

find_package(Eigen3 REQUIRED)
find_package(cereal REQUIRED)
find_package(console_bridge REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/common/test/*
    ${CMAKE_CURRENT_LIST_DIR}/collision/test/*
    ${CMAKE_CURRENT_LIST_DIR}/environment/test/*
    ${CMAKE_CURRENT_LIST_DIR}/geometry/test/*
    ${CMAKE_CURRENT_LIST_DIR}/kinematics/test/*
    ${CMAKE_CURRENT_LIST_DIR}/scene_graph/test/*
    ${CMAKE_CURRENT_LIST_DIR}/state_solver/test/*
    ${CMAKE_CURRENT_LIST_DIR}/srdf/test/*
    ${CMAKE_CURRENT_LIST_DIR}/urdf/test/*
    ${CMAKE_CURRENT_LIST_DIR}/visualization/test/*
    ${CMAKE_CURRENT_LIST_DIR}/include/collision/test_suite/*
    ${CMAKE_CURRENT_LIST_DIR}/include/collision/vhacd/*
    /*/install/*
    /*/devel/*
    /*/gtest/*
    /*/bullet/LinearMath/*
    /*/bullet/BulletCollision/*
    /*/include/ccd/*
    /*/include/fcl/*)

add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

set(SUPPORTED_COMPONENTS "")

if(TESSERACT_ENABLE_TESTING)
  # Enable testing for colcon test discovery
  enable_testing()
endif()

# Add subdirectories in dependency order.
add_subdirectory(common)
add_subdirectory(geometry)
add_subdirectory(scene_graph)
add_subdirectory(state_solver)
add_subdirectory(collision)
add_subdirectory(srdf)
add_subdirectory(urdf)
add_subdirectory(kinematics)
add_subdirectory(environment)
add_subdirectory(visualization)
add_subdirectory(support)

configure_package(COMPONENT tesseract SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS})

if(TESSERACT_PACKAGE)
  cpack_component_package(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/README.md
    COMPONENT_DEPENDS ${SUPPORTED_COMPONENTS})

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
