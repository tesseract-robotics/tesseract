list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(octomap REQUIRED)
find_package(assimp REQUIRED)

# These targets are necessary for 16.04 builds. Remove when Kinetic support is dropped
if(NOT TARGET octomap)
  add_library(octomap INTERFACE IMPORTED)
  set_target_properties(octomap PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIRS}")
  set_target_properties(octomap PROPERTIES INTERFACE_LINK_LIBRARIES "${OCTOMAP_LIBRARIES}")
endif()
if(NOT TARGET octomath)
  add_library(octomath INTERFACE IMPORTED)
  set_target_properties(octomath PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIRS}")
  set_target_properties(octomath PROPERTIES INTERFACE_LINK_LIBRARIES "${OCTOMAP_LIBRARIES}")
endif()

find_file(ASSIMP_PBR_MATERIAL_FOUND assimp/pbrmaterial.h HINTS ${assimp_INCLUDE_DIRS})

if(ASSIMP_PBR_MATERIAL_FOUND)
  set(TESSERACT_ASSIMP_USE_PBRMATERIAL TESSERACT_ASSIMP_USE_PBRMATERIAL=1)
else()
  set(TESSERACT_ASSIMP_USE_PBRMATERIAL)
endif()

set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} "geometry" PARENT_SCOPE)

add_library(
  geometry
  src/cereal_serialization.cpp
  src/conversions.cpp
  src/geometry.cpp
  src/utils.cpp
  src/geometries/box.cpp
  src/geometries/capsule.cpp
  src/geometries/compound_mesh.cpp
  src/geometries/cone.cpp
  src/geometries/convex_mesh.cpp
  src/geometries/cylinder.cpp
  src/geometries/mesh_material.cpp
  src/geometries/mesh.cpp
  src/geometries/octree.cpp
  src/geometries/plane.cpp
  src/geometries/polygon_mesh.cpp
  src/geometries/sdf_mesh.cpp
  src/geometries/sphere.cpp)
add_library(tesseract::geometry ALIAS geometry)
set_target_properties(geometry PROPERTIES OUTPUT_NAME tesseract_geometry)
target_link_libraries(
  geometry
  PUBLIC Eigen3::Eigen
         tesseract::common
         console_bridge::console_bridge
         octomap
         octomath
         cereal::cereal
         assimp::assimp)
target_compile_options(geometry PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(geometry PUBLIC ${TESSERACT_COMPILE_DEFINITIONS} ${TESSERACT_ASSIMP_USE_PBRMATERIAL})
target_clang_tidy(geometry ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(geometry PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  geometry
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(geometry PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                           "$<INSTALL_INTERFACE:include>")

# Configure Package
configure_component(COMPONENT geometry NAMESPACE tesseract TARGETS geometry)

# Mark cpp header files for installation
install(
  DIRECTORY include/tesseract
  DESTINATION include
  COMPONENT geometry
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(FILES "${CMAKE_CURRENT_LIST_DIR}/cmake/Findassimp.cmake" DESTINATION lib/cmake/${PROJECT_NAME}
        COMPONENT geometry)

# ADD Examples
if(TESSERACT_ENABLE_EXAMPLES OR TESSERACT_GEOMETRY_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

if(TESSERACT_ENABLE_TESTING OR TESSERACT_GEOMETRY_ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack(
    COMPONENT geometry
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Geometry Components"
    LINUX_DEPENDS
      "libassimp-dev"
      "libconsole-bridge-dev"
      "liboctomap-dev (>= 1.9.3) | ${TESSERACT_PACKAGE_PREFIX}octomap"
      "libeigen3-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
    WINDOWS_DEPENDS
      "Assimp"
      "console_bridge"
      "octomap | ${TESSERACT_PACKAGE_PREFIX}octomap"
      "Eigen3"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
