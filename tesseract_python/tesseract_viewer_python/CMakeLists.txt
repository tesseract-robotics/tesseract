cmake_minimum_required(VERSION 3.5.0)
project(tesseract_python_viewer)

if(NOT PYTHON_VERSION)
  set(PYTHON_VERSION  $ENV{ROS_PYTHON_VERSION})
  message(STATUS "Setting Python Version: " ${PYTHON_VERSION})
endif()

if(PYTHON_VERSION)
  if(${PYTHON_VERSION} LESS 3)
    set(Python_ADDITIONAL_VERSIONS 2.7 2.6 2.5 2.4 2.3 2.2 2.1 2.0)
  else()
    set(Python_ADDITIONAL_VERSIONS 3.8 3.7 3.6 3.5 3.4 3.3 3.2 3.1 3.0)
  endif()
endif()

find_package(PythonInterp ${PYTHON_VERSION} REQUIRED)
find_package(PythonLibs REQUIRED)
message(STATUS "Building Python Version: " ${PYTHON_VERSION_MAJOR})

# Read version
FILE (STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" tesseract_viewer_python_version1 REGEX "<version>[0-9]+\\.[0-9]+\\.[0-9]+</version>")
IF ("${tesseract_viewer_python_version1}" STREQUAL "" )
MESSAGE(FATAL_ERROR "Could not read tesseract_viewer_python version.")
ENDIF()
STRING (REGEX REPLACE "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" "\\1" tesseract_viewer_python_version2 ${tesseract_viewer_python_version1})
STRING (STRIP ${tesseract_viewer_python_version2} tesseract_viewer_python_version)
MESSAGE(STATUS "tesseract_viewer_python version: ${tesseract_viewer_python_version}")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/setup.py.in" "${CMAKE_CURRENT_BINARY_DIR}/python/setup.py" @ONLY)

set(enable_setuptools_deb_layout OFF)
if(EXISTS "/etc/debian_version")
  set(enable_setuptools_deb_layout ON)
endif()
option(SETUPTOOLS_DEB_LAYOUT "Enable debian style python package layout" ${enable_setuptools_deb_layout})
if(SETUPTOOLS_DEB_LAYOUT)
  message(STATUS "Using Debian Python package layout")
  set(SETUPTOOLS_ARG_EXTRA "--install-layout=deb")
endif()

install(CODE "message(STATUS \"Running setup.py in ${CMAKE_CURRENT_SOURCE_DIR}\")
execute_process(COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/python/setup.py install -f
--root=/ --prefix=${CMAKE_INSTALL_PREFIX} ${SETUPTOOLS_ARG_EXTRA} --single-version-externally-managed WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})")

  # Allows Colcon to find non-Ament packages when using workspace underlays
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv DESTINATION share/${PROJECT_NAME}/hook)

if (${PYTHON_VERSION_MAJOR} LESS 3)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv "prepend-non-duplicate;PYTHONPATH;lib/python2.7/dist-packages")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
else()
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv "prepend-non-duplicate;PYTHONPATH;lib/python3/dist-packages")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
endif()
