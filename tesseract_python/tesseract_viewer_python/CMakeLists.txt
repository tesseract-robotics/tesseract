cmake_minimum_required(VERSION 2.8.3)
project(tesseract_python_viewer)

# Read version
FILE (STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" tesseract_viewer_python_version1 REGEX "<version>[0-9]+\\.[0-9]+\\.[0-9]+</version>")
IF ("${tesseract_viewer_python_version1}" STREQUAL "" )
MESSAGE(FATAL_ERROR "Could not read tesseract_viewer_python version.")
ENDIF()
STRING (REGEX REPLACE "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" "\\1" tesseract_viewer_python_version ${tesseract_viewer_python_version1})
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
execute_process(COMMAND python ${CMAKE_CURRENT_BINARY_DIR}/python/setup.py install -f
--root=/ --prefix=${CMAKE_INSTALL_PREFIX} ${SETUPTOOLS_ARG_EXTRA} --single-version-externally-managed WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})")
