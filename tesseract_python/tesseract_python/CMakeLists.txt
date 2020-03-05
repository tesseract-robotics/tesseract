cmake_minimum_required(VERSION 3.5.0)
project(tesseract_python)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# Read version
FILE (STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" tesseract_python_version1 REGEX "<version>[0-9]+\\.[0-9]+\\.[0-9]+</version>")
IF ("${tesseract_python_version1}" STREQUAL "" )
MESSAGE(FATAL_ERROR "Could not read tesseract_python version.")
ENDIF()
STRING (REGEX REPLACE "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" "\\1" tesseract_python_version ${tesseract_python_version1})
MESSAGE(STATUS "tesseract_python version: ${tesseract_python_version}")

if (NOT MSVC)
  #add_compile_options(-std=c++11 -Wall -Wextra -Wno-write-strings -Wno-unused-paramete -Wno-suggest-override)
  add_compile_options(-std=c++11 -w)
else()
  add_compile_options(/bigobj)
endif()

find_package(trajopt REQUIRED)
find_package(tesseract REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)
find_package(trajopt REQUIRED)

find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)

# Find NumPy

find_path(NUMPY_INCLUDE_NDARRAYOBJECT_DIR numpy/ndarrayobject.h PATHS ${PYTHON_INCLUDE_DIRS} NO_DEFAULT_PATH)
if (NUMPY_INCLUDE_NDARRAYOBJECT_DIR)
set(NUMPY_INCLUDE_DIR ${NUMPY_INCLUDE_NDARRAYOBJECT_DIR})
else()
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import numpy; import os; print(os.path.dirname(numpy.__file__) + '/core/include')" RESULT_VARIABLE FIND_NUMPY_RESULT OUTPUT_VARIABLE NUMPY_INCLUDE_DIR )
string(STRIP "${NUMPY_INCLUDE_DIR}" NUMPY_INCLUDE_DIR)

if(${FIND_NUMPY_RESULT})
message(FATAL_ERROR "Could not determine NumPy include directory")
endif()
endif()

if (NOT EXISTS "${NUMPY_INCLUDE_DIR}/numpy/ndarrayobject.h")
message(FATAL_ERROR "Could not find numpy/ndarrayobject.h include file")
endif()

message(STATUS "NumPy Include Directory: ${NUMPY_INCLUDE_DIR}")

# End Find NumPy

include(FindSWIG)
find_package(SWIG 4.0 REQUIRED)
include(UseSWIG)

# Primary tesseract_python library

set(SWIG_CXX_EXTENSION cxx)
set_property(SOURCE swig/tesseract_python.i PROPERTY CPLUSPLUS ON)
if (${PYTHON_VERSION_MAJOR} LESS 3)
set_property(SOURCE swig/tesseract_python.i PROPERTY SWIG_FLAGS -relativeimport -threads)
else()
set_property(SOURCE swig/tesseract_python.i PROPERTY SWIG_FLAGS -relativeimport -threads -py3)
endif()

set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract)
set(SWIG_OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR})
swig_add_module(tesseract_python python swig/tesseract_python.i)
swig_link_libraries(tesseract_python tesseract::tesseract_motion_planners_trajopt tesseract::tesseract_motion_planners_core tesseract::tesseract_visualization tesseract::tesseract trajopt::trajopt ${PYTHON_LIBRARIES} ${EIGEN3_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(${SWIG_MODULE_tesseract_python_REAL_NAME} PUBLIC 
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/swig>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${SWIG_MODULE_tesseract_python_REAL_NAME}  SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${console_bridge_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${TinyXML2_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
	${NUMPY_INCLUDE_DIR}
    $<TARGET_PROPERTY:tesseract::tesseract,INTERFACE_INCLUDE_DIRECTORIES>)

set_target_properties(${SWIG_MODULE_tesseract_python_REAL_NAME}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract)
set_target_properties(${SWIG_MODULE_tesseract_python_REAL_NAME}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract)
set_target_properties(${SWIG_MODULE_tesseract_python_REAL_NAME}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract)

add_custom_command(TARGET ${SWIG_MODULE_tesseract_python_REAL_NAME} POST_BUILD COMMAND 
${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/swig/__init__.py
$<SHELL_PATH:$<TARGET_FILE_DIR:${SWIG_MODULE_tesseract_python_REAL_NAME}>>/__init__.py)

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

install(CODE "message(STATUS \"Running setup.py in ${CMAKE_CURRENT_BINARY_DIR}/python\")
execute_process(COMMAND python setup.py install -f
--root=/ --prefix=${CMAKE_INSTALL_PREFIX} ${SETUPTOOLS_ARG_EXTRA} --single-version-externally-managed WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python)")

if(TESSERACT_ENABLE_TESTING)
  enable_testing()

  tesseract_add_run_tests_target()

  add_test (NAME python-tests
  COMMAND ${PYTHON_EXECUTABLE} -m pytest -p no:cacheprovider
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  
  find_package(tesseract_support REQUIRED)
  if (WIN32)
  set_tests_properties(python-tests PROPERTIES ENVIRONMENT "TESSERACT_SUPPORT_DIR=${TESSERACT_SUPPORT_DIR}")
  else()
  set_tests_properties(python-tests PROPERTIES ENVIRONMENT "TESSERACT_SUPPORT_DIR=${TESSERACT_SUPPORT_DIR}")
  endif()
  

endif()
