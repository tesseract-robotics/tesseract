cmake_minimum_required(VERSION 3.5.0)
project(tesseract_python)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# Read version
FILE (STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" tesseract_python_version1 REGEX "<version>[0-9]+\\.[0-9]+\\.[0-9]+</version>")
IF ("${tesseract_python_version1}" STREQUAL "" )
MESSAGE(FATAL_ERROR "Could not read tesseract_python version.")
ENDIF()
STRING (REGEX REPLACE "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" "\\1" tesseract_python_version2 ${tesseract_python_version1})
STRING (STRIP ${tesseract_python_version2} tesseract_python_version)
MESSAGE(STATUS "tesseract_python version: ${tesseract_python_version}")


if (NOT MSVC)
  add_compile_options(-w)
else()
  add_compile_options(/bigobj)
endif()

find_package(cmake_common_scripts REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_urdf REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_time_parameterization REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)
find_package(trajopt REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(opw_kinematics REQUIRED)

tesseract_variables()

include_directories(
  ${trajopt_INCLUDE_DIRS} 
  ${tesseract_common_INCLUDE_DIRS}
  ${tesseract_geometry_INCLUDE_DIRS}
  ${tesseract_collision_INCLUDE_DIRS}
  ${tesseract_motion_planners_INCLUDE_DIRS}
  ${tesseract_kinematics_INCLUDE_DIRS}
  ${tesseract_urdf_INCLUDE_DIRS}
  ${tesseract_scene_graph_INCLUDE_DIRS}
  ${tesseract_environment_INCLUDE_DIRS}
  ${tesseract_command_language_INCLUDE_DIRS}
  ${tesseract_time_parameterization_INCLUDE_DIRS}
  ${tesseract_visualization_INCLUDE_DIRS}
  ${tesseract_process_managers_INCLUDE_DIRS}
  ${opw_kinematics_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
)

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

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/tesseract_python_module.cmake)

tesseract_python_module(tesseract_common_python SWIG_SRCS swig/tesseract_common_python.i PACKAGE tesseract_common LIBS tesseract::tesseract_common )
tesseract_python_module(tesseract_geometry_python SWIG_SRCS swig/tesseract_geometry_python.i PACKAGE tesseract_geometry LIBS tesseract::tesseract_geometry )
tesseract_python_module(tesseract_collision_python SWIG_SRCS swig/tesseract_collision_python.i PACKAGE tesseract_collision LIBS tesseract::tesseract_collision_core )
tesseract_python_module(tesseract_collision_bullet_python SWIG_SRCS swig/tesseract_collision_bullet_python.i PACKAGE tesseract_collision_bullet LIBS tesseract::tesseract_collision_bullet )
tesseract_python_module(tesseract_collision_fcl_python SWIG_SRCS swig/tesseract_collision_fcl_python.i PACKAGE tesseract_collision_fcl LIBS tesseract::tesseract_collision_fcl )
tesseract_python_module(tesseract_scene_graph_python SWIG_SRCS swig/tesseract_scene_graph_python.i PACKAGE tesseract_scene_graph LIBS tesseract::tesseract_scene_graph )
tesseract_python_module(tesseract_urdf_python SWIG_SRCS swig/tesseract_urdf_python.i PACKAGE tesseract_urdf LIBS tesseract::tesseract_urdf )
tesseract_python_module(tesseract_kinematics_python SWIG_SRCS swig/tesseract_kinematics_python.i PACKAGE tesseract_kinematics LIBS tesseract::tesseract_kinematics_core )
tesseract_python_module(tesseract_kinematics_kdl_python SWIG_SRCS swig/tesseract_kinematics_kdl_python.i PACKAGE tesseract_kinematics_kdl LIBS tesseract::tesseract_kinematics_kdl )
tesseract_python_module(tesseract_kinematics_opw_python SWIG_SRCS swig/tesseract_kinematics_opw_python.i PACKAGE tesseract_kinematics_opw LIBS tesseract::tesseract_kinematics_opw )
tesseract_python_module(tesseract_environment_python SWIG_SRCS swig/tesseract_environment_python.i PACKAGE tesseract_environment LIBS tesseract::tesseract_environment_core tesseract::tesseract_environment_kdl )
tesseract_python_module(tesseract_command_language_python SWIG_SRCS swig/tesseract_command_language_python.i PACKAGE tesseract_command_language LIBS tesseract::tesseract_command_language)
tesseract_python_module(tesseract_motion_planners_python SWIG_SRCS swig/tesseract_motion_planners_python.i PACKAGE tesseract_motion_planners LIBS tesseract::tesseract_motion_planners_simple tesseract::tesseract_motion_planners_core )
tesseract_python_module(tesseract_motion_planners_simple_python SWIG_SRCS swig/tesseract_motion_planners_simple_python.i PACKAGE tesseract_motion_planners_simple LIBS tesseract::tesseract_motion_planners_simple )
tesseract_python_module(tesseract_motion_planners_trajopt_python SWIG_SRCS swig/tesseract_motion_planners_trajopt_python.i PACKAGE tesseract_motion_planners_trajopt LIBS tesseract::tesseract_motion_planners_trajopt )
tesseract_python_module(tesseract_motion_planners_ompl_python SWIG_SRCS swig/tesseract_motion_planners_ompl_python.i PACKAGE tesseract_motion_planners_ompl LIBS tesseract::tesseract_motion_planners_ompl )
tesseract_python_module(tesseract_motion_planners_descartes_python SWIG_SRCS swig/tesseract_motion_planners_descartes_python.i PACKAGE tesseract_motion_planners_descartes LIBS tesseract::tesseract_motion_planners_descartes )
tesseract_python_module(tesseract_time_parameterization_python SWIG_SRCS swig/tesseract_time_parameterization_python.i PACKAGE tesseract_time_parameterization LIBS tesseract::tesseract_time_parameterization )
tesseract_python_module(tesseract_visualization_python SWIG_SRCS swig/tesseract_visualization_python.i PACKAGE tesseract_visualization LIBS tesseract::tesseract_visualization)
tesseract_python_module(tesseract_process_managers_python SWIG_SRCS swig/tesseract_process_managers_python.i PACKAGE tesseract_process_managers LIBS tesseract::tesseract_process_managers )

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/__package_init__.py.in ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract/__init__.py @ONLY)

#install(CODE "MESSAGE(\"INSTALL TODO!\")")

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

if (${PYTHON_VERSION_MAJOR} LESS 3)
    install(CODE "message(STATUS \"Running setup.py in ${CMAKE_CURRENT_BINARY_DIR}/python\")
    execute_process(COMMAND python2 setup.py install -f
--root=/ --prefix=${CMAKE_INSTALL_PREFIX} ${SETUPTOOLS_ARG_EXTRA} --single-version-externally-managed WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python)")
else()
  install(CODE "message(STATUS \"Running setup.py in ${CMAKE_CURRENT_BINARY_DIR}/python\")
  execute_process(COMMAND ${PYTHON_EXECUTABLE} setup.py install -f
  --root=/ --prefix=${CMAKE_INSTALL_PREFIX} ${SETUPTOOLS_ARG_EXTRA} --single-version-externally-managed WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python)")
endif()


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
  
if(TESSERACT_ENABLE_TESTING)
  enable_testing()

  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})

  if (${TESSERACT_ENABLE_RUN_TESTING})
  add_dependencies(run_tests _tesseract_common_python _tesseract_geometry_python _tesseract_collision_python 
    _tesseract_collision_bullet_python _tesseract_collision_fcl_python _tesseract_scene_graph_python
    _tesseract_urdf_python _tesseract_kinematics_python _tesseract_kinematics_kdl_python
    _tesseract_kinematics_opw_python _tesseract_environment_python _tesseract_command_language_python
    _tesseract_motion_planners_python _tesseract_motion_planners_simple_python
    _tesseract_motion_planners_trajopt_python _tesseract_motion_planners_ompl_python
    _tesseract_motion_planners_descartes_python _tesseract_time_parameterization_python
    _tesseract_visualization_python)
    endif()

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
