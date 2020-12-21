function(tesseract_python_module PY_MOD_NAME )

  cmake_parse_arguments(PY_MOD "" "PACKAGE" "SWIG_SRCS;LIBS" ${ARGN})	

  list(GET PY_MOD_SWIG_SRCS 0 PY_MOD_SWIG_SRC1)

  set(SWIG_CXX_EXTENSION cxx)
  set_property(SOURCE ${PY_MOD_SWIG_SRC1} PROPERTY CPLUSPLUS ON)
  if (${PYTHON_VERSION_MAJOR} LESS 3)
    set_property(SOURCE ${PY_MOD_SWIG_SRC1} PROPERTY SWIG_FLAGS -relativeimport -threads -doxygen -DSWIGPYTHON2 )
  else()
    set_property(SOURCE ${PY_MOD_SWIG_SRC1} PROPERTY SWIG_FLAGS -relativeimport -threads -doxygen -py3 )
  endif()
  set_property(SOURCE ${PY_MOD_SWIG_SRC1} PROPERTY USE_LIBRARY_INCLUDE_DIRECTORIES TRUE)

  set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract/${PY_MOD_PACKAGE})
  set(SWIG_OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR})

  swig_add_module(${PY_MOD_NAME} python ${PY_MOD_SWIG_SRCS})
  swig_link_libraries(${PY_MOD_NAME} ${PY_MOD_LIBS} jsoncpp_lib
    yaml-cpp ${TinyXML2_LIBRARIES} ${EIGEN3_LIBRARIES} ${PYTHON_LIBRARIES})

  set(PY_MOD_REAL_NAME1 SWIG_MODULE_${PY_MOD_NAME}_REAL_NAME)
  set(PY_MOD_REAL_NAME ${${PY_MOD_REAL_NAME1}})

  target_include_directories(${PY_MOD_REAL_NAME} PUBLIC 
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/swig>"
      "$<INSTALL_INTERFACE:include>")
  target_include_directories(${PY_MOD_REAL_NAME}  SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${TinyXML2_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${NUMPY_INCLUDE_DIR}
    )
  target_compile_definitions(${PY_MOD_REAL_NAME} PRIVATE -DSWIG_TYPE_TABLE=tesseract_python )
 
  set_target_properties(${PY_MOD_REAL_NAME}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract/${PY_MOD_PACKAGE})
  set_target_properties(${PY_MOD_REAL_NAME}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract/${PY_MOD_PACKAGE})
  set_target_properties(${PY_MOD_REAL_NAME}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract/${PY_MOD_PACKAGE})

  target_compile_options(${PY_MOD_REAL_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
  target_compile_options(${PY_MOD_REAL_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_cxx_version(${PY_MOD_REAL_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/__init__.py.in ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract/${PY_MOD_PACKAGE}/__init__.py @ONLY)

endfunction()