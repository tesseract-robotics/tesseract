if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ASSIMP_ARCHITECTURE "64")
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
  set(ASSIMP_ARCHITECTURE "32")
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

if(MSVC)
  find_package(PkgConfig REQUIRED)
  # assimp is required, so REQUIRE the second attempt
  pkg_check_modules(PC_ASSIMP REQUIRED assimp)

  if(MSVC_TOOLSET_VERSION)
    set(ASSIMP_MSVC_VERSION "vc${MSVC_TOOLSET_VERSION}")
  else(MSVC_TOOLSET_VERSION)
    if(MSVC12)
      set(ASSIMP_MSVC_VERSION "vc120")
    elseif(MSVC14)
      set(ASSIMP_MSVC_VERSION "vc140")
    endif(MSVC12)
  endif(MSVC_TOOLSET_VERSION)

  # Find path of each library
  find_path(assimp_INCLUDE_DIR NAMES assimp/anim.h HINTS ${PC_ASSIMP_INCLUDEDIR} REQUIRED)
  find_path(
    assimp_LIBRARY_DIR
    NAMES assimp-${ASSIMP_MSVC_VERSION}-mt.lib
          assimp-vc140-mt.lib
          assimp-vc141-mt.lib
          assimp-vc120-mt.lib
    HINTS ${PC_ASSIMP_LIBDIR} "${PC_ASSIMP_PREFIX}/Lib" REQUIRED)
  find_library(
    assimp_LIBRARY
    NAMES assimp-${ASSIMP_MSVC_VERSION}-mt.lib
          assimp-vc140-mt.lib
          assimp-vc141-mt.lib
          assimp-vc120-mt.lib
    PATHS ${assimp_LIBRARY_DIR} REQUIRED)
else(MSVC)

  find_path(
    assimp_INCLUDE_DIR
    NAMES assimp/postprocess.h
          assimp/scene.h
          assimp/version.h
          assimp/config.h
          assimp/cimport.h
    PATHS /usr/local/include
    PATHS /usr/include/ REQUIRED)

  find_library(
    assimp_LIBRARY
    NAMES assimp
    PATHS /usr/local/lib/
    PATHS /usr/lib64/
    PATHS /usr/lib/ REQUIRED)
endif(MSVC)

mark_as_advanced(assimp_INCLUDE_DIR assimp_LIBRARY assimp_LIBRARY_DIR)

# Output variables generation
include(FindPackageHandleStandardArgs)
if(MSVC)
  find_package_handle_standard_args(assimp REQUIRED_VARS assimp_INCLUDE_DIR assimp_LIBRARY assimp_LIBRARY_DIR)
else(MSVC)
  find_package_handle_standard_args(assimp REQUIRED_VARS assimp_INCLUDE_DIR assimp_LIBRARY)
endif(MSVC)

if(assimp_FOUND)
  set(assimp_INCLUDE_DIRS ${assimp_INCLUDE_DIR})
  set(assimp_LIBRARIES ${assimp_LIBRARY})
  set(assimp_LIBRARY_DIRS ${assimp_LIBRARY_DIR})
endif()
