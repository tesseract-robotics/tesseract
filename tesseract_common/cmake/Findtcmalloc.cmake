# * Find tcmalloc
#
# tcmalloc_LIBRARIES   - List of libraries when using tcmalloc. tcmalloc_FOUND       - True if tcmalloc found.

set(tcmalloc_NAMES
    tcmalloc_minimal
    libtcmalloc_minimal
    tcmalloc
    tcmalloc_minimal4
    libtcmalloc_minimal.so.4)

find_library(
  tcmalloc_LIBRARY NO_DEFAULT_PATH
  NAMES ${tcmalloc_NAMES}
  PATHS /lib
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
        /opt/local/lib)

if(tcmalloc_LIBRARY)
  set(tcmalloc_FOUND TRUE)
else()
  set(tcmalloc_FOUND FALSE)
endif()

if(tcmalloc_FOUND)
  message(STATUS "Found tcmalloc: ${tcmalloc_LIBRARY}")
else()
  message(STATUS "Not Found tcmalloc: ${tcmalloc_LIBRARY} with names ${tcmalloc_NAMES}")
endif()

mark_as_advanced(tcmalloc_LIBRARY)

# Output variables generation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(tcmalloc DEFAULT_MSG tcmalloc_LIBRARY)

if(tcmalloc_FOUND)
  add_library(tcmalloc::tcmalloc INTERFACE IMPORTED)
  set_target_properties(tcmalloc::tcmalloc PROPERTIES INTERFACE_LINK_LIBRARIES ${tcmalloc_LIBRARY})
endif()
