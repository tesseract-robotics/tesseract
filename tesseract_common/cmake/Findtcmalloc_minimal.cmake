# * Find tcmalloc_minimal
#
# tcmalloc_minimal_LIBRARIES   - List of libraries when using tcmalloc. tcmalloc_minimal_FOUND       - True if
# tcmalloc_minimal found.

set(tcmalloc_minimal_NAMES
    tcmalloc_minimal
    libtcmalloc_minimal
    libtcmalloc_minimal.so
    tcmalloc_minimal4
    libtcmalloc_minimal.so.4)

find_library(
  tcmalloc_minimal_LIBRARY NO_DEFAULT_PATH
  NAMES ${tcmalloc_minimal_NAMES}
  PATHS /lib
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
        /opt/local/lib)

if(tcmalloc_minimal_LIBRARY)
  set(tcmalloc_minimal_FOUND TRUE)
else()
  set(tcmalloc_minimal_FOUND FALSE)
endif()

if(tcmalloc_minimal_FOUND)
  message(STATUS "Found tcmalloc_minimal: ${tcmalloc_minimal_LIBRARY}")
else()
  message(STATUS "Not Found tcmalloc_minimal: ${tcmalloc_minimal_LIBRARY} with names ${tcmalloc_minimal_NAMES}")
endif()

mark_as_advanced(tcmalloc_minimal_LIBRARY)

# Output variables generation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(tcmalloc_minimal DEFAULT_MSG tcmalloc_minimal_LIBRARY)

if(tcmalloc_minimal_FOUND)
  find_package(Threads REQUIRED)
  add_library(tcmalloc::tcmalloc_minimal INTERFACE IMPORTED)
  set_property(TARGET tcmalloc::tcmalloc_minimal PROPERTY INTERFACE_LINK_LIBRARIES ${tcmalloc_minimal_LIBRARY}
                                                          Threads::Threads)
endif()
