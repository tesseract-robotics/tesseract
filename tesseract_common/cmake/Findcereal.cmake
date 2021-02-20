# Try to find Cereal
#
# cereal_INCLUDE_DIRS - Directories with Cereal headers
# cereal_FOUND        - True if cereal found

find_package(cereal CONFIG)

if (NOT TARGET cereal)
  find_path(cereal_header_paths_tmp NAMES cereal.hpp PATH_SUFFIXES include include/cereal)

  get_filename_component(cereal_INCLUDE_DIRS ${cereal_header_paths_tmp} PATH)

  mark_as_advanced(cereal_INCLUDE_DIRS)

  # Output variables generation
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(cereal REQUIRED_VARS cereal_INCLUDE_DIRS)

  if (cereal_FOUND)
    add_library(cereal::cereal INTERFACE IMPORTED)
    set_target_properties(cereal::cereal PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${cereal_INCLUDE_DIRS}")
  endif()
endif()
