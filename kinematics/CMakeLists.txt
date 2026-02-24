# All of these are used by all components so to avoid having them in each they are include here
find_package(boost_plugin_loader REQUIRED)
find_package(yaml-cpp REQUIRED)

# Define compile-time default variables
if(MSVC)
  set(TESSERACT_KINEMATICS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_KINEMATICS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

# Variable for kinematics plugins
set(KINEMATICS_PLUGINS "")

# Core
list(APPEND SUPPORTED_COMPONENTS kinematics)
add_subdirectory(core)

# IKFast
option(TESSERACT_BUILD_IKFAST "Build IKFast components" ON)
if(TESSERACT_BUILD_IKFAST)
  message("Building IKFast components")
  list(APPEND SUPPORTED_COMPONENTS kinematics_ikfast)
  add_subdirectory(ikfast)
endif()

# KDL
option(TESSERACT_BUILD_KDL "Build KDL components" ON)
if(TESSERACT_BUILD_KDL)
  message("Building KDL components")
  list(APPEND SUPPORTED_COMPONENTS kinematics_kdl)
  add_subdirectory(kdl)
endif()

# OPW
option(TESSERACT_BUILD_OPW "Build OPW components" ON)
if(TESSERACT_BUILD_OPW)
  message("Building OPW components")
  list(APPEND SUPPORTED_COMPONENTS kinematics_opw)
  add_subdirectory(opw)
endif()

# UR
option(TESSERACT_BUILD_UR "Build UR components" ON)
if(TESSERACT_BUILD_UR)
  message("Building UR components")
  list(APPEND SUPPORTED_COMPONENTS kinematics_ur)
  add_subdirectory(ur)
endif()

# Add compiler definition to core so it can find all plugins produced
string(
  REPLACE ";"
          ":"
          KINEMATICS_PLUGINS_STRING
          "${KINEMATICS_PLUGINS}")
target_compile_definitions(kinematics PRIVATE TESSERACT_KINEMATICS_PLUGINS="${KINEMATICS_PLUGINS_STRING}")

# Testing
if((TESSERACT_ENABLE_TESTING OR TESSERACT_KINEMATICS_ENABLE_TESTING)
   AND TESSERACT_BUILD_IKFAST
   AND TESSERACT_BUILD_KDL
   AND TESSERACT_BUILD_OPW
   AND TESSERACT_BUILD_UR)
  enable_testing()
  add_subdirectory(test)
endif()

# Propagate accumulated components to parent scope
set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} PARENT_SCOPE)
