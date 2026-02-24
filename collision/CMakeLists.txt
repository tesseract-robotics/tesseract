# All of these are used by all components so to avoid having them in each they are include here
find_package(Boost REQUIRED COMPONENTS system program_options)
find_package(octomap REQUIRED)
find_package(boost_plugin_loader REQUIRED)
find_package(yaml-cpp REQUIRED)

# Define compile-time default variables
if(MSVC)
  set(TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

# Variable for kinematics plugins
set(CONTACT_MANAGERS_PLUGINS "")

# Core
list(APPEND SUPPORTED_COMPONENTS collision)
add_subdirectory(core)

# Bullet (currently required for creation of convex hulls)
list(APPEND SUPPORTED_COMPONENTS collision_bullet)
add_subdirectory(bullet)

# Test Suite
option(TESSERACT_BUILD_TEST_SUITE "Build Test Suite components" ON)
if(TESSERACT_BUILD_TEST_SUITE)
  message("Building Test Suite components")
  list(APPEND SUPPORTED_COMPONENTS collision_test_suite)
  add_subdirectory(test_suite)
endif()

# FCL
option(TESSERACT_BUILD_FCL "Build FCL components" ON)
if(TESSERACT_BUILD_FCL)
  message("Building FCL components")
  list(APPEND SUPPORTED_COMPONENTS collision_fcl)
  add_subdirectory(fcl)
endif()

# VHACD
option(TESSERACT_BUILD_VHACD "Build VHACD components" ON)
if(TESSERACT_BUILD_VHACD)
  message("Building VHACD components")
  list(APPEND SUPPORTED_COMPONENTS collision_vhacd)
  add_subdirectory(vhacd)
endif()

# Add compiler definition to core so it can find all plugins produced
string(
  REPLACE ";"
          ":"
          CONTACT_MANAGERS_PLUGINS_STRING
          "${CONTACT_MANAGERS_PLUGINS}")
target_compile_definitions(collision PRIVATE TESSERACT_CONTACT_MANAGERS_PLUGINS="${CONTACT_MANAGERS_PLUGINS_STRING}")

# Testing
if((TESSERACT_ENABLE_TESTING OR TESSERACT_COLLISION_ENABLE_TESTING) AND TESSERACT_BUILD_FCL)
  enable_testing()
  add_subdirectory(test)
endif()

# Benchmarks
if((TESSERACT_ENABLE_BENCHMARKING OR TESSERACT_COLLISION_ENABLE_BENCHMARKING) AND TESSERACT_BUILD_FCL
   AND TESSERACT_BUILD_TEST_SUITE)
  add_subdirectory(test/benchmarks)
endif()

# Examples
if((TESSERACT_ENABLE_EXAMPLES OR TESSERACT_COLLISION_ENABLE_EXAMPLES) AND TESSERACT_BUILD_FCL)
  message("Building examples components")
  list(APPEND SUPPORTED_COMPONENTS collision_examples)
  add_subdirectory(examples)
endif()

# Propagate accumulated components to parent scope
set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} PARENT_SCOPE)
