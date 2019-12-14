cmake_minimum_required(VERSION 3.5.0)
project(tesseract_kinematics VERSION 0.1.0 LANGUAGES CXX)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(opw_kinematics REQUIRED)

# Create interface for core
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE tesseract::tesseract_scene_graph)
tesseract_target_compile_options(${PROJECT_NAME}_core INTERFACE)
tesseract_clang_tidy(${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_core INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_core SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:tesseract::tesseract_common,INTERFACE_INCLUDE_DIRECTORIES>) #tesseract::tesseract_common Due to bug in catkin, there is an open PR

# Create ikfast kinematics interface
add_library(${PROJECT_NAME}_ikfast INTERFACE)
target_link_libraries(${PROJECT_NAME}_ikfast INTERFACE ${PROJECT_NAME}_core)
tesseract_target_compile_options(${PROJECT_NAME}_ikfast INTERFACE)
tesseract_clang_tidy(${PROJECT_NAME}_ikfast)
target_include_directories(${PROJECT_NAME}_ikfast INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_ikfast SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS})

# Add KDL implementation of tesseract_kinematics
add_library(${PROJECT_NAME}_kdl SHARED
  src/kdl/kdl_fwd_kin_chain.cpp
  src/kdl/kdl_fwd_kin_tree.cpp
  src/kdl/kdl_inv_kin_chain_lma.cpp
  src/kdl/kdl_inv_kin_chain_nr.cpp
)
target_link_libraries(${PROJECT_NAME}_kdl PUBLIC ${PROJECT_NAME}_core tesseract::tesseract_scene_graph orocos-kdl console_bridge)
tesseract_target_compile_options(${PROJECT_NAME}_kdl PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_kdl)
if(KDL_VERSION VERSION_LESS "1.4.0")
    target_compile_definitions(${PROJECT_NAME}_kdl PUBLIC KDL_LESS_1_4_0=ON)
endif()
target_include_directories(${PROJECT_NAME}_kdl PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_kdl SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}) # Orocos did not export target correctly

# Add OPW implementation of tesseract_kinematics
add_library(${PROJECT_NAME}_opw SHARED src/opw/opw_inv_kin.cpp)
target_link_libraries(${PROJECT_NAME}_opw PUBLIC ${PROJECT_NAME}_core tesseract::tesseract_scene_graph opw_kinematics::opw_kinematics console_bridge)
tesseract_target_compile_options(${PROJECT_NAME}_opw PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_opw)
target_include_directories(${PROJECT_NAME}_opw PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_opw SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

tesseract_configure_package(${PROJECT_NAME}_core ${PROJECT_NAME}_kdl ${PROJECT_NAME}_ikfast ${PROJECT_NAME}_opw)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

if (TESSERACT_ENABLE_TESTING)
  enable_testing()
  tesseract_add_run_tests_target()
  add_subdirectory(test)
endif()
