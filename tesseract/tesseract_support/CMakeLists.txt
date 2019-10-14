cmake_minimum_required(VERSION 3.5.0)
project(tesseract_support VERSION 0.1.0)

find_package(tesseract_common REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE)
target_compile_definitions(${PROJECT_NAME} INTERFACE TESSERACT_SUPPORT_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

tesseract_configure_package(${PROJECT_NAME})

foreach(dir urdf meshes)
   install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()
