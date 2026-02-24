set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} "support" PARENT_SCOPE)

# Configure Package
configure_component(COMPONENT support)

foreach(dir urdf meshes)
  install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/support/${dir} COMPONENT support)
endforeach()

if(TESSERACT_PACKAGE)
  cpack(COMPONENT support VERSION ${pkg_extracted_version} DESCRIPTION "Tesseract Support Components")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
