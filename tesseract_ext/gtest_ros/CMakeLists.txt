cmake_minimum_required(VERSION 3.5.0)

find_package(GTest QUIET)
if ( NOT ${GTest_FOUND} )
    include(ExternalProject)

    ExternalProject_Add(GTest
      GIT_REPOSITORY    https://github.com/google/googletest.git
      GIT_TAG           release-1.8.1
      SOURCE_DIR        ${CMAKE_BINARY_DIR}/../googletest-src
      BINARY_DIR        ${CMAKE_BINARY_DIR}/../googletest-build
      CMAKE_CACHE_ARGS
              -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
              -DCMAKE_BUILD_TYPE:STRING=Release
              -DBUILD_GMOCK:BOOL=OFF
              -DBUILD_GTEST:BOOL=ON
              -DBUILD_SHARED_LIBS:BOOL=ON
    )
endif()

install(FILES package.xml DESTINATION share/gtest)
