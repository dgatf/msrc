# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/tmp/workspace/dgatf/msrc/build-validate/_deps/pico_sdk-src")
  file(MAKE_DIRECTORY "/tmp/workspace/dgatf/msrc/build-validate/_deps/pico_sdk-src")
endif()
file(MAKE_DIRECTORY
  "/tmp/workspace/dgatf/msrc/build-validate/_deps/pico_sdk-build"
  "/tmp/workspace/dgatf/msrc/build-validate/_deps/pico_sdk-subbuild/pico_sdk-populate-prefix"
  "/tmp/workspace/dgatf/msrc/build-validate/_deps/pico_sdk-subbuild/pico_sdk-populate-prefix/tmp"
  "/tmp/workspace/dgatf/msrc/build-validate/_deps/pico_sdk-subbuild/pico_sdk-populate-prefix/src/pico_sdk-populate-stamp"
  "/tmp/workspace/dgatf/msrc/build-validate/_deps/pico_sdk-subbuild/pico_sdk-populate-prefix/src"
  "/tmp/workspace/dgatf/msrc/build-validate/_deps/pico_sdk-subbuild/pico_sdk-populate-prefix/src/pico_sdk-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/tmp/workspace/dgatf/msrc/build-validate/_deps/pico_sdk-subbuild/pico_sdk-populate-prefix/src/pico_sdk-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/tmp/workspace/dgatf/msrc/build-validate/_deps/pico_sdk-subbuild/pico_sdk-populate-prefix/src/pico_sdk-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
