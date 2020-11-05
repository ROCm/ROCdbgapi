## Copyright (c) 2019-2020 Advanced Micro Devices, Inc.
##
## Permission is hereby granted, free of charge, to any person obtaining a copy
## of this software and associated documentation files (the "Software"), to
## deal in the Software without restriction, including without limitation the
## rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
## sell copies of the Software, and to permit persons to whom the Software is
## furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in
## all copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
## AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
## FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
## IN THE SOFTWARE.

cmake_minimum_required(VERSION 3.8)

project(AMD_DBGAPI VERSION 0.42.0)

# The project command does not set the <PROJECT-NAME>_NAME variable so must set
# it explicitly.
set(AMD_DBGAPI_NAME "${PROJECT_NAME}")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/amd-dbgapi.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/amd-dbgapi.h @ONLY)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/exportmap.in
  ${CMAKE_CURRENT_BINARY_DIR}/src/exportmap @ONLY)

file(GLOB SOURCES "src/*.cpp")
add_library(amd-dbgapi SHARED ${SOURCES})

set_target_properties(amd-dbgapi PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  OUTPUT_NAME "rocm-dbgapi"
  DEFINE_SYMBOL "AMD_DBGAPI_EXPORTS"
  VERSION ${AMD_DBGAPI_VERSION}
  SOVERSION ${AMD_DBGAPI_VERSION_MAJOR})

target_compile_options(amd-dbgapi PRIVATE -Werror -Wall)

check_include_file(backtrace.h BACKTRACE_H)
if(BACKTRACE_H)
  add_definitions(-DHAVE_BACKTRACE_H)
  find_library(BACKTRACE_LIB "backtrace" ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES})
endif()

if(BACKTRACE_LIB)
  add_definitions(-DENABLE_BACKTRACE)
  target_link_libraries(amd-dbgapi PRIVATE ${BACKTRACE_LIB})
endif()

find_package(amd_comgr REQUIRED CONFIG
  PATHS
    /opt/rocm/
  PATH_SUFFIXES
    cmake/amd_comgr
    lib/cmake/amd_comgr
)
MESSAGE(STATUS "Code Object Manager found at ${amd_comgr_DIR}.")

if(DEFINED ENV{ROCM_BUILD_ID})
  # ROCM_BUILD_ID is set by the ROCm-CI build environment.
  set(build_info $ENV{ROCM_BUILD_ID})
else()
  string(TIMESTAMP NOW "%Y%m%dT%H%M%S")
  set(build_info developer-build-${NOW})

  if(DEFINED ENV{USER})
    set(build_info ${build_info}-$ENV{USER})
  endif()

  find_package(Git)
  if(GIT_FOUND)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
      WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
      OUTPUT_VARIABLE build_revision
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE
   )
  else()
    message(STATUS "GIT not found")
  endif()

  if(DEFINED build_revision)
    set(build_info ${build_info}-git-${build_revision})
  endif()
endif()

target_link_libraries(amd-dbgapi PRIVATE amd_comgr Threads::Threads)

target_compile_options(amd-dbgapi
  PRIVATE -fno-rtti -Wall -Wno-attributes -fvisibility=hidden)

target_compile_definitions(amd-dbgapi
  PRIVATE _GNU_SOURCE __STDC_LIMIT_MACROS __STDC_CONSTANT_MACROS)

set_source_files_properties(src/versioning.cpp PROPERTIES
  COMPILE_DEFINITIONS "AMD_DBGAPI_VERSION_PATCH=${AMD_DBGAPI_VERSION_PATCH};AMD_DBGAPI_BUILD_INFO=\"${AMD_DBGAPI_VERSION}-${build_info}\"")

target_include_directories(amd-dbgapi
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(amd-dbgapi
  PRIVATE -Wl,--version-script=${CMAKE_CURRENT_BINARY_DIR}/src/exportmap -Wl,--no-undefined)

set(AMD_DBGAPI_CONFIG_NAME amd-dbgapi-config.cmake)
set(AMD_DBGAPI_TARGETS_NAME amd-dbgapi-targets.cmake)
set(AMD_DBGAPI_PACKAGE_PREFIX lib/cmake/amd-dbgapi)

# Generate the build-tree package.
set(AMD_DBGAPI_PREFIX_CODE)
set(AMD_DBGAPI_TARGETS_PATH
  "${CMAKE_CURRENT_BINARY_DIR}/${AMD_DBGAPI_PACKAGE_PREFIX}/${AMD_DBGAPI_TARGETS_NAME}")
export(TARGETS amd-dbgapi
  FILE "${AMD_DBGAPI_PACKAGE_PREFIX}/${AMD_DBGAPI_TARGETS_NAME}")
configure_file("cmake/${AMD_DBGAPI_CONFIG_NAME}.in"
  "${AMD_DBGAPI_PACKAGE_PREFIX}/${AMD_DBGAPI_CONFIG_NAME}"
  @ONLY)

install(TARGETS amd-dbgapi
  EXPORT amd-dbgapi-export
  DESTINATION lib)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/include/amd-dbgapi.h"
  DESTINATION include)

install(FILES
  "LICENSE.txt"
  DESTINATION share/doc/amd-dbgapi)

# Generate the install-tree package.
set(AMD_DBGAPI_PREFIX_CODE "
# Derive absolute install prefix from config file path.
get_filename_component(AMD_DBGAPI_PREFIX \"\${CMAKE_CURRENT_LIST_FILE}\" PATH)")
string(REGEX REPLACE "/" ";" count "${AMD_DBGAPI_PACKAGE_PREFIX}")
foreach(p ${count})
  set(AMD_DBGAPI_PREFIX_CODE "${AMD_DBGAPI_PREFIX_CODE}
get_filename_component(AMD_DBGAPI_PREFIX \"\${AMD_DBGAPI_PREFIX}\" PATH)")
endforeach()
set(AMD_DBGAPI_TARGETS_PATH "\${AMD_DBGAPI_PREFIX}/${AMD_DBGAPI_PACKAGE_PREFIX}/${AMD_DBGAPI_TARGETS_NAME}")
configure_file("cmake/${AMD_DBGAPI_CONFIG_NAME}.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${AMD_DBGAPI_CONFIG_NAME}.install"
  @ONLY)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${AMD_DBGAPI_CONFIG_NAME}.install"
  DESTINATION "${AMD_DBGAPI_PACKAGE_PREFIX}"
  RENAME "${AMD_DBGAPI_CONFIG_NAME}")
install(EXPORT amd-dbgapi-export
  DESTINATION "${AMD_DBGAPI_PACKAGE_PREFIX}"
  FILE "${AMD_DBGAPI_TARGETS_NAME}")

# Add packaging directives for amd-dbgapi
set(CPACK_PACKAGE_NAME rocm-dbgapi)
set(CPACK_PACKAGE_VENDOR "Advanced Micro Devices, Inc.")
set(CPACK_PACKAGE_VERSION_MAJOR ${AMD_DBGAPI_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${AMD_DBGAPI_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${AMD_DBGAPI_VERSION_PATCH})
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_CONTACT "Advanced Micro Devices, Inc.")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Library to provide AMD GPU debugger API")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt")

if(DEFINED ENV{ROCM_LIBPATCH_VERSION})
  set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION}.$ENV{ROCM_LIBPATCH_VERSION}")
  message("Using CPACK_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION}")
endif()

# Debian package specific variables
if(DEFINED ENV{CPACK_DEBIAN_PACKAGE_RELEASE})
  set(CPACK_DEBIAN_PACKAGE_RELEASE $ENV{CPACK_DEBIAN_PACKAGE_RELEASE})
else()
  set(CPACK_DEBIAN_PACKAGE_RELEASE "local")
endif()
message("Using CPACK_DEBIAN_PACKAGE_RELEASE ${CPACK_DEBIAN_PACKAGE_RELEASE}")
set(CPACK_DEBIAN_FILE_NAME "DEB-DEFAULT")

set(CPACK_DEBIAN_PACKAGE_DEPENDS "comgr(>=1.2.0)")

if(DEFINED ENV{CPACK_RPM_PACKAGE_RELEASE})
  set(CPACK_RPM_PACKAGE_RELEASE $ENV{CPACK_RPM_PACKAGE_RELEASE})
else()
  set(CPACK_RPM_PACKAGE_RELEASE "local")
endif()
message("Using CPACK_RPM_PACKAGE_RELEASE ${CPACK_RPM_PACKAGE_RELEASE}")

## 'dist' breaks manual builds on debian systems due to empty Provides
execute_process(COMMAND rpm --eval %{?dist}
                 RESULT_VARIABLE PROC_RESULT
                 OUTPUT_VARIABLE EVAL_RESULT
                 OUTPUT_STRIP_TRAILING_WHITESPACE)
if(PROC_RESULT EQUAL "0" AND NOT EVAL_RESULT STREQUAL "")
  string(APPEND CPACK_RPM_PACKAGE_RELEASE "%{?dist}")
endif()
set(CPACK_RPM_FILE_NAME "RPM-DEFAULT")

set(CPACK_RPM_PACKAGE_REQUIRES "comgr >= 1.2.0")

# Debian package specific variables
#set(CPACK_DEBIAN_PACKAGE_HOMEPAGE
#  "https://github.com/RadeonOpenCompute/")

# RPM package specific variables
if(DEFINED CPACK_PACKAGING_INSTALL_PREFIX)
  set(CPACK_RPM_EXCLUDE_FROM_AUTO_FILELIST_ADDITION
    "${CPACK_PACKAGING_INSTALL_PREFIX}")
endif()

if(NOT CPack_CMake_INCLUDED)
  include(CPack)
endif()

find_package(Doxygen)
if(DOXYGEN_FOUND)
  # set input and output files
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  # request to configure the file
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/doc/html/index.html ${CMAKE_CURRENT_BINARY_DIR}/doc/latex/refman.pdf
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/doc/latex pdf
    MAIN_DEPENDENCY ${DOXYGEN_OUT} ${DOXYGEN_IN}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/amd-dbgapi.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/amd-dbgapi.h
    COMMENT "Generating documentation")

  add_custom_target(doc DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/doc/html/index.html
    ${CMAKE_CURRENT_BINARY_DIR}/doc/latex/refman.pdf)

  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/doc/latex/refman.pdf"
    DESTINATION share/doc/amd-dbgapi
    RENAME "amd-dbgapi.pdf"
    OPTIONAL)

  install(DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/doc/html/"
    DESTINATION share/html/amd-dbgapi
    OPTIONAL)

endif()
