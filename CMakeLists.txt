## Copyright (c) 2019-2022 Advanced Micro Devices, Inc.
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

project(amd-dbgapi VERSION 0.68.0)

include(GNUInstallDirs)
# Convert the project's name to uppercase and replace '-' with '_'.
string(TOUPPER "${PROJECT_NAME}" AMD_DBGAPI_NAME)
string(REPLACE "-" "_" AMD_DBGAPI_NAME ${AMD_DBGAPI_NAME})

string(TOUPPER "${CMAKE_BUILD_TYPE}" uppercase_CMAKE_BUILD_TYPE)

# Add an option to enable assertions even in non-debug builds.
if(NOT uppercase_CMAKE_BUILD_TYPE STREQUAL "DEBUG")
  option(ENABLE_ASSERTIONS "Enable assertions" OFF)
else()
  option(ENABLE_ASSERTIONS "Enable assertions" ON)
endif()

if(ENABLE_ASSERTIONS)
  if(NOT uppercase_CMAKE_BUILD_TYPE STREQUAL "DEBUG")
    # Add -UNDEBUG to the CFLAGS and CXXFLAGS
    add_compile_options(
      $<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:-UNDEBUG>)
    # Remove -DNDEBUG from the CFLAGS and CXXFLAGS
    foreach (flags
             CMAKE_C_FLAGS_RELEASE
             CMAKE_C_FLAGS_RELWITHDEBINFO
             CMAKE_C_FLAGS_MINSIZEREL
             CMAKE_CXX_FLAGS_RELEASE
             CMAKE_CXX_FLAGS_RELWITHDEBINFO
             CMAKE_CXX_FLAGS_MINSIZEREL)
      string (REGEX REPLACE "(^| )-D *NDEBUG($| )" " " "${flags}" "${${flags}}")
     endforeach()
  endif()
else()
  if(uppercase_CMAKE_BUILD_TYPE STREQUAL "DEBUG")
    add_compile_definitions(NDEBUG)
  endif()
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/amd-dbgapi.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/amd-dbgapi/amd-dbgapi.h @ONLY)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/exportmap.in
  ${CMAKE_CURRENT_BINARY_DIR}/src/exportmap @ONLY)

if(NOT CPACK_PACKAGING_INSTALL_PREFIX)
  set(CPACK_PACKAGING_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/amd-dbgapi.pc.in
  ${CMAKE_CURRENT_BINARY_DIR}/share/pkgconfig/amd-dbgapi.pc @ONLY)

file(GLOB SOURCES "src/*.cpp")
add_library(amd-dbgapi SHARED ${SOURCES})

set_target_properties(amd-dbgapi PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS ON
  CXX_VISIBILITY_PRESET hidden
  OUTPUT_NAME "rocm-dbgapi"
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
  DEFINE_SYMBOL "AMD_DBGAPI_EXPORTS"
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR})

target_compile_options(amd-dbgapi PRIVATE
  -fno-rtti -Werror -Wall -Wextra -Wshadow -Wno-attributes) #-pedantic)

target_compile_definitions(amd-dbgapi
  PRIVATE _GNU_SOURCE __STDC_LIMIT_MACROS __STDC_CONSTANT_MACROS)

check_include_file(backtrace.h BACKTRACE_H)
if(BACKTRACE_H)
  target_compile_definitions(amd-dbgapi PRIVATE HAVE_BACKTRACE_H)
  find_library(BACKTRACE_LIB "backtrace" ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES})
endif()

if(BACKTRACE_LIB)
  target_compile_definitions(amd-dbgapi PRIVATE ENABLE_BACKTRACE)
  target_link_libraries(amd-dbgapi PRIVATE ${BACKTRACE_LIB})
endif()

option(API_TRACING "Enable API tracing" ON)
if(API_TRACING)
  target_compile_definitions(amd-dbgapi PRIVATE WITH_API_TRACING)
endif()

find_package(amd_comgr REQUIRED CONFIG
  PATHS
    /opt/rocm/
  PATH_SUFFIXES
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

target_link_libraries(amd-dbgapi PRIVATE amd_comgr Threads::Threads ${CMAKE_DL_LIBS})

set_source_files_properties(src/versioning.cpp src/initialization.cpp PROPERTIES
  COMPILE_DEFINITIONS "AMD_DBGAPI_VERSION_PATCH=${PROJECT_VERSION_PATCH};AMD_DBGAPI_BUILD_INFO=\"${PROJECT_VERSION}-${build_info}\"")

# We are using the HSA runtime headers, but not the runtime library. Get the HSA runtime
# include directories from the hsa-runtime64::hsa-runtime64 interface.
find_package(hsa-runtime64 REQUIRED CONFIG PATHS "/opt/rocm")
get_property(HSA_RUNTIME_INCLUDE_DIRECTORIES TARGET hsa-runtime64::hsa-runtime64 PROPERTY INTERFACE_INCLUDE_DIRECTORIES)

target_include_directories(amd-dbgapi
  PRIVATE
    ${HSA_RUNTIME_INCLUDE_DIRECTORIES}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include/amd-dbgapi>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(amd-dbgapi
  PRIVATE -Wl,--version-script=${CMAKE_CURRENT_BINARY_DIR}/src/exportmap -Wl,--no-undefined)

set(AMD_DBGAPI_CONFIG_NAME amd-dbgapi-config.cmake)
set(AMD_DBGAPI_TARGETS_NAME amd-dbgapi-targets.cmake)
set(AMD_DBGAPI_PACKAGE_PREFIX ${CMAKE_INSTALL_LIBDIR}/cmake/amd-dbgapi)

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
  DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/include/amd-dbgapi/amd-dbgapi.h"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/amd-dbgapi)

install (FILES
  "${CMAKE_CURRENT_BINARY_DIR}/share/pkgconfig/amd-dbgapi.pc"
  DESTINATION ${CMAKE_INSTALL_DATADIR}/pkgconfig)

install(FILES
  "LICENSE.txt"
  DESTINATION ${CMAKE_INSTALL_DOCDIR})

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
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_CONTACT "ROCm Debugger Support <rocm-gdb.support@amd.com>")
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

set(CPACK_DEBIAN_PACKAGE_DEPENDS "comgr(>=1.2.0), rocm-core")

if(DEFINED ENV{CPACK_RPM_PACKAGE_RELEASE})
  set(CPACK_RPM_PACKAGE_RELEASE $ENV{CPACK_RPM_PACKAGE_RELEASE})
else()
  set(CPACK_RPM_PACKAGE_RELEASE "local")
endif()
set(CPACK_RPM_PACKAGE_REQUIRES "rocm-core")
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

# Remove dependency on rocm-core if -DROCM_DEP_ROCMCORE=ON not given to cmake
if(NOT ROCM_DEP_ROCMCORE)
    string(REGEX REPLACE ",? ?rocm-core" "" CPACK_RPM_PACKAGE_REQUIRES ${CPACK_RPM_PACKAGE_REQUIRES})
    string(REGEX REPLACE ",? ?rocm-core" "" CPACK_DEBIAN_PACKAGE_DEPENDS ${CPACK_DEBIAN_PACKAGE_DEPENDS})
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
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/amd-dbgapi.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/amd-dbgapi/amd-dbgapi.h
    COMMENT "Generating documentation")

  add_custom_target(doc DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/doc/html/index.html
    ${CMAKE_CURRENT_BINARY_DIR}/doc/latex/refman.pdf)

  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/doc/latex/refman.pdf"
    DESTINATION ${CMAKE_INSTALL_DOCDIR}
    RENAME "amd-dbgapi.pdf"
    OPTIONAL)

  install(DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/doc/html/"
    DESTINATION ${CMAKE_INSTALL_DATADIR}/html/amd-dbgapi
    OPTIONAL)

endif()
