# set variables
set(AMD_DBGAPI_NAME "AMD_DBGAPI")
set(PROJECT_VERSION_MAJOR "0")
set(PROJECT_VERSION_MINOR "68")
set(PROJECT_VERSION_PATCH "0")

# configure input header file
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/../include/amd-dbgapi.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/../include/amd-dbgapi/amd-dbgapi.h @ONLY)

# set input and output files
set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doxygen/Doxyfile.in)
set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/doxygen/Doxyfile)

# configure Doxyfile
configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
