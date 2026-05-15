#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ocs2_mpc::ocs2_mpc" for configuration "Release"
set_property(TARGET ocs2_mpc::ocs2_mpc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ocs2_mpc::ocs2_mpc PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libocs2_mpc.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS ocs2_mpc::ocs2_mpc )
list(APPEND _IMPORT_CHECK_FILES_FOR_ocs2_mpc::ocs2_mpc "${_IMPORT_PREFIX}/lib/libocs2_mpc.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
