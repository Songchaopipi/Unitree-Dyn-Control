#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ocs2_oc::ocs2_oc" for configuration "Release"
set_property(TARGET ocs2_oc::ocs2_oc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ocs2_oc::ocs2_oc PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libocs2_oc.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS ocs2_oc::ocs2_oc )
list(APPEND _IMPORT_CHECK_FILES_FOR_ocs2_oc::ocs2_oc "${_IMPORT_PREFIX}/lib/libocs2_oc.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
