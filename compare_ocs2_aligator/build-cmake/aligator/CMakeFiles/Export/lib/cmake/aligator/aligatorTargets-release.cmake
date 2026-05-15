#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "aligator::aligator" for configuration "Release"
set_property(TARGET aligator::aligator APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(aligator::aligator PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libaligator.so.0.18.0"
  IMPORTED_SONAME_RELEASE "libaligator.so.0.18.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS aligator::aligator )
list(APPEND _IMPORT_CHECK_FILES_FOR_aligator::aligator "${_IMPORT_PREFIX}/lib/libaligator.so.0.18.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
