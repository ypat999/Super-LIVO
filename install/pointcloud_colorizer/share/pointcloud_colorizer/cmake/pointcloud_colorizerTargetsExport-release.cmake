#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pointcloud_colorizer::pointcloud_colorizer" for configuration "Release"
set_property(TARGET pointcloud_colorizer::pointcloud_colorizer APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pointcloud_colorizer::pointcloud_colorizer PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpointcloud_colorizer.so"
  IMPORTED_SONAME_RELEASE "libpointcloud_colorizer.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pointcloud_colorizer::pointcloud_colorizer )
list(APPEND _IMPORT_CHECK_FILES_FOR_pointcloud_colorizer::pointcloud_colorizer "${_IMPORT_PREFIX}/lib/libpointcloud_colorizer.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
