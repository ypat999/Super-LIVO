#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "super_livo_visual_b::super_livo_visual_b" for configuration "Release"
set_property(TARGET super_livo_visual_b::super_livo_visual_b APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(super_livo_visual_b::super_livo_visual_b PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libsuper_livo_visual_b.so"
  IMPORTED_SONAME_RELEASE "libsuper_livo_visual_b.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS super_livo_visual_b::super_livo_visual_b )
list(APPEND _IMPORT_CHECK_FILES_FOR_super_livo_visual_b::super_livo_visual_b "${_IMPORT_PREFIX}/lib/libsuper_livo_visual_b.so" )

# Import target "super_livo_visual_b::visual_b_node" for configuration "Release"
set_property(TARGET super_livo_visual_b::visual_b_node APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(super_livo_visual_b::visual_b_node PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/super_livo_visual_b/visual_b_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS super_livo_visual_b::visual_b_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_super_livo_visual_b::visual_b_node "${_IMPORT_PREFIX}/lib/super_livo_visual_b/visual_b_node" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
