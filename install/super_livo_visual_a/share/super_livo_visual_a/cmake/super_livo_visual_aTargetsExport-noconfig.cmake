#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "super_livo_visual_a::super_livo_visual_a" for configuration ""
set_property(TARGET super_livo_visual_a::super_livo_visual_a APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(super_livo_visual_a::super_livo_visual_a PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsuper_livo_visual_a.so"
  IMPORTED_SONAME_NOCONFIG "libsuper_livo_visual_a.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS super_livo_visual_a::super_livo_visual_a )
list(APPEND _IMPORT_CHECK_FILES_FOR_super_livo_visual_a::super_livo_visual_a "${_IMPORT_PREFIX}/lib/libsuper_livo_visual_a.so" )

# Import target "super_livo_visual_a::visual_a_node" for configuration ""
set_property(TARGET super_livo_visual_a::visual_a_node APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(super_livo_visual_a::visual_a_node PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/super_livo_visual_a/visual_a_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS super_livo_visual_a::visual_a_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_super_livo_visual_a::visual_a_node "${_IMPORT_PREFIX}/lib/super_livo_visual_a/visual_a_node" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
