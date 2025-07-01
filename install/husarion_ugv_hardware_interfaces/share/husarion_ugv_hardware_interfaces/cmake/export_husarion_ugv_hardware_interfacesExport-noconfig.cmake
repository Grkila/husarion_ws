#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "husarion_ugv_hardware_interfaces::husarion_ugv_hardware_interfaces" for configuration ""
set_property(TARGET husarion_ugv_hardware_interfaces::husarion_ugv_hardware_interfaces APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(husarion_ugv_hardware_interfaces::husarion_ugv_hardware_interfaces PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libhusarion_ugv_hardware_interfaces.so"
  IMPORTED_SONAME_NOCONFIG "libhusarion_ugv_hardware_interfaces.so"
  )

list(APPEND _cmake_import_check_targets husarion_ugv_hardware_interfaces::husarion_ugv_hardware_interfaces )
list(APPEND _cmake_import_check_files_for_husarion_ugv_hardware_interfaces::husarion_ugv_hardware_interfaces "${_IMPORT_PREFIX}/lib/libhusarion_ugv_hardware_interfaces.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
