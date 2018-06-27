#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "djiosdk-core" for configuration "Debug"
set_property(TARGET djiosdk-core APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(djiosdk-core PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "C:/Program Files (x86)/matrice210/lib/libdjiosdk-core.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS djiosdk-core )
list(APPEND _IMPORT_CHECK_FILES_FOR_djiosdk-core "C:/Program Files (x86)/matrice210/lib/libdjiosdk-core.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
