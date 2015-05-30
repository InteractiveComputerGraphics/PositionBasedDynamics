# TODO: integrate with AntTweakBar.cmake

find_path(ANTTWEAKBAR_INCLUDE_DIR
  HINTS
  /usr/include
  /usr/local/include
  ${ANTTWEAKBAR_INCLUDE_DIRS}
  NAMES AntTweakBar.h
  DOC "AntTweakBar header"
  )
if( ANTTWEAKBAR_INCLUDE_DIR )
  list( REMOVE_DUPLICATES ANTTWEAKBAR_INCLUDE_DIR )
endif( ANTTWEAKBAR_INCLUDE_DIR )

find_library(ANTTWEAKBAR_LIBRARIES ${ANTTWEAKBAR_LIB_NAME}
  HINTS
  /usr/lib
  /usr/local/lib
  ${ANTTWEAKBAR_LIB_DIRS}
  )

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( AntTweakBar DEFAULT_MSG ANTTWEAKBAR_LIBRARIES ANTTWEAKBAR_INCLUDE_DIR)