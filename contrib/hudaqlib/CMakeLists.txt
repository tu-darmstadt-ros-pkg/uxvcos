include_directories(${UXVCOS_PATH}/contrib/hudaqlib)
add_library(hudaqlib SHARED IMPORTED)
set_target_properties(hudaqlib PROPERTIES
  IMPORTED_LOCATION "${UXVCOS_PATH}/contrib/hudaqlib/libhudaqlib.so"
)
