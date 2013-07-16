set(UPLOAD_MACHINE "" CACHE STRING "Target machine for uploading binaries")
set(UPLOAD_PATH "" CACHE STRING "Target path for uploading binaries")
set(UPLOAD_USER "" CACHE STRING "User name on target machine")

get_target_property(TARGET_FILENAME ${TARGET_NAME} OUTPUT_NAME)
get_target_property(TARGET_DIRECTORY ${TARGET_NAME} RUNTIME_OUTPUT_DIRECTORY)

if(NOT TARGET_FILENAME)
  return()
endif(NOT TARGET_FILENAME)

add_custom_target(upload-${TARGET_NAME}
  COMMAND scp -C ${TARGET_FILENAME} "${UPLOAD_USER}@${UPLOAD_MACHINE}:${UPLOAD_PATH}"
  WORKING_DIRECTORY ${TARGET_DIRECTORY}
)
add_dependencies(upload-${TARGET_NAME} ${TARGET_NAME})
