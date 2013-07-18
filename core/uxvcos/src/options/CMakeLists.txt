set(uxvcos_SOURCES ${uxvcos_SOURCES}
  options/options.cpp
)

set(uxvcos_LIBRARIES ${uxvcos_LIBRARIES}
   ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

source_group("options" REGULAR_EXPRESSION "options/[^/]+$")
