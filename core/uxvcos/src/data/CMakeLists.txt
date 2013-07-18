set(uxvcos_SOURCES ${uxvcos_SOURCES}
  data/TypeInfo.cpp
  data/Key.cpp
  data/Timestamp.cpp
  data/Streamable.cpp
  data/TypeRegistry.cpp
)

list(APPEND uxvcos_LIBRARIES rt)
source_group("data" REGULAR_EXPRESSION "data/[^/]+$")
