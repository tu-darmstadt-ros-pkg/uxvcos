set(uxvcos_SOURCES ${uxvcos_SOURCES}
  stream/VectorStream.cpp
  stream/ubx/Stream.cpp
  stream/ubx/Protocol.cpp
)

source_group("stream" REGULAR_EXPRESSION "stream/[^/]+$")
source_group("stream\\ubx" REGULAR_EXPRESSION "stream/ubx/[^/]+$")

