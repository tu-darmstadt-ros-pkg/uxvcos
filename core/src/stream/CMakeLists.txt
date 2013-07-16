set(uxvcos_SOURCES ${uxvcos_SOURCES}
  stream/Stream.h
  stream/Stream.cpp
  stream/Buffer.h
  stream/Chunk.h

  stream/ubx/Stream.h
  stream/ubx/Stream.cpp
  stream/ubx/Protocol.h
  stream/ubx/Protocol.cpp

  stream/helper/PropertyStream.h
  stream/helper/PropertyStream.cpp
  stream/helper/VectorStream.h
  stream/helper/VectorStream.cpp
  stream/helper/StreamMapper.h
)

source_group("stream" REGULAR_EXPRESSION "stream/[^/]+$")
source_group("stream\\ubx" REGULAR_EXPRESSION "stream/ubx/[^/]+$")
source_group("stream\\helper" REGULAR_EXPRESSION "stream/helper/[^/]+$")

