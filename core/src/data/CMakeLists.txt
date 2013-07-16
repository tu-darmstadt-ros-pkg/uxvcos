set(uxvcos_SOURCES ${uxvcos_SOURCES}
  data/TypeInfo.h
  data/TypeInfo.cpp
  data/TypeInfoGenerator.h
  data/StreamableTypeInfo.h
  data/StreamableHandler.h
  data/StreamableCompositionFactory.h
  data/UnknownTypeInfo.h
  data/Key.h
  data/Key.cpp
  data/Timestamp.h
  data/Timestamp.cpp
  data/Streamable.h
  data/Streamable.cpp
  data/StreamableValue.h
  data/StreamableVector.h
  data/StreamableTemplates.h
  data/StreamableContainer.h
  data/TypeRegistry.h
  data/TypeRegistry.cpp
  data/Typekit.h
)

list(APPEND uxvcos_LIBRARIES rt)
source_group("data" REGULAR_EXPRESSION "data/[^/]+$")

uxvcos_typekit(uxvcos-types data/uxvcos-types.cpp data/uxvcos-types.h)
