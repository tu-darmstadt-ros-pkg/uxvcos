#ifndef DATA_UXVCOS_TYPES_H
#define DATA_UXVCOS_TYPES_H

#include <uxvcos/data/Typekit.h>
#include <uxvcos/data/StreamableTemplates.h>
#include <uxvcos/data/Timestamp.h>

UXVCOS_DECLARE_TYPEKIT(uxvcos);
UXVCOS_DECLARE_TYPE(Data::StreamableVector<double>);
UXVCOS_DECLARE_TYPE(Data::StreamableVector<int>);
UXVCOS_DECLARE_TYPE(Data::StreamableVector<unsigned short>);
UXVCOS_DECLARE_TYPE(Data::StreamableValue<double>);
UXVCOS_DECLARE_TYPE(Data::StreamableValue<int>);
UXVCOS_DECLARE_TYPE(Data::StreamableValue<unsigned short>);
UXVCOS_DECLARE_TYPE(Data::Timestamp);

#endif // DATA_UXVCOS_TYPES_H
