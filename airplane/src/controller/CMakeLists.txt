add_library(controller-airplane ${LIBRARY_TYPE} Airplane.cpp Airplane.h)
set_property(TARGET controller-airplane PROPERTY COMPILE_DEFINITIONS CONTROLLER_AIRPLANE) 
target_link_libraries(controller-airplane ${UXVCOS_LIBRARIES})
