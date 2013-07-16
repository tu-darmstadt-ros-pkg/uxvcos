set(controller_quadro_SOURCES
    Quadrotor.cpp
    Quadrotor.h
    Altimeter.cpp
    Altimeter.h

    Position.h
    Position.cpp
    Velocity.h
    Velocity.cpp
    Attitude.h
    Attitude.cpp
    Heading.h
    Heading.cpp
    Height.h
    Height.cpp
    Thrust.h
    Thrust.cpp
    Force.h
    Force.cpp
    Motor.h
    Motor.cpp
    Test.h
    Test.cpp

    wrench_operations.h
)

orocos_use_package(uxvcos-controller-common)
orocos_component(controller-quadro ${controller_quadro_SOURCES})
set_target_properties(controller-quadro PROPERTIES COMPILE_DEFINITIONS "CONTROLLER_QUADRO")
