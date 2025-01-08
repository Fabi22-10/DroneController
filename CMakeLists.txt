cmake_minimum_required(VERSION 3.9)
project(my-sbus-project)

# Set C and C++ standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add subdirectories for dependencies
add_subdirectory(raspberry-sbus libserial)

# Define executable target with source files
add_executable(DroneController 
    Compass.h
    Compass.cpp
    Connector.h
    Connector.cpp
    ControlLoop.cpp
    ControlLoop.h
    GPSModule.h
    GPSModule.cpp
    serialib.cpp
    serialib.h
    main.cpp
)

# Link libraries
target_link_libraries(DroneController 
    PUBLIC libsbus
    pthread
    wiringPi
)
