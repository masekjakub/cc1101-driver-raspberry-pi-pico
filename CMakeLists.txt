# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

#include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ Standards
project(example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()
add_subdirectory(CC1101)

# point out the CMake, where to find the executable source file
add_executable(${PROJECT_NAME}_send
        example_send.cpp
)

add_executable(${PROJECT_NAME}_recv
        example_recv.cpp
)

# create map/bin/hex/uf2 files.
pico_add_extra_outputs(${PROJECT_NAME}_send)
pico_add_extra_outputs(${PROJECT_NAME}_recv)

# Pull in our pico_stdlib which pulls in commonly used features (gpio, timer-delay etc)
target_link_libraries(${PROJECT_NAME}_send
        CC1101
)

target_link_libraries(${PROJECT_NAME}_recv
        CC1101
)

pico_enable_stdio_usb(${PROJECT_NAME}_send 1)
pico_enable_stdio_uart(${PROJECT_NAME}_send 0)

pico_enable_stdio_usb(${PROJECT_NAME}_recv 1)
pico_enable_stdio_uart(${PROJECT_NAME}_recv 0)