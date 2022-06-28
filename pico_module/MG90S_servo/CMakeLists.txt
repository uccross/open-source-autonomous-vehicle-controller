cmake_minimum_required(VERSION 3.13)

# Need to set SDK file path here
include("C:/Users/Bhumil Depani/pico/Downloads/pico-sdk/external/pico_sdk_import.cmake")

# Set name of the project (as PROJECT_NAME) and C/C++ Standard
project(MG90S_servo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creats a pico-sdk library in our project 
pico_sdk_init()

add_executable(${PROJECT_NAME}
        MG90S_servo.c
        )

# pull in common dependencies; libraries we included into our C Prohram
target_link_libraries(${PROJECT_NAME}
        hardware_pwm
        pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

# add url via pico_set_program_url
# example_auto_set_url(blink)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)