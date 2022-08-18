cmake_minimum_required(VERSION 3.13)

# Need to set SDK file path here
include("C:/Users/Bhumil Depani/pico/Downloads/pico-sdk/external/pico_sdk_import.cmake")

# Set name of the project (as PROJECT_NAME) and C/C++ Standard
project(cross-project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creats a pico-sdk library in our project 
pico_sdk_init()

add_executable(${PROJECT_NAME}
        main.c
        range_at_angle/range_at_angle.c
        constant_panning_mode/constant_panning_mode.c
        MG90S_servo/MG90S_servo.c
        V3HP_lidar/V3HP_lidar.c
        AS5047D_encoder/AS5047D_encoder.c
        )

# pull in common dependencies; libraries we included into our C Prohram
target_link_libraries(${PROJECT_NAME}
        hardware_spi
        hardware_i2c
        hardware_pwm
        hardware_uart
        pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

# add url via pico_set_program_url
# example_auto_set_url(blink)


pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)