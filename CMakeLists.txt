cmake_minimum_required(VERSION 3.12)

# Change your executable name to something creative!
set(NAME pico_remote_car) # <-- Name your project/executable here!

# include(pimoroni_pico_import.cmake)
include(pico_sdk_import.cmake)


# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# enable exceptions for picoled
set(PICO_CXX_ENABLE_EXCEPTIONS 1)
# Initialize the SDK
pico_sdk_init()

## other executables
add_subdirectory(uart_test)
#add_subdirectory(motor_test)
#add_subdirectory(encoder_test)
#add_subdirectory(pid_motor_test)
#add_subdirectory(multicore_test)
#add_subdirectory(i2c_test_master)
#add_subdirectory(i2c_test_slave)
add_subdirectory(test)
add_subdirectory(robot)

# include neopixel lib
# include(PicoLED/PicoLed.cmake)
# Add your source files
add_executable(${NAME}
        src/main.cpp # <-- Add source files here!
        src/dc_motor.cpp
        )

target_include_directories(${NAME} PRIVATE include)

# Don't forget to link the libraries you need!
target_link_libraries(${NAME}
        pico_stdlib
        hardware_uart
        hardware_pwm
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)

## Set up files for the release packages
#install(FILES
#        ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.uf2
#        ${CMAKE_CURRENT_LIST_DIR}/README.md
#        DESTINATION .
#        )
#
#set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
#set(CPACK_GENERATOR "ZIP" "TGZ")
#include(CPack)
