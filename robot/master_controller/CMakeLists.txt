add_executable(master master.cpp ../../src/pi_controller.cpp ../../src/dc_motor_v2.cpp ../../src/encoder.cpp)
target_include_directories(master PRIVATE ../../include)
target_link_libraries(master pico_stdlib hardware_pwm hardware_i2c)
pico_enable_stdio_usb(master 1)
pico_enable_stdio_uart(master 0)

pico_add_extra_outputs(master)