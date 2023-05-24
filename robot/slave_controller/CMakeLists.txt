add_executable(slave slave_V3.cpp ../../src/pid_v3.cpp ../../src/dc_motor_v2.cpp ../../src/encoder.cpp)
target_include_directories(slave PRIVATE ../../include)
target_link_libraries(slave pico_stdlib hardware_pwm hardware_i2c pico_multicore)
pico_enable_stdio_usb(slave 1)
pico_enable_stdio_uart(slave 0)

pico_add_extra_outputs(slave)