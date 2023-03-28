add_executable(master master.cpp ../../src/pid_filter.cpp ../../src/dc_motor_v2.cpp ../../src/encoder.cpp ../../src/pid_controller.cpp)
target_include_directories(master PRIVATE ../../include)
target_link_libraries(master pico_stdlib hardware_pwm hardware_adc hardware_i2c pico_multicore)
pico_enable_stdio_usb(master 1)
pico_enable_stdio_uart(master 0)

pico_add_extra_outputs(master)