add_executable(pid_motor_test one_pid_motor_test.cpp ../../src/pid_v3.cpp ../../src/pi_controller.cpp ../../src/dc_motor_v2.cpp ../../src/encoder.cpp)
target_include_directories(pid_motor_test PRIVATE ../../include)
target_link_libraries(pid_motor_test pico_stdlib hardware_pwm)
pico_enable_stdio_usb(pid_motor_test 1)
pico_enable_stdio_uart(pid_motor_test 0)

pico_add_extra_outputs(pid_motor_test)