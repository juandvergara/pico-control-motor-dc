add_executable(motor_test motor_test.cpp ../src/dc_motor.cpp)
target_include_directories(motor_test PRIVATE ../include)
target_link_libraries(motor_test pico_stdlib hardware_pwm)

pico_add_extra_outputs(motor_test)