add_executable(adc_test adc_test.cpp)
target_include_directories(adc_test PRIVATE ../../include)
target_link_libraries(adc_test pico_stdlib hardware_adc hardware_pwm)
pico_enable_stdio_usb(adc_test 1)
pico_enable_stdio_uart(adc_test 0)

pico_add_extra_outputs(adc_test)