add_executable(encoder_test encoder_test.cpp ../src/encoder.cpp)
target_include_directories(encoder_test PRIVATE ../include)
target_link_libraries(encoder_test pico_stdlib)
pico_enable_stdio_usb(encoder_test 1)
pico_enable_stdio_uart(encoder_test 0)

pico_add_extra_outputs(encoder_test)