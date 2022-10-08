add_executable(multicore multicore.cpp)
target_include_directories(multicore PRIVATE ../include)
target_link_libraries(multicore pico_stdlib pico_multicore)
pico_enable_stdio_usb(multicore 1)
pico_enable_stdio_uart(multicore 0)

pico_add_extra_outputs(multicore)