include_directories(. pio protocol sensor)

add_executable(${PROJECT_NAME})

add_subdirectory(protocol)
add_subdirectory(sensor)
add_subdirectory(pio)

target_sources(${PROJECT_NAME} PRIVATE
    main.c
    uart.c
    common.c
    led.c
    config.c
    sim_rx.c
    uart_pio.c
    usb.c
    serial_monitor.c
    dbg_rb.c
    dbg_task.c
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    freertos
    hardware_uart
    hardware_irq
    hardware_pwm
    hardware_clocks
    hardware_adc
    hardware_pio
    hardware_i2c
    hardware_flash
    pico_i2c_slave
)

pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_usb(${PROJECT_NAME} 1) # 1 normal, 0 probe debug 
pico_enable_stdio_uart(${PROJECT_NAME} 0) # 0 normal, 1 probe debug
