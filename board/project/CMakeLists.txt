add_executable(${PROJECT_NAME}
        main.c
        esc_hw3.c
        uart.c
        common.c
        voltage.c
        ibus.c
        led.c
        esc_hw4.c
        current.c
        ntc.c
        airspeed.c
        esc_kontronik.c
        esc_apd_f.c
        esc_apd_hv.c
        nmea.c
        vspeed.c
        distance.c
        config.c
        esc_pwm.c
        bmp280.c
        ms5611.c
        bmp180.c
        cell_count.c
        auto_offset.c
        frsky_d.c
        multiplex.c
        smartport.c
        jetiex.c
        sbus.c
        hitec.c
        xbus.c
        sim_rx.c
        capture_edge.c
        i2c_multi.c
        uart_rx.c
        uart_pio.c
        esc_castle.c
        usb.c
        castle_link.c
        srxl.c
        pwm_out.c
        ws2812.c
)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/capture_edge.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/i2c_multi.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/uart_rx.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/castle_link.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

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
)

pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_usb(${PROJECT_NAME} 1) # 1 normal, 0 probe debug 
pico_enable_stdio_uart(${PROJECT_NAME} 0) # 0 normal, 1 probe debug
