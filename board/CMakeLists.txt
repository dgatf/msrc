cmake_minimum_required(VERSION 3.13.0)

include(pico_sdk_import.cmake)

project(MSRC-RP2040)

pico_sdk_init()

add_subdirectory(freertos)
add_subdirectory(project)

#target_compile_definitions(MSRC-RP2040 PUBLIC
#    #PICO_FLASH_SPI_CLKDIV=4
#	PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
#)