cmake_minimum_required(VERSION 3.17.0)

include(pico_sdk_import.cmake)

project(MSRC-RP2040)

pico_sdk_init()

add_subdirectory(freertos)
add_subdirectory(project)
