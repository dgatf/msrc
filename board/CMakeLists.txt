cmake_minimum_required(VERSION 3.17.0)

include(pico_sdk_import.cmake)

project(MSRC-RP2040)

execute_process(
    COMMAND git config --global --add safe.directory /github/workspace
    COMMAND git describe --tags
    OUTPUT_VARIABLE PROJECT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS "MSRC ${PROJECT_VERSION}") 
add_compile_definitions(PROJECT_VERSION="${PROJECT_VERSION}")

pico_sdk_init()

add_subdirectory(freertos)
add_subdirectory(project)
