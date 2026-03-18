cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(gearbox_embedded C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Automatically find every .c file in this folder
file(GLOB SOURCES "*.c")

# Build each .c file as its own separate program
foreach(SOURCE ${SOURCES})
    get_filename_component(EXEC_NAME ${SOURCE} NAME_WE)
    add_executable(${EXEC_NAME} ${SOURCE})
    target_link_libraries(${EXEC_NAME} pico_stdlib)
    pico_add_extra_outputs(${EXEC_NAME})
    pico_enable_stdio_usb(${EXEC_NAME} 1)
    pico_enable_stdio_uart(${EXEC_NAME} 0)
endforeach()