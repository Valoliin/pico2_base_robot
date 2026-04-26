# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(robot C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
set(SOURCES
    src/robot.cpp
    src/lib/mks_servo/mks_servo.c
    src/lib/paa5100/pmw3901.cpp
    src/odom.cpp
    src/pico_uart_transport.c
)

add_executable(robot ${SOURCES}) 

pico_set_program_name(robot "robot")
pico_set_program_version(robot "0.5")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(robot 0)
pico_enable_stdio_usb(robot 0)

# Add the standard library to the build
target_link_libraries(robot
        pico_stdlib
        hardware_spi
        hardware_uart
        hardware_clocks
        pico_multicore
        ${CMAKE_CURRENT_SOURCE_DIR}/libmicroros/libmicroros.a
        
)

# Add the standard include files to the build
target_include_directories(robot PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/libmicroros/include
        ${CMAKE_CURRENT_LIST_DIR}
)
# On force l'UART0 sur les pins GP0 et GP1 (Broches physiques 1 et 2)
target_compile_definitions(robot PRIVATE
    PICO_DEFAULT_UART=0
    PICO_DEFAULT_UART_TX_PIN=12
    PICO_DEFAULT_UART_RX_PIN=13
    MICRO_ROS_TRANSPORT_UART=uart0
)
pico_add_extra_outputs(robot)

