cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME LightCheck)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
option(PC_ETHERNET "Use CN2 main Ethernet port instead of UART1" ON)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Communication/Src/crc16.c
    Communication/Src/axis_motion.c
    Communication/Src/axis_guard.c
    Communication/Src/axis_guard_controller.c
    Communication/Src/trajectory.c
    Communication/Src/limit_switch.c
    Communication/Src/limit_debug.c
    Communication/Src/pc_protocol.c
    Communication/Src/pc_comm.c
    Communication/Src/command_manager.c
    Communication/Src/rs485.c
    Communication/Src/modbus_rtu.c
    Motor_driver/Src/p100s.c
    Motor_driver/Src/p100s_bench.c
    Motor_driver/Src/p100s_console.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Communication/Inc
    Motor_driver/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

file(GLOB LWIP_CORE CONFIGURE_DEPENDS Middlewares/Third_Party/LwIP/src/core/*.c)
file(GLOB LWIP_IPV4 CONFIGURE_DEPENDS Middlewares/Third_Party/LwIP/src/core/ipv4/*.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${LWIP_CORE} ${LWIP_IPV4}
    Middlewares/Third_Party/LwIP/src/netif/ethernet.c
    Network/Src/ethernet_port.c Network/Src/network.c Network/Src/pc_comm_net.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_eth.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_eth_ex.c)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE Network/Inc Middlewares/Third_Party/LwIP/src/include)
target_include_directories(STM32_Drivers PRIVATE Network/Inc Middlewares/Third_Party/LwIP/src/include)
if(PC_ETHERNET)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE PC_USE_ETHERNET=1)
else()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE PC_USE_ETHERNET=0)
endif()
