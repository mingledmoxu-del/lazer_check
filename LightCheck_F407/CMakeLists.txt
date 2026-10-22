cmake_minimum_required(VERSION 3.22)
project(LightCheck_F407 LANGUAGES C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
file(GLOB APP_SOURCES CONFIGURE_DEPENDS Core/Src/*.c Communication/Src/*.c Motor_driver/Src/*.c)
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS Drivers/STM32F4xx_HAL_Driver/Src/*.c)
add_executable(${PROJECT_NAME} ${APP_SOURCES} ${HAL_SOURCES} startup_stm32f407xx.s)
target_include_directories(${PROJECT_NAME} PRIVATE Core/Inc Communication/Inc Motor_driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc Drivers/CMSIS/Include Drivers/CMSIS/Device/ST/STM32F4xx/Include)
target_compile_definitions(${PROJECT_NAME} PRIVATE USE_HAL_DRIVER STM32F407xx)
target_link_libraries(${PROJECT_NAME} PRIVATE m)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>)
