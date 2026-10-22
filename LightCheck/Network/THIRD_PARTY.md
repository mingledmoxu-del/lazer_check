# Network dependencies

- `Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_eth*.{c,h}`: official STMicroelectronics `stm32h7xx-hal-driver` tag **v1.11.6**, matching the pre-existing HAL version in this project. Downloaded from https://github.com/STMicroelectronics/stm32h7xx-hal-driver/tree/v1.11.6 . License: `Drivers/STM32H7xx_HAL_Driver/LICENSE.md`.
- `Middlewares/Third_Party/LwIP/src`: **lwIP 2.1.2**, copied from the locally installed ST **STM32Cube_FW_F4_V1.28.3** middleware package. This is the portable TCP/IP core, not an F4 Ethernet hardware driver. License: `Middlewares/Third_Party/LwIP/COPYING`.
- `Network/Src/ethernet_port.c`: this project's H743 CN2 board adaptation. It uses the IEEE PHY management registers and the H7 HAL polling receive/transmit API; no DM9000 driver is used.

The source and license files are included in the repository. Firmware builds do not download dependencies.
