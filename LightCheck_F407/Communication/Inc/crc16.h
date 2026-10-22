#ifndef CRC16_H
#define CRC16_H

#include <stddef.h>
#include <stdint.h>

uint16_t CRC16_Modbus(const uint8_t *data, size_t length);

#endif
