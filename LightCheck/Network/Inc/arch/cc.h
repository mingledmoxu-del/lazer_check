#ifndef LWIP_PORT_CC_H
#define LWIP_PORT_CC_H
#include <stdint.h>
#include <stdlib.h>
#define BYTE_ORDER LITTLE_ENDIAN
#define PACK_STRUCT_STRUCT __attribute__((packed))
#define LWIP_PLATFORM_DIAG(x) do {} while (0)
void Error_Handler(void);
#define LWIP_PLATFORM_ASSERT(x) do { (void)(x); Error_Handler(); } while (0)
#define LWIP_RAND() ((uint32_t)rand())
#endif
