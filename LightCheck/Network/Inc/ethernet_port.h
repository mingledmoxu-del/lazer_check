#ifndef ETHERNET_PORT_H
#define ETHERNET_PORT_H
#include "lwip/netif.h"
#include "lwip/err.h"
err_t Ethernet_Init(struct netif *netif);
void Ethernet_Poll(struct netif *netif);
int Ethernet_Link(struct netif *netif);
#endif
