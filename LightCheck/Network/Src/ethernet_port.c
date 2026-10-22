/* HYKJN1002 CN2 main port: H743 RMII + DM9161/DM9162, 50MHz external REF_CLK.
 * Polling HAL ETH API, no RTOS. DMA memory is in D2 SRAM; D-cache stays disabled. */
#include "ethernet_port.h"
#include "stm32h7xx_hal.h"
#include "lwip/pbuf.h"
#include "lwip/etharp.h"
#include <stddef.h>
#include <string.h>

#define RX_BYTES 1536U
#define RX_SLOTS 8U
#define DMA_MEM __attribute__((section(".eth_dma"),aligned(32)))
static ETH_HandleTypeDef eth;
static ETH_DMADescTypeDef rx_desc[ETH_RX_DESC_CNT] DMA_MEM;
static ETH_DMADescTypeDef tx_desc[ETH_TX_DESC_CNT] DMA_MEM;
typedef struct {
    struct pbuf_custom custom;
    uint8_t used;
    uint8_t bytes[RX_BYTES] __attribute__((aligned(32)));
} rx_slot_t;
static rx_slot_t rx_slots[RX_SLOTS] DMA_MEM;
static uint8_t tx_bytes[RX_BYTES] DMA_MEM;
static uint32_t phy_address=32;
static uint8_t mac[6];
static uint8_t started;
static uint8_t io_failed;

void HAL_ETH_MspInit(ETH_HandleTypeDef *handle)
{
    if (handle->Instance!=ETH) return;
    __HAL_RCC_ETH1MAC_CLK_ENABLE();
    __HAL_RCC_ETH1TX_CLK_ENABLE();
    __HAL_RCC_ETH1RX_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_GPIOG_CLK_ENABLE();
    GPIO_InitTypeDef g={0};
    /* PB12: active-low PHY reset. */
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
    g.Pin=GPIO_PIN_12;g.Mode=GPIO_MODE_OUTPUT_PP;g.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB,&g);HAL_Delay(5);
    g.Mode=GPIO_MODE_AF_PP;g.Pull=GPIO_NOPULL;g.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate=GPIO_AF11_ETH;
    g.Pin=GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;HAL_GPIO_Init(GPIOA,&g);
    g.Pin=GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;HAL_GPIO_Init(GPIOC,&g);
    g.Pin=GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;HAL_GPIO_Init(GPIOG,&g);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);HAL_Delay(50);
}
static void free_rx(struct pbuf *p) { ((rx_slot_t *)p)->used=0; }
void HAL_ETH_RxAllocateCallback(uint8_t **buffer)
{
    *buffer=NULL;
    for (unsigned n=0;n<RX_SLOTS;++n) if (!rx_slots[n].used) {
        rx_slot_t *slot=&rx_slots[n];slot->used=1;
        slot->custom.custom_free_function=free_rx;
        pbuf_alloced_custom(PBUF_RAW,0,PBUF_REF,&slot->custom,slot->bytes,RX_BYTES);
        *buffer=slot->bytes;return;
    }
}
void HAL_ETH_RxLinkCallback(void **start,void **end,uint8_t *buffer,uint16_t length)
{
    rx_slot_t *slot=(rx_slot_t *)(buffer - offsetof(rx_slot_t,bytes));
    struct pbuf *p=&slot->custom.pbuf;
    p->next=NULL;p->len=p->tot_len=length;
    if (*start) pbuf_cat((struct pbuf *)*start,p);else *start=p;
    *end=p;
}
static err_t output(struct netif *netif,struct pbuf *p)
{
    (void)netif;
    if (!started || p->tot_len>RX_BYTES) return ERR_IF;
    pbuf_copy_partial(p,tx_bytes,p->tot_len,0);
    ETH_BufferTypeDef buffer={0};buffer.buffer=tx_bytes;buffer.len=p->tot_len;
    ETH_TxPacketConfigTypeDef config={0};
    config.Attributes=ETH_TX_PACKETS_FEATURES_CRCPAD;
    config.CRCPadCtrl=ETH_CRC_PAD_INSERT;
    config.Length=p->tot_len;config.TxBuffer=&buffer;
    __DSB();
    if (HAL_ETH_Transmit(&eth,&config,10U)==HAL_OK) return ERR_OK;
    io_failed=1;return ERR_IF;
}
static int phy_read(uint32_t reg,uint32_t *value)
{ return HAL_ETH_ReadPHYRegister(&eth,phy_address,reg,value)==HAL_OK; }
err_t Ethernet_Init(struct netif *netif)
{
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    __HAL_RCC_D2SRAM3_CLK_ENABLE();
    memset(rx_slots,0,sizeof(rx_slots));
    uint32_t uid=HAL_GetUIDw0()^HAL_GetUIDw1()^HAL_GetUIDw2();
    mac[0]=2;mac[1]=0x48;mac[2]=(uint8_t)(uid>>24);mac[3]=(uint8_t)(uid>>16);mac[4]=(uint8_t)(uid>>8);mac[5]=(uint8_t)uid;
    eth.Instance=ETH;eth.Init.MACAddr=mac;eth.Init.MediaInterface=HAL_ETH_RMII_MODE;
    eth.Init.TxDesc=tx_desc;eth.Init.RxDesc=rx_desc;eth.Init.RxBuffLen=RX_BYTES;
    if (HAL_ETH_Init(&eth)!=HAL_OK) return ERR_IF;
    for (uint32_t n=0;n<32;++n) {
        uint32_t id1=0,id2=0;
        if (HAL_ETH_ReadPHYRegister(&eth,n,2,&id1)==HAL_OK &&
            HAL_ETH_ReadPHYRegister(&eth,n,3,&id2)==HAL_OK &&
            id1!=0 && id1!=0xffffU && id2!=0xffffU) {phy_address=n;break;}
    }
    if (phy_address>=32) return ERR_IF;
    /* Standard IEEE PHY registers: advertise 10/100 half/full, restart negotiation. */
    if (HAL_ETH_WritePHYRegister(&eth,phy_address,4,0x01E1U)!=HAL_OK ||
        HAL_ETH_WritePHYRegister(&eth,phy_address,0,0x1200U)!=HAL_OK) return ERR_IF;
    netif->name[0]='e';netif->name[1]='0';netif->hwaddr_len=6;
    memcpy(netif->hwaddr,mac,6);netif->mtu=1500;
    netif->flags=NETIF_FLAG_BROADCAST|NETIF_FLAG_ETHARP;
    netif->output=etharp_output;netif->linkoutput=output;
    return ERR_OK;
}
int Ethernet_Link(struct netif *netif)
{
    uint32_t bmsr=0,bmcr=0,local=0,partner=0;
    if (io_failed || (eth.Instance->DMACSR & ETH_DMACSR_FBE)) {io_failed=0;goto down;}
    /* Link is latch-low; read twice before interpreting. */
    if (!phy_read(1,&bmsr) || !phy_read(1,&bmsr) || !(bmsr&4U) || !phy_read(0,&bmcr)) goto down;
    ETH_MACConfigTypeDef config={0};
    if (HAL_ETH_GetMACConfig(&eth,&config)!=HAL_OK) goto down;
    if (bmcr&0x1000U) {
        if (!(bmsr&0x20U) || !phy_read(4,&local) || !phy_read(5,&partner)) goto down;
        uint32_t common=local&partner;
        if (!(common&0x01E0U)) goto down;
        config.Speed=(common&0x0180U) ? ETH_SPEED_100M : ETH_SPEED_10M;
        config.DuplexMode=(common&0x0100U) || (!(common&0x0080U) && (common&0x0040U)) ?
            ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;
    } else {
        config.Speed=(bmcr&0x2000U) ? ETH_SPEED_100M : ETH_SPEED_10M;
        config.DuplexMode=(bmcr&0x0100U) ? ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;
    }
    if (!started) {
        if (HAL_ETH_SetMACConfig(&eth,&config)!=HAL_OK || HAL_ETH_Start(&eth)!=HAL_OK) goto down;
        started=1;netif_set_link_up(netif);
    }
    return 1;
down:
    if (started) {(void)HAL_ETH_Stop(&eth);started=0;}
    netif_set_link_down(netif);return 0;
}
void Ethernet_Poll(struct netif *netif)
{
    if (!started) return;
    /* A receive flood cannot monopolize the motion foreground loop. */
    for (unsigned n=0;n<4;++n) {
        struct pbuf *p=NULL;
        if (HAL_ETH_ReadData(&eth,(void **)&p)!=HAL_OK || !p) break;
        if (netif->input(p,netif)!=ERR_OK) pbuf_free(p);
    }
}
