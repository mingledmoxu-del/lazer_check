/* Exercise the production raw-TCP session handling using real lwIP/parser,
 * with a virtual Ethernet link and no board or motor I/O. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../Network/Src/network.c"
#include "lwip/etharp.h"
#include "crc16.h"
static uint32_t tick;
static unsigned commands,stops,diagnostics;
static int abort_on_command,physical_link=1;
uint32_t HAL_GetTick(void) {return tick;}
uint32_t HAL_GetUIDw0(void) {return 1;}
uint32_t HAL_GetUIDw1(void) {return 2;}
uint32_t HAL_GetUIDw2(void) {return 3;}
void Error_Handler(void) {abort();}
static err_t virtual_tx(struct netif *n,struct pbuf *p) {(void)n;(void)p;return ERR_OK;}
err_t Ethernet_Init(struct netif *n)
{
    n->name[0]='t';n->name[1]='0';n->hwaddr_len=6;n->hwaddr[0]=2;n->mtu=1500;
    n->flags=NETIF_FLAG_BROADCAST|NETIF_FLAG_ETHARP;
    n->output=etharp_output;n->linkoutput=virtual_tx;return ERR_OK;
}
int Ethernet_Link(struct netif *n) {if(physical_link)netif_set_link_up(n);else netif_set_link_down(n);return physical_link;}
void Ethernet_Poll(struct netif *n) {(void)n;}
uint8_t Trajectory_Active(const trajectory_t *t) {(void)t;return 0;}
void Command_Manager_TransportLost(command_manager_t *m) {++stops;m->continuous_stop_pending[0]=m->continuous_stop_pending[1]=1;}
void GuardController_Command(command_manager_t *m,const pc_command_packet_t *packet)
{(void)m;assert(packet->command==PC_CMD_READ_DUAL_LIMIT || packet->command==PC_CMD_READ_GUARD);++diagnostics;}
void Command_Process(void *context,const pc_command_packet_t *packet)
{
    (void)context;assert(packet->valid_payload);++commands;
    uint8_t reply[64];
    uint16_t length=PC_Protocol_BuildResponse(reply,sizeof(reply),packet->device_id,
        packet->command,PC_STATUS_SUCCESS,packet->parameter,0);
    if(abort_on_command)client->snd_buf=0; /* exercise production TX-error teardown */
    assert(Network_Send(reply,length)==(abort_on_command ? HAL_ERROR : HAL_OK));
}
static struct tcp_pcb *connection(void)
{
    struct tcp_pcb *p=tcp_new();assert(p);ip_addr_t address;IP_ADDR4(&address,192,168,10,1);
    assert(tcp_connect(p,&address,12345,NULL)==ERR_OK);p->state=ESTABLISHED;
    return p;
}
static void feed(struct tcp_pcb *p,const uint8_t *data,unsigned n)
{
    struct pbuf *buf=pbuf_alloc(PBUF_RAW,(u16_t)n,PBUF_RAM);assert(buf);
    assert(pbuf_take(buf,data,(u16_t)n)==ERR_OK);assert(receive(NULL,p,buf,ERR_OK)==ERR_OK);
}
int main(void)
{
    command_manager_t m={0};pc_protocol_t proto={0};
    assert(Network_Init(&proto,&m)==HAL_OK);Network_Task();
    struct tcp_pcb *owner=connection();assert(accepted(NULL,owner,ERR_OK)==ERR_OK);
    struct tcp_pcb *second=connection();assert(accepted(NULL,second,ERR_OK)==ERR_ABRT && client==owner);
    uint8_t frame[9]={0xaa,0x55,6,1,PC_CMD_READ_SPEED,0,0,0,0};
    uint16_t crc=CRC16_Modbus(frame,7);frame[7]=(uint8_t)crc;frame[8]=(uint8_t)(crc>>8);
    feed(owner,frame,4);Network_Task();assert(commands==0);
    feed(owner,frame+4,5);Network_Task();assert(commands==1);
    uint8_t pair[18];memcpy(pair,frame,9);memcpy(pair+9,frame,9);
    feed(owner,pair,18);Network_Task();assert(commands==3);
    m.trajectory.state=TJ_READY;m.trajectory.session=77;m.trajectory.count=2;
    abort_on_command=1;feed(owner,pair,18);Network_Task();
    assert(commands==4 && stops==1 && !client && stopping && !proto.size);
    abort_on_command=0;
    owner=connection();assert(accepted(NULL,owner,ERR_OK)==ERR_OK);
    /* Reconnection succeeds even if a drive cannot confirm stopped. Reads that
       need bus I/O and every motion stay blocked; RAM diagnostics still work. */
    feed(owner,frame,9);Network_Task();assert(commands==4 && stopping);
    frame[4]=PC_CMD_READ_DUAL_LIMIT;crc=CRC16_Modbus(frame,7);
    frame[7]=(uint8_t)crc;frame[8]=(uint8_t)(crc>>8);
    feed(owner,frame,9);Network_Task();assert(diagnostics==1 && stopping);
    uint8_t move[15]={0xaa,0x55,12,1,PC_CMD_MOVE_OUTPUT_RELATIVE,0,0,0,10,0,0,3,232,0,0};
    crc=CRC16_Modbus(move,13);move[13]=(uint8_t)crc;move[14]=(uint8_t)(crc>>8);
    feed(owner,move,15);Network_Task();assert(commands==4 && stopping);
    m.continuous_stop_pending[0]=m.continuous_stop_pending[1]=0;Network_Task();assert(!stopping);
    assert(!m.trajectory.session && !m.trajectory.count);
    /* No stale bytes survive takeover. */
    Network_Task();assert(commands==4);
    physical_link=0;tick+=250;Network_Task();assert(stops==2 && !client && stopping);
    puts("TCP ownership, split/merged frames, disconnect stops, stale-command rejection and link-loss tests passed");
    return 0;
}
