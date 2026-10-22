#include "network.h"
#include "network_config.h"
#include "ethernet_port.h"
#include "axis_guard_controller.h"
#include "lwip/init.h"
#include "lwip/tcp.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include <string.h>

static struct netif interface;
static struct tcp_pcb *client;
static pc_protocol_t *protocol;
static command_manager_t *manager;
static uint8_t rx[8192]; /* larger than the entire advertised TCP receive window */
static uint16_t head,tail;
static uint8_t lost,stopping,link_up;
static uint32_t link_at;
static int motors_stopping(void);
uint32_t sys_now(void) {return HAL_GetTick();}
static void command(void *context,const pc_command_packet_t *packet)
{
    /* A failed response may abort the socket while a batch is being parsed. */
    if (!client || lost) return;
    if (stopping || motors_stopping()) {
        if (packet->command==PC_CMD_MOTOR_STOP || packet->command==PC_CMD_EMERGENCY_STOP) {
            Command_Process(context,packet);return;
        }
        /* A missing drive must not lock operators out of the network. Keep
           motion/configuration blocked while permitting RAM-only diagnostics. */
        if ((packet->device_id==1 || packet->device_id==2) && packet->valid_payload &&
            (packet->command==PC_CMD_READ_GUARD || packet->command==PC_CMD_READ_DUAL_LIMIT)) {
            GuardController_Command(manager,packet);return;
        }
        uint8_t frame[PC_PROTOCOL_MAX_FRAME];
        uint16_t size=PC_Protocol_BuildResponse(frame,sizeof(frame),packet->device_id,packet->command,
            packet->valid_payload ? PC_STATUS_BUSY : PC_STATUS_ILLEGAL_PARAM,packet->parameter,0);
        if (size) (void)Network_Send(frame,size);
        return;
    }
    Command_Process(context,packet);
}

static void discard_session(void)
{ head=tail=0;lost=1; }
static void disconnected(void *arg,err_t error)
{ (void)arg;(void)error;client=NULL;discard_session(); }
static void abort_client(void)
{
    struct tcp_pcb *old=client;client=NULL;
    if (old) {tcp_arg(old,NULL);tcp_err(old,NULL);tcp_recv(old,NULL);tcp_abort(old);}
    discard_session();
}
static err_t receive(void *arg,struct tcp_pcb *pcb,struct pbuf *p,err_t error)
{
    (void)arg;
    if (!p || error!=ERR_OK) {
        if (p) pbuf_free(p);
        client=NULL;discard_session();tcp_err(pcb,NULL);tcp_recv(pcb,NULL);
        if (tcp_close(pcb)!=ERR_OK) {tcp_abort(pcb);return ERR_ABRT;}
        return ERR_OK;
    }
    uint16_t used=(uint16_t)((head+sizeof(rx)-tail)%sizeof(rx));
    if (p->tot_len>=sizeof(rx)-used) return ERR_MEM; /* leave entire pbuf unconsumed */
    for (struct pbuf *q=p;q;q=q->next)
        for (uint16_t n=0;n<q->len;++n) {rx[head]=((uint8_t *)q->payload)[n];head=(uint16_t)((head+1U)%sizeof(rx));}
    tcp_recved(pcb,p->tot_len);pbuf_free(p);return ERR_OK;
}
static int motors_stopping(void)
{
    return manager->continuous_stop_pending[0] || manager->continuous_stop_pending[1] ||
        manager->guard[0].stop_pending || manager->guard[1].stop_pending || Trajectory_Active(&manager->trajectory);
}
static err_t accepted(void *arg,struct tcp_pcb *pcb,err_t error)
{
    (void)arg;
    if (!pcb) return ERR_VAL;
    if (error!=ERR_OK || client || lost) {tcp_abort(pcb);return ERR_ABRT;}
    client=pcb;head=tail=0;PC_Protocol_Init(protocol,command,manager);
    tcp_recv(pcb,receive);tcp_err(pcb,disconnected);tcp_nagle_disable(pcb);
    ip_set_option(pcb,SOF_KEEPALIVE);
    pcb->keep_idle=3000;pcb->keep_intvl=1000;pcb->keep_cnt=3;
    return ERR_OK;
}
HAL_StatusTypeDef Network_Init(pc_protocol_t *p,command_manager_t *m)
{
    protocol=p;manager=m;
    srand(HAL_GetUIDw0()^HAL_GetUIDw1()^HAL_GetUIDw2());
    lwip_init();
    ip4_addr_t ip,mask,gateway;
    IP4_ADDR(&ip,NETWORK_IP_A,NETWORK_IP_B,NETWORK_IP_C,NETWORK_IP_D);
    IP4_ADDR(&mask,255,255,255,0);IP4_ADDR(&gateway,NETWORK_IP_A,NETWORK_IP_B,NETWORK_IP_C,1);
    if (!netif_add(&interface,&ip,&mask,&gateway,NULL,Ethernet_Init,ethernet_input)) return HAL_ERROR;
    netif_set_default(&interface);netif_set_up(&interface);
    struct tcp_pcb *pcb=tcp_new();if (!pcb) return HAL_ERROR;
    if (tcp_bind(pcb,IP_ADDR_ANY,NETWORK_TCP_PORT)!=ERR_OK) {tcp_abort(pcb);return HAL_ERROR;}
    struct tcp_pcb *listener=tcp_listen_with_backlog(pcb,1);
    if (!listener) {tcp_abort(pcb);return HAL_ERROR;}
    tcp_accept(listener,accepted);return HAL_OK;
}
void Network_Task(void)
{
    if ((int32_t)(HAL_GetTick()-link_at)>=0) {
        link_at=HAL_GetTick()+250U;
        uint8_t now=(uint8_t)Ethernet_Link(&interface);
        if (link_up && !now && client) abort_client();
        link_up=now;
    }
    Ethernet_Poll(&interface);sys_check_timeouts();
    if (lost) {
        lost=0;stopping=1;head=tail=0;
        PC_Protocol_Init(protocol,command,manager);
        Command_Manager_TransportLost(manager);
    }
    if (stopping) {
        if (!motors_stopping()) {memset(&manager->trajectory,0,sizeof(manager->trajectory));stopping=0;}
    }
    if (!client) return;
    uint8_t local[64];uint16_t n=0;
    while (tail!=head && n<sizeof(local)) {local[n++]=rx[tail];tail=(uint16_t)((tail+1U)%sizeof(rx));}
    if (n) PC_Protocol_Input(protocol,local,n);
    if (lost) {
        lost=0;stopping=1;head=tail=0;
        PC_Protocol_Init(protocol,command,manager);
        Command_Manager_TransportLost(manager);
    }
}
HAL_StatusTypeDef Network_Send(const uint8_t *data,uint16_t length)
{
    if (!client || lost) return HAL_ERROR;
    if (tcp_write(client,data,length,TCP_WRITE_FLAG_COPY)!=ERR_OK || tcp_output(client)!=ERR_OK) {
        abort_client();return HAL_ERROR;
    }
    return HAL_OK;
}
