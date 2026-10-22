#include "pc_protocol.h"
#include "crc16.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned calls;
static pc_command_packet_t last;
static void receive(void *context,const pc_command_packet_t *c) { (void)context;++calls;last=*c; }
static void input(pc_protocol_t *p,uint8_t command,uint8_t size)
{
    uint8_t f[64]={0xAA,0x55,0,1,0,0,17};f[2]=(uint8_t)(6+size);f[4]=command;
    uint16_t crc=CRC16_Modbus(f,7U+size);f[7+size]=(uint8_t)crc;f[8+size]=(uint8_t)(crc>>8);
    PC_Protocol_Input(p,f,4);PC_Protocol_Input(p,f+4,(uint16_t)(5+size));
}
int main(void)
{
    pc_protocol_t p;PC_Protocol_Init(&p,receive,NULL);
    input(&p,PC_CMD_TRAJECTORY_BEGIN,20);assert(calls==1 && last.valid_payload && last.parameter==17);
    input(&p,PC_CMD_TRAJECTORY_BEGIN,19);assert(calls==2 && !last.valid_payload);
    input(&p,PC_CMD_TRAJECTORY_POINT,14);assert(last.valid_payload);
    input(&p,PC_CMD_TRAJECTORY_POINT,15);assert(!last.valid_payload);
    for (uint8_t op=PC_CMD_TRAJECTORY_SEAL;op<=PC_CMD_TRAJECTORY_KEEPALIVE;++op) {
        input(&p,op,0);assert(last.valid_payload);input(&p,op,1);assert(!last.valid_payload);
    }
    input(&p,PC_CMD_READ_AXES,0);assert(!last.valid_payload); /* parameter must be zero */
    input(&p,PC_CMD_TRAJECTORY_CLEAR,0);assert(last.valid_payload);
    input(&p,PC_CMD_TRAJECTORY_CLEAR,1);assert(!last.valid_payload);
    input(&p,PC_CMD_SET_REFERENCE,4);assert(last.valid_payload && last.data_length==4);
    input(&p,PC_CMD_SET_REFERENCE,0);assert(!last.valid_payload);
    input(&p,PC_CMD_SET_REFERENCE,5);assert(!last.valid_payload);
    uint8_t axes[9]={0xAA,0x55,6,1,PC_CMD_READ_AXES,0,0,0,0};
    uint16_t axes_crc=CRC16_Modbus(axes,7);axes[7]=(uint8_t)axes_crc;axes[8]=(uint8_t)(axes_crc>>8);
    PC_Protocol_Input(&p,axes,sizeof(axes));assert(last.valid_payload && last.command==PC_CMD_READ_AXES);
    uint8_t data[40]={0},frame[64];
    uint16_t length=PC_Protocol_BuildResponseData(frame,sizeof(frame),1,PC_CMD_TRAJECTORY_STATUS,PC_STATUS_SUCCESS,17,data,40);
    assert(length==50 && frame[2]==47);
    assert(CRC16_Modbus(frame,length-2)==((uint16_t)frame[length-2]|((uint16_t)frame[length-1]<<8)));
    puts("trajectory protocol lengths, split frames and CRC tests passed");return 0;
}
