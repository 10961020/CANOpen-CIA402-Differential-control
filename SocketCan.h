#ifndef __SOCKETCAN_H__
#define __SOCKETCAN_H__

#include <iostream>
#include <cstring>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <iomanip>
#include <thread>
#include "Kinematics.h"
#include "share.h"


#define CAN_INTERFACE "vcan0"   // 使用的 CAN 接口名称


int Can_Init();
uint32_t Can_FilterID(uint32_t can_id);
void Can_Print(can_frame frame_local, std::string prefix);
void Can_Send_w4(int socketDescriptor_local, int node, uint16_t index, uint8_t sub, int32_t value);
void Can_Send_r4(int socketDescriptor_local, int node, uint16_t index, uint8_t sub);
void Can_Send_RPOD1(int socketDescriptor_local, int node, int32_t speed);
void NMT_Start_Node(int socketDescriptor_local, uint8_t node);
void Driver_Init_Wait(int socketDescriptor_local);
int Shared_Memory_Init(SharedData* shared);
void Can_Recvtask(int socketDescriptor_local, SharedData* shared);

#endif
