#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__
#include "SocketCan.h"
#include "share.h"
#include <cmath>

#define status_word_index 0x6041        // 状态字
#define control_word_index 0x6040       // 控制字
#define mode_of_operation_index 0x6060  // 工作模式 (速度模式,位置模式) 
#define target_speed_index 0x60FF       // 目标速度
#define target_position_index 0x607A    // 目标位置
#define map_rpdo1_index 0x1600          // 映射 RPDO1
#define MAX_RPM 3000.0                  // 最大转速

typedef struct
{
    uint8_t id;                // 驱动器 ID
    int32_t position;          // 目标位置，单位rpm
    int32_t speed;             // 目标速度，单位dec
    int32_t current_position;  // 当前位置，单位rpm
    int32_t current_speed;     // 当前速度，单位dec
} driver_param_t;

struct RobotParams {
    double wheel_radius;      // 轮子半径 (m)
    double wheel_base;        // 轮距 (m)
    uint16_t counts_per_rev;        // 编码器分辨率
    uint8_t gear_ratio;           // 减速比
    double counts_per_meter;   // 每米脉冲数
};

void Kinematic_Init();
void Kinematic_Inverse(double linear_speed, double angular_speed);
void Kinematic_Forward(double &linear_speed, double &angular_speed);
void Kinematic_CtrlTask(int socketDescriptor_local, SharedData* shared);
void Kinematic_Preoperational2Operational(int socketDescriptor_local, int node);
void Kinematic_SpeedMode(int socketDescriptor_local, int node);
void Kinematic_PositionMode(int socketDescriptor_local, int node);
void map_rpdo1_to_target_speed(int socketDescriptor_local, int node);
int32_t Kinematic_Compute_position(double dx, double dy, double dtheta);
void SHM_read(SharedData* shared, double &linear_speed, double &angular_speed, double &dx, double &dy, double &dtheta);
void SHM_write(SharedData* shared, double &linear_speed, double &angular_speed);
double target_limit_double(double insert, double low, double high);

#endif