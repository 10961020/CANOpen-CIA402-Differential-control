#include "Kinematics.h"

extern uint8_t Condition_Flag_1;
extern uint8_t Condition_Flag_2;
extern bool position_arrive_1;
extern bool position_arrive_2;
extern const uint8_t NODE_ID_1;
extern const uint8_t NODE_ID_2;

driver_param_t driver1 = {1, 0, 0.0, 0, 0.0};
driver_param_t driver2 = {2, 0, 0.0, 0, 0.0};
RobotParams AGV_p = {0.075, 0.34, 10000, 1, 0.0}; // 轮子半径75mm 轮距340mm 编码器分辨率10000 减速比1:1


/**
 * @brief 初始化运动学参数
 * 
 */
void Kinematic_Init(){
    int32_t counts_per_wheel_rev = AGV_p.counts_per_rev * AGV_p.gear_ratio;
    AGV_p.counts_per_meter = counts_per_wheel_rev / (2.0 * M_PI * AGV_p.wheel_radius);
}


/**
 * @brief 逆运动学计算，根据线速度和角速度计算左右轮速度
 * 
 * @param linear_speed 线速度，单位米每秒
 * @param angular_speed 角速度，单位弧度每秒
 */
void Kinematic_Inverse(double linear_speed, double angular_speed)
{
    // 计算左右轮速度，单位米每秒
    double wheel_1 = linear_speed - (angular_speed * AGV_p.wheel_base) / 2.0;
    double wheel_2 = linear_speed + (angular_speed * AGV_p.wheel_base) / 2.0;

    // 转换为驱动器单位，单位rpm
    wheel_1 = (wheel_1 / (2 * M_PI * AGV_p.wheel_radius)) * 60.0 * AGV_p.gear_ratio;
    wheel_2 = (wheel_2 / (2 * M_PI * AGV_p.wheel_radius)) * 60.0 * AGV_p.gear_ratio;

    // 限制最大最小rpm
    wheel_1 = target_limit_double(wheel_1, -MAX_RPM, MAX_RPM);
    wheel_2 = target_limit_double(wheel_2, -MAX_RPM, MAX_RPM);

    // 转换为驱动器单位，单位dec
    driver1.speed = static_cast<int32_t>((wheel_1 * 512.0 * AGV_p.counts_per_rev) / 1875.0);
    driver2.speed = static_cast<int32_t>((wheel_2 * 512.0 * AGV_p.counts_per_rev) / 1875.0);
}


/**
 * @brief 正运动学计算，根据左右轮速度计算线速度和角速度
 * 
 * @param linear_speed 输出线速度，单位米每秒
 * @param angular_speed 输出角速度，单位弧度每秒
 */
void Kinematic_Forward(double &linear_speed, double &angular_speed)
{
    double wheel_1 = (double)driver1.current_speed * 1875.0 / (512.0 * AGV_p.counts_per_rev * AGV_p.gear_ratio);
    double wheel_2 = (double)driver2.current_speed * 1875.0 / (512.0 * AGV_p.counts_per_rev * AGV_p.gear_ratio);

    wheel_1 = (wheel_1 * 2.0 * M_PI * AGV_p.wheel_radius) / 60.0;
    wheel_2 = (wheel_2 * 2.0 * M_PI * AGV_p.wheel_radius) / 60.0;

    linear_speed = (wheel_1 + wheel_2) / 2.0;   // 计算线速度
    angular_speed = (wheel_1 - wheel_2) / AGV_p.wheel_base;   // 计算角速度
}


/**
 * @brief 计算目标位置增量，单位脉冲
 * 
 * @param dx 前进位移，单位米
 * @param dy 侧向位移，单位米
 * @param dtheta 旋转位移，单位弧度
 * @return int32_t 左右轮目标位置增量，单位脉冲
 */
int32_t Kinematic_Compute_position(double dx, double dy, double dtheta) {
    // 机器人运动位移
    double ds = std::sqrt(dx*dx + dy*dy); // 前进位移

    // 左右轮脉冲
    double dsL = (ds - dtheta * (AGV_p.wheel_base / 2.0)) * AGV_p.counts_per_meter;
    double dsR = (ds + dtheta * (AGV_p.wheel_base / 2.0)) * AGV_p.counts_per_meter;

    return static_cast<int32_t>(dsL), static_cast<int32_t>(dsR);
}


/**
 * @brief 限制目标值在指定范围内
 * 
 * @param insert 要限制的值
 * @param low 最小值
 * @param high 最大值
 * @return double 限制后的值
 */
double target_limit_double(double insert, double low, double high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}


/**
 * @brief 从预操作态切换到运动就绪态
 * 
 * @param socketDescriptor_local 套接字描述符
 */
void Kinematic_Preoperational2Operational(int socketDescriptor_local, int node){
    Can_Send_w4(socketDescriptor_local, node, control_word_index, 0x00, 0x06);
    usleep(50*1000);
    Can_Send_w4(socketDescriptor_local, node, control_word_index, 0x00, 0x07);
    Can_Send_r4(socketDescriptor_local, node, status_word_index, 0x00); // 读取状态字
}


/**
 * @brief 设置驱动器为速度模式
 * 
 * @param socketDescriptor_local 套接字描述符
 * @param node 节点 ID
 */
void Kinematic_SpeedMode(int socketDescriptor_local, int node){
    Can_Send_w4(socketDescriptor_local, node, mode_of_operation_index, 0x00, 3);
}


/**
 * @brief 设置驱动器为位置模式
 * 
 * @param socketDescriptor_local 套接字描述符
 * @param node 节点 ID
 */
void Kinematic_PositionMode(int socketDescriptor_local, int node){
    Can_Send_w4(socketDescriptor_local, node, mode_of_operation_index, 0x00, 1);
}


/**
 * @brief 映射 RPDO1 到目标速度
 * 
 * @param socketDescriptor_local 套接字描述符
 * @param node 节点 ID
 */
void map_rpdo1_to_target_speed(int socketDescriptor_local, int node) {
    // 1. 禁用映射
    Can_Send_w4(socketDescriptor_local, node, map_rpdo1_index, 0x00, 0);

    // 2. 添加 0x60FF (Target Speed)
    uint32_t mapping = 0x60FF0020; // index=0x60FF, sub=0, len=32bit
    Can_Send_w4(socketDescriptor_local, node, map_rpdo1_index, 0x01, mapping);

    // 3. 启用映射 (数量=1)
    Can_Send_w4(socketDescriptor_local, node, map_rpdo1_index, 0x00, 1);
}


/**
 * @brief 从共享内存读取数据
 * 
 * @param shared 共享内存指针
 * @param linear_speed 输出线速度，单位米每秒
 * @param angular_speed 输出角速度，单位弧度每秒
 * @param dx 输出前进位移，单位米
 * @param dy 输出侧向位移，单位米
 * @param dtheta 输出旋转位移，单位弧度
 */
void SHM_read(SharedData* shared, double &linear_speed, double &angular_speed, double &dx, double &dy, double &dtheta){
    pthread_mutex_lock(&shared->mutex);
    linear_speed = shared->speed_x; // 线速度，单位米每秒
    angular_speed = shared->speed_z; // 角速度，单位弧度每秒
    dx = shared->dx;
    dy = shared->dy;
    dtheta = shared->dtheta_z;
    pthread_mutex_unlock(&shared->mutex);
    driver1.position, driver2.position = Kinematic_Compute_position(dx, dy, dtheta);
    Kinematic_Inverse(linear_speed, angular_speed);
}


/**
 * @brief 向共享内存写入当前速度
 * 
 * @param shared 共享内存指针
 * @param linear_speed 当前线速度，单位米每秒
 * @param angular_speed 当前角速度，单位弧度每秒
 */
void SHM_write(SharedData* shared, double &linear_speed, double &angular_speed){
    Kinematic_Forward(linear_speed, angular_speed);
    pthread_mutex_lock(&shared->mutex);
    shared->current_linear_speed = linear_speed;
    shared->current_angular_speed = angular_speed;
    pthread_mutex_unlock(&shared->mutex);
}


/**
 * @brief 控制任务，管理驱动器状态和控制
 * 
 * @param socketDescriptor_local 套接字描述符
 */
void Kinematic_CtrlTask(int socketDescriptor_local, SharedData* shared){
    bool mode_condition = true; // true 位置模式 false 速度模式
    double dx, dy, dtheta;
    double linear_speed = 0.0; // 线速度，单位米每秒
    double angular_speed = 0.0; // 角速度，单位弧度每秒
    Kinematic_Init();

    while (1){
        usleep(10*1000); // 10ms

        SHM_read(shared, linear_speed, angular_speed, dx, dy, dtheta);

        if (Condition_Flag_1 == 1 && Condition_Flag_2 == 1){ // 预操作态
            Kinematic_Preoperational2Operational(socketDescriptor_local, NODE_ID_1);
            Kinematic_Preoperational2Operational(socketDescriptor_local, NODE_ID_2);
            usleep(10*1000);
            if (mode_condition == true){   //直行采用定位模式
                Kinematic_PositionMode(socketDescriptor_local, NODE_ID_1);
                Kinematic_PositionMode(socketDescriptor_local, NODE_ID_2);
            }else{
                Kinematic_SpeedMode(socketDescriptor_local, NODE_ID_1);
                Kinematic_SpeedMode(socketDescriptor_local, NODE_ID_2);
            }
            usleep(10*1000);
        }else if (Condition_Flag_1 == 3 && Condition_Flag_2 == 3){ // 运动就绪态
            Can_Send_w4(socketDescriptor_local, NODE_ID_1, control_word_index, 0x00, 0x0F);
            Can_Send_w4(socketDescriptor_local, NODE_ID_2, control_word_index, 0x00, 0x0F);
            // usleep(10*1000);
        }else if (Condition_Flag_1 == 4 && Condition_Flag_2 == 4){ // 使能态
            if (mode_condition == true){
                if (position_arrive_1 && position_arrive_2){
                    position_arrive_1 = false;
                    position_arrive_2 = false;

                    Can_Send_w4(socketDescriptor_local, NODE_ID_1, target_position_index, 0x00, static_cast<int32_t>(driver1.position));
                    Can_Send_w4(socketDescriptor_local, NODE_ID_2, target_position_index, 0x00, static_cast<int32_t>(driver2.position));
                    Can_Send_w4(socketDescriptor_local, NODE_ID_1, control_word_index, 0x00, 0x4F);
                    Can_Send_w4(socketDescriptor_local, NODE_ID_2, control_word_index, 0x00, 0x4F);
                    Can_Send_w4(socketDescriptor_local, NODE_ID_1, control_word_index, 0x00, 0x5F);
                    Can_Send_w4(socketDescriptor_local, NODE_ID_2, control_word_index, 0x00, 0x5F);
                }
            }else{
                Can_Send_w4(socketDescriptor_local, NODE_ID_1, target_speed_index, 0x00, static_cast<int32_t>(driver1.speed)); // 发送目标速度，单位mm/s
                Can_Send_w4(socketDescriptor_local, NODE_ID_2, target_speed_index, 0x00, static_cast<int32_t>(driver2.speed)); // 发送目标速度，单位mm/s
            }
        }else if (Condition_Flag_1 == 5 || Condition_Flag_2 == 5){ // 关闭态
            Can_Send_w4(socketDescriptor_local, NODE_ID_1, control_word_index, 0x00, 0x00);
            Can_Send_w4(socketDescriptor_local, NODE_ID_2, control_word_index, 0x00, 0x00);
        }
    
        // switch (Condition_Flag){
        //     case 1:
        //         std::cout << "Kinematics: Driver is in preoperational..." << std::endl;
        //         Kinematic_Preoperational2Operational(socketDescriptor_local, NODE_ID);
        //         break;
        //     case 3:
        //         std::cout << "Kinematics: Driver is in running..." << std::endl;
        //         if (mode_condition == true){   //直行采用定位模式 旋转采用速度模式
        //             Kinematic_PositionMode(socketDescriptor_local, NODE_ID);
        //             Can_Send_w4(socketDescriptor_local, NODE_ID, target_position_index, 0x00, static_cast<int32_t>(driver1.position * 1000)); // 发送目标位置，单位mm
        //         }else{
        //             Kinematic_SpeedMode(socketDescriptor_local, NODE_ID);
        //             Can_Send_w4(socketDescriptor_local, NODE_ID, target_speed_index, 0x00, static_cast<int32_t>(driver1.speed * 1000)); // 发送目标速度，单位mm/s
        //         }
        //         Can_Send_w4(socketDescriptor_local, NODE_ID, control_word_index, 0x00, 0x0F);
        //         //Can_Send_r4(socketDescriptor_local, NODE_ID, status_word_index, 0x00); // 读取状态字
        //         break;
        //     case 4:
        //         std::cout << "Kinematics: Driver is in enable..." << std::endl;
        //         if (mode_condition){
        //             Can_Send_RPOD1(socketDescriptor_local, NODE_ID, static_cast<int32_t>(driver1.speed * 1000)); // 发送目标速度，单位mm/s
        //         }
        //         break;
        //     case 5:
        //         Can_Send_w4(socketDescriptor_local, NODE_ID, control_word_index, 0x00, 0x00);
        //         break;
        //     default:
        //         break;
        // }
        SHM_write(shared, linear_speed, angular_speed);

    }
    
}
