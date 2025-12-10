#include "SocketCan.h"


uint8_t Condition_Flag_1 = 0; // 0 初始化 1禁止运行 2 运行就绪 3 运行 4 使能
uint8_t Condition_Flag_2 = 0;
uint8_t Can_Network_Flag_1 = 0x00; // 0 启动0x00 1 停止0x04 2 运行0x05 3 预操作0x7F
uint8_t Can_Network_Flag_2 = 0x00;
bool position_arrive_1 = true;
bool position_arrive_2 = true;
const uint8_t NODE_ID_1 = 1; // 节点 ID
const uint8_t NODE_ID_2 = 2;
extern driver_param_t driver1, driver2;


/**
 * @brief 初始化 SocketCAN，返回套接字描述符
 * 
 * @return int 套接字描述符，失败返回 0
 */
int Can_Init() {
    // 创建 SocketCAN 套接字
    int socketDescriptor_local = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketDescriptor_local == -1) {
        std::cerr << "CAN_Init: Failed to create socket" << std::endl;
        exit(1);
    }

    // 绑定到 vcan0 接口
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ-1] = 0;
    if (ioctl(socketDescriptor_local, SIOCGIFINDEX, &ifr) == -1) {
        std::cerr << "CAN_Init: Failed to get interface index for " << CAN_INTERFACE << std::endl;
        exit(1);
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socketDescriptor_local, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        std::cerr << "CAN_Init: Failed to bind socket to " << CAN_INTERFACE << std::endl;
        exit(1);
    }

    // 设置过滤器
    // struct can_filter rfilter[1];
    // rfilter[0].can_id   = 0x600;
    // rfilter[0].can_mask = 0x700;
    // setsockopt(socketDescriptor_local, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
               
    return socketDescriptor_local;
}


/**
 * @brief 过滤 CAN ID，返回标准帧 ID
 * 
 * @param can_id 原始 CAN ID
 * @return uint32_t 标准帧 ID
 */
uint32_t Can_FilterID(uint32_t can_id) {
    uint32_t sff_id;
    if (can_id & CAN_EFF_FLAG) {
        sff_id = can_id & CAN_EFF_MASK; // 29-bit
    } else 
        sff_id = can_id & CAN_SFF_MASK; // 11-bit
    return sff_id;
}


/**
 * @brief 打印 CAN 帧信息
 * 
 * @param frame_local 要打印的 CAN 帧
 * @param prefix 可选的前缀字符串
 */
void Can_Print(can_frame frame_local, std::string prefix = "") {
    std::cout << "-------" << prefix << CAN_INTERFACE << " ";
    std::cout << std::hex << Can_FilterID(frame_local.can_id) << " ";
    for (uint8_t i = 0; i < frame_local.can_dlc; ++i) {
        std::cout << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(frame_local.data[i]) << " ";
    }
    std::cout << "-------" << std::endl;
}


/**
 * @brief 发送 CAN 帧
 * 
 * @param socketDescriptor_local 套接字描述符
 * @param node 节点 ID
 * @param index 索引
 * @param sub 子索引
 * @param value 要发送的 32 位值
 */
void Can_Send_w4(int socketDescriptor_local, int node, uint16_t index, uint8_t sub, int32_t value) {
    struct can_frame frame = {0};
    frame.can_id = 0x600 + node;
    frame.can_dlc = 8;
    frame.data[0] = 0x23; // command: download 4 bytes
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = sub;
    frame.data[4] = value & 0xFF;
    frame.data[5] = (value >> 8) & 0xFF;
    frame.data[6] = (value >> 16) & 0xFF;
    frame.data[7] = (value >> 24) & 0xFF;
    write(socketDescriptor_local, &frame, sizeof(frame));
    Can_Print(frame, "Send write data: ");
    // usleep(20*1000);
}


/**
 * @brief 发送读取请求 CAN 帧
 * 
 * @param socketDescriptor_local 套接字描述符
 * @param node 节点 ID
 * @param index 索引
 * @param sub 子索引
 */
void Can_Send_r4(int socketDescriptor_local, int node, uint16_t index, uint8_t sub) {
    struct can_frame frame = {0};
    frame.can_id = 0x600 + node;
    frame.can_dlc = 8;
    frame.data[0] = 0x40; // command: read 4 bytes
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = sub;
    write(socketDescriptor_local, &frame, sizeof(frame));
    Can_Print(frame, "Send read data: ");
    // usleep(20*1000);
}


/**
 * @brief 发送 RPDO1 消息，设置目标速度
 * 
 * @param socketDescriptor_local 套接字描述符
 * @param node 节点 ID
 * @param target_speed 目标速度，单位 mm/s
 */
void Can_Send_RPOD1(int socketDescriptor_local, int node, int32_t target_speed) {
    struct can_frame frame = {0};
    frame.can_id = 0x200 + node; // RPDO1 的 CAN ID
    frame.can_dlc = 4;
    memcpy(&frame.data[0], &target_speed, 4);
    write(socketDescriptor_local, &frame, sizeof(frame));
    Can_Print(frame, "Send RPDO1: ");
    usleep(20*1000);
}


/**
 * @brief 发送 NMT Start Node  驱动器开启远程节点
 * 
 * @param socketDescriptor_local 套接字描述符
 * @param node 节点 ID，0 表示所有节点
 */
void NMT_Start_Node(int socketDescriptor_local, uint8_t node) {
    struct can_frame frame = {0};
    frame.can_id = 0x000; // NMT 消息的 CAN ID 是 0
    frame.can_dlc = 2;
    uint8_t data[2] = {0x01, static_cast<uint8_t>(node)}; // 01=start, 00=all nodes
    write(socketDescriptor_local, &frame, sizeof(frame));
    if (node == NODE_ID_1)
        Can_Network_Flag_1 = 0x05; // 进入运行状态
    else
        Can_Network_Flag_2 = 0x05;
}


void Driver_Init_Wait(int socketDescriptor_local){
    struct can_frame recv_frame;
    ssize_t bytesRead;

    while(1){
        bytesRead = read(socketDescriptor_local, &recv_frame, sizeof(recv_frame));
        if (bytesRead == -1){
            std::cerr << "Read error" << std::endl;
            usleep(10*1000); // 1000ms
            continue;
        }
        if ((Can_FilterID(recv_frame.can_id) == 0x700 + NODE_ID_1 && recv_frame.can_dlc == 1 && recv_frame.data[0] == 0x00) && Can_Network_Flag_1 == 0x00) {
            Can_Network_Flag_1 = 0x7F; // 进入预操作
            Condition_Flag_1 = 1;
            map_rpdo1_to_target_speed(socketDescriptor_local, NODE_ID_1);
            NMT_Start_Node(socketDescriptor_local, NODE_ID_1);  //多个伺服 第二个参数为0
        }else if ((Can_FilterID(recv_frame.can_id) == 0x700 + NODE_ID_2 && recv_frame.can_dlc == 1 && recv_frame.data[0] == 0x00) && Can_Network_Flag_2 == 0x00) {
            Can_Network_Flag_2 = 0x7F;
            Condition_Flag_2 = 1;
            map_rpdo1_to_target_speed(socketDescriptor_local, NODE_ID_2);
            NMT_Start_Node(socketDescriptor_local, NODE_ID_2);  //多个伺服 第二个参数为0
        }
        if (Can_Network_Flag_1 == 0x05 && Can_Network_Flag_2 == 0x05) break;
    }
    std::cout << "Driver initialized and in pre-operational state." << std::endl;
}


int Shared_Memory_Init(SharedData* shared) {
    int SHM_fd;
    while (1)
    {
        SHM_fd = shm_open("/myshm", O_RDWR, 0666);
        if (SHM_fd >= 0) {
            std::cout << "Shared memory opened successfully." << std::endl;
            break;
        }
    }
    shared = (SharedData*)mmap(NULL, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, SHM_fd, 0);

    // 初始化共享数据
    shared->motor_condition = false;
    shared->current_linear_speed = 0.0;
    shared->current_angular_speed = 0.0;

    return SHM_fd;
}


/**
 * @brief 线程函数，持续接收 CAN 消息
 * 
 * @param socketDescriptor_local 套接字描述符
 */
void Can_Recvtask(int socketDescriptor_local, SharedData* shared){
    using namespace std::chrono;

    struct can_frame recv_frame;
    ssize_t bytesRead;
    uint8_t condition_driver = 0;
    std::chrono::time_point<std::chrono::steady_clock> now;
    long long diff_1;
    long long diff_2;
    auto last_heartbeat_1 = steady_clock::now();
    auto last_heartbeat_2 = steady_clock::now();

    while(1){
        bytesRead = read(socketDescriptor_local, &recv_frame, sizeof(recv_frame));
        if (bytesRead == -1){
            std::cerr << "Read error" << std::endl;
            usleep(10*1000); // 10ms
            continue;
        }

        now = steady_clock::now();
        diff_1 = duration_cast<seconds>(now - last_heartbeat_1).count();
        diff_2 = duration_cast<seconds>(now - last_heartbeat_2).count();
        if (diff_1 >= 2 || diff_2 >=2){ // 停止或运行状态{
            std::cerr << "Can_Recvtask: Heartbeat timeout." << std::endl;
            continue;
        }
        if ((Can_Network_Flag_1 == 0x04 || Can_Network_Flag_1 == 0x7F)||(Can_Network_Flag_2 == 0x04 || Can_Network_Flag_2 == 0x7F)){
            std::cerr << "Can_Recvtask: Driver not in operational state." << std::endl;
            continue;
        }

        switch (Can_FilterID(recv_frame.can_id)){
            case 0x580 + NODE_ID_1: // SDO 消息反馈
                if (recv_frame.data[0] == 0x80){
                    Can_Print(recv_frame, "error code: ");
                }else{
                    // 反馈消息暂不处理
                }
                break;
            case 0x580 + NODE_ID_2: // SDO 消息反馈
                if (recv_frame.data[0] == 0x80){
                    Can_Print(recv_frame, "error code: ");
                }else{
                    // 反馈消息暂不处理
                }
                break;
            
            case 0x180 + NODE_ID_1: // TPDO1 速度位置状态反馈
                driver1.current_speed = recv_frame.data[0] | (recv_frame.data[1] << 8) | (recv_frame.data[2] << 16) | (recv_frame.data[3] << 24);
                driver1.current_position = recv_frame.data[4] | (recv_frame.data[5] << 8) | (recv_frame.data[6] << 16) | (recv_frame.data[7] << 24);
                break;
            case 0x180 + NODE_ID_2: // TPDO1 速度位置状态反馈
                driver2.current_speed = recv_frame.data[0] | (recv_frame.data[1] << 8) | (recv_frame.data[2] << 16) | (recv_frame.data[3] << 24);
                driver2.current_position = recv_frame.data[4] | (recv_frame.data[5] << 8) | (recv_frame.data[6] << 16) | (recv_frame.data[7] << 24);
                break;
            
            case 0x280 + NODE_ID_1: // TPDO2 驱动器状态字
                condition_driver = recv_frame.data[0];
                if (condition_driver == 0x31) Condition_Flag_1 = 2;
                else if (condition_driver == 0x33) Condition_Flag_1 = 3;
                else if (condition_driver == 0x37) Condition_Flag_1 = 4;
                else if (condition_driver == 0x70) Condition_Flag_1 = 0;
                else{ Condition_Flag_1 = 5; std::cout << "Unknown driver condition: " << std::hex << static_cast<int>(condition_driver) << std::endl;}

                if (recv_frame.data[1] & (1 << 2)){
                    position_arrive_1 = true;
                }
                break;
            case 0x280 + NODE_ID_2: // TPDO2 驱动器状态字
                condition_driver = recv_frame.data[0];
                if (condition_driver == 0x31) Condition_Flag_2 = 2;
                else if (condition_driver == 0x33) Condition_Flag_2 = 3;
                else if (condition_driver == 0x37) Condition_Flag_2 = 4;
                else if (condition_driver == 0x70) Condition_Flag_2 = 1;
                else{ Condition_Flag_2 = 5; std::cout << "Unknown driver condition: " << std::hex << static_cast<int>(condition_driver) << std::endl;}

                if (recv_frame.data[1] & (1 << 2)){
                    position_arrive_2 = true;
                }
                break;
            
            case 0x700 + NODE_ID_1: // heartbeat
                if (recv_frame.can_dlc == 1){
                    last_heartbeat_1 = steady_clock::now();
                    Can_Network_Flag_1 = recv_frame.data[0];
                }
                break;
            case 0x700 + NODE_ID_2: // heartbeat
                if (recv_frame.can_dlc == 1){
                    last_heartbeat_2 = steady_clock::now();
                    Can_Network_Flag_2 = recv_frame.data[0];
                }
                break;
            default:
                break;
        }

    }
}