#include "SocketCan.h"
#include "Kinematics.h"

int main() {
    // 创建 SocketCAN 套接字
    int socketDescriptor = Can_Init();
    if (socketDescriptor == 0) {
        return -1;
    }

    Driver_Init_Wait(socketDescriptor);
    SharedData* shared = nullptr;
    int SHM_fd = Shared_Memory_Init(shared);
    sleep(1);

    std::thread t1(Can_Recvtask, socketDescriptor, shared);
    std::thread t2(Kinematic_CtrlTask, socketDescriptor, shared);
    t1.join();
    t2.join();


    // 关闭套接字
    close(socketDescriptor);
    munmap(shared, sizeof(SharedData));
    close(SHM_fd);
    return 0;
}