#ifndef __SHARE_H__
#define __SHARE_H__

#include <pthread.h>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

struct SharedData {
    pthread_mutex_t mutex;
    double dx;
    double dy;
    double speed_x;
    double speed_y;
    double speed_z;
    double dtheta_x;
    double dtheta_y;
    double dtheta_z;

    bool motor_condition;
    bool node_arrive;
    double current_linear_speed;
    double current_angular_speed;
};


#endif