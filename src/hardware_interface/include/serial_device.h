//
// Created by lxj on 22-11-6.
//

#ifndef SENTRY_ROS_SERIAL_DEVICE_H
#define SENTRY_ROS_SERIAL_DEVICE_H

#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <iostream>
#include <stdio.h>
#include <cstring>
#include "CRC_Check.h"
#include "filter.h"
#include <ros/ros.h>
using namespace std;

typedef union
{
    float float_d;
    unsigned char char_d[4];
}float_char;

typedef union
{
    int32_t int32_d;
    unsigned char char_d[4];
}int32_char;

typedef union
{
    int16_t int16_d;
    unsigned char char_d[2];
}int16_char;

typedef struct
{
   float_char yaw_angle;
   float_char pitch_angle;
   float_char chassis_vx;
   float_char chassis_vy;
   float_char chassis_vw;
}sentry_control;

typedef struct
{
    float_char sentry_id;
    float_char shoot_speed;
    float_char yaw_angle;
    float_char pitch_angle;
    float_char roll_angle;
    int16_char chassis_vx;
    int16_char chassis_vy;
    int16_char chassis_vw;
}sentry_info;

namespace serial {

    class serial_device {
    private:
        int serial_fd_;//串口号
        int baud_rate_;//波特率
        int data_bits_,stop_bits_,parity_bits_;//数据位,停止位,校验位
        string port_name_;
        fd_set serial_fd_set_;
        struct termios new_termios_, old_termios_;
        first_order_filter_type_t yaw_row_data_filter;
        unsigned char read_row_data[255];
        unsigned char transform_data[30];//要发送出去的数据
        bool config_device();
        bool open_device();
        bool close_device();
    public:
        serial_device(string port_name, int baud_rate);
        void reload_dev(string port_name);
        bool init_serial_port();
        int Read(uint8_t *buf,int len);
        int send(const uint8_t *buf,int len);
        void receiveData(sentry_info &data);
        void transformData(const sentry_control &data);
    };

} // serial

#endif //SENTRY_ROS_SERIAL_DEVICE_H
