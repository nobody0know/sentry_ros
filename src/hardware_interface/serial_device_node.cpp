// #include"serial_device_node.h"
// sentry_info info_data;
// sentry_control ctr_data;
// serial::serial_device seri_dev("/dev/wheeltec_ch340",115200);
// void cmdCallback(const geometry_msgs::Twist &cmd_data)
// {
//     ctr_data.chassis_vx.float_d = cmd_data.linear.x;
//     ctr_data.chassis_vy.float_d = cmd_data.linear.y;
//     ctr_data.chassis_vw.float_d = cmd_data.angular.z;
//     seri_dev.transformData(ctr_data);
// }

// void gimbalYawCallback(const std_msgs::Float32 &cmd_data)
// {
//     ctr_data.yaw_angle.float_d = cmd_data.data;
//     seri_dev.transformData(ctr_data);
// }

// void gimbalPitchCallback(const std_msgs::Float32 &cmd_data)
// {
//     ctr_data.pitch_angle.float_d = cmd_data.data;
//     seri_dev.transformData(ctr_data);
// }

// void fireCallback(const std_msgs::Bool &cmd_data)
// {
//     ctr_data.fire_control = cmd_data.data;
//     seri_dev.transformData(ctr_data);
// }

// int main(int argc,char **argv)
// {
//     ros::init(argc,argv,"hardware_publisher_node");
//     ros::NodeHandle n;
//     // seri_dev.init_serial_port();

//     // ros::Publisher chassis_vx_pub = n.advertise<std_msgs::Float32>("chassis_vx",100);
//     // ros::Publisher chassis_vy_pub = n.advertise<std_msgs::Float32>("chassis_vy",100);
//     // ros::Publisher chassis_vw_pub = n.advertise<std_msgs::Float32>("chassis_vw",100);
//     // ros::Publisher gimbal_yaw_pub = n.advertise<std_msgs::Float32>("gimbal_yaw_angle",100);
//     // ros::Publisher gimbal_pitch_pub = n.advertise<std_msgs::Float32>("gimbal_pitch_angle",100);
//     // ros::Publisher gimbal_roll_pub = n.advertise<std_msgs::Float32>("gimbal_roll_angle",100);
//     // ros::Subscriber gimbal_yaw_sub = n.subscribe("gimbal_yaw_ctr",100,gimbalYawCallback);
//     // ros::Subscriber gimbal_pitch_sub = n.subscribe("gimbal_pitch_ctr",100,gimbalPitchCallback);
//     // ros::Subscriber fire_sub = n.subscribe("fire_ctr",100,fireCallback);

//     ros::Rate loop_rate(100);
//     while (ros::ok())
//     {
//         // std_msgs::Float32 vx;
//         // std_msgs::Float32 vy;
//         // std_msgs::Float32 vw;
//         // std_msgs::Float32 yaw_angle;
//         // std_msgs::Float32 pitch_angle;
//         // std_msgs::Float32 roll_angle;

//         // seri_dev.receiveData(info_data);

//         // vx.data = (float)info_data.chassis_vx.int16_d/1000 ;
//         // vy.data = (float)info_data.chassis_vy.int16_d/1000 ;
//         // vw.data = (float)info_data.chassis_vw.int16_d/1000 ;
//         // yaw_angle.data = (float)info_data.yaw_angle.int16_d/100;
//         // //ROS_INFO("get yaw data:%f\n",yaw_angle.data);
//         // pitch_angle.data = (float)info_data.pitch_angle.int16_d/100;
//         // roll_angle.data = (float)info_data.roll_angle.int16_d/100;

//         // chassis_vx_pub.publish(vx);
//         // chassis_vy_pub.publish(vy);
//         // chassis_vw_pub.publish(vw);
//         // if (yaw_angle.data<180&&yaw_angle.data>-180)
//         // {
//         //     if(yaw_angle.data>0.1||yaw_angle.data<-0.1){
//         //         gimbal_yaw_pub.publish(yaw_angle);
//         //     }
//         // }
//         // if (pitch_angle.data<180&&pitch_angle.data>-180&&(pitch_angle.data>0.1||pitch_angle.data<-0.1))
//         // {
//         //     gimbal_pitch_pub.publish(pitch_angle);
//         // }
//         // if (roll_angle.data<180&&roll_angle.data>-180&&(roll_angle.data>0.1||roll_angle.data<-0.1))
//         // {
//         //     gimbal_roll_pub.publish(roll_angle);
//         // }
//         loop_rate.sleep();
//         ros::spinOnce();
//     }
//     return 0;
// }
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>
#include<unistd.h>
using namespace std;
int main(int argc,char** argv)
{
    ros::init(argc, argv, "cmdveltest");
     ros::NodeHandle cmdh;
    ros::Publisher cmdpub= cmdh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    ros::Rate r(60);
    while(1){
        geometry_msgs::Twist twist;
        geometry_msgs::Vector3 linear;
        linear.x=50;
        linear.y=30;
        linear.z=0;
        geometry_msgs::Vector3 angular;
        angular.x=0;
        angular.y=0;
        //直行
        //angular.z=0;
        //转圈
        angular.z=-0.5;
        twist.linear=linear;
        twist.angular=angular;

        cmdpub.publish(twist);
        cout<<"hello"<<endl;
       // ros::spinOnce();
        //r.sleep();
        sleep(1);
    }
    return 0;
}