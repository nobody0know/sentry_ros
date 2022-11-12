#include"serial_device.h"
#include "std_msgs/Float32.h"
int main(int argc,char **argv)
{
    ros::init(argc,argv,"hardware_publisher_node");
    ros::NodeHandle n;
    sentry_info data;
    serial::serial_device seri_dev("/dev/ttyUSB0",115200);
    ros::Publisher chassis_vx_pub = n.advertise<std_msgs::Float32>("chassis_vx",1000);
    ros::Publisher chassis_vy_pub = n.advertise<std_msgs::Float32>("chassis_vy",1000);
    ros::Publisher chassis_vw_pub = n.advertise<std_msgs::Float32>("chassis_vw",1000);
    ros::Publisher gimbal_yaw_pub = n.advertise<std_msgs::Float32>("gimbal_yaw_angle",1000);
    ros::Publisher gimbal_pitch_pub = n.advertise<std_msgs::Float32>("gimbal_pitch_angle",1000);
    ros::Publisher gimbal_roll_pub = n.advertise<std_msgs::Float32>("gimbal_roll_angle",1000);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std_msgs::Float32 vx;
        std_msgs::Float32 vy;
        std_msgs::Float32 vw;
        std_msgs::Float32 yaw_angle;
        std_msgs::Float32 pitch_angle;
        std_msgs::Float32 roll_angle;
        seri_dev.receiveData(data);
        vx.data = data.chassis_vx.float_d;
        vy.data = data.chassis_vy.float_d;
        vw.data = data.chassis_vw.float_d;
        yaw_angle.data = data.yaw_angle.float_d;
        pitch_angle.data = data.pitch_angle.float_d;
        roll_angle.data = data.row_angle.float_d;
        
    }
    

}