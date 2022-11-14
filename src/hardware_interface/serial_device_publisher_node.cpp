#include"serial_device_publisher_node.h"
sentry_info info_data;
int main(int argc,char **argv)
{
    ros::init(argc,argv,"hardware_publisher_node");
    ros::NodeHandle n;
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
        seri_dev.receiveData(info_data);
        vx.data = info_data.chassis_vx.float_d;
        vy.data = info_data.chassis_vy.float_d;
        vw.data = info_data.chassis_vw.float_d;
        yaw_angle.data = info_data.yaw_angle.float_d;
        pitch_angle.data = info_data.pitch_angle.float_d;
        roll_angle.data = info_data.roll_angle.float_d;
        chassis_vx_pub.publish(vx);
        chassis_vy_pub.publish(vy);
        chassis_vw_pub.publish(vw);
        gimbal_yaw_pub.publish(yaw_angle);
        gimbal_pitch_pub.publish(pitch_angle);
        gimbal_roll_pub.publish(roll_angle);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}