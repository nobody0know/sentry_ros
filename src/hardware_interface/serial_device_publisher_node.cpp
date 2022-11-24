#include"serial_device_publisher_node.h"
sentry_info info_data;
int main(int argc,char **argv)
{
    ros::init(argc,argv,"hardware_publisher_node");
    ros::NodeHandle n;
    first_order_filter_type_t yaw_data_filter;
    serial::serial_device seri_dev("/dev/ttyUSB0",115200);

    ros::Publisher chassis_vx_pub = n.advertise<std_msgs::Int16>("chassis_vx",10);
    ros::Publisher chassis_vy_pub = n.advertise<std_msgs::Int16>("chassis_vy",10);
    ros::Publisher chassis_vw_pub = n.advertise<std_msgs::Int16>("chassis_vw",10);
    ros::Publisher gimbal_yaw_pub = n.advertise<std_msgs::Float32>("gimbal_yaw_angle",10);
    ros::Publisher gimbal_pitch_pub = n.advertise<std_msgs::Float32>("gimbal_pitch_angle",10);
    ros::Publisher gimbal_roll_pub = n.advertise<std_msgs::Float32>("gimbal_roll_angle",10);
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        std_msgs::Int16 vx;
        std_msgs::Int16 vy;
        std_msgs::Int16 vw;
        std_msgs::Float32 yaw_angle;
        std_msgs::Float32 pitch_angle;
        std_msgs::Float32 roll_angle;

        seri_dev.receiveData(info_data);

        vx.data = info_data.chassis_vx.int16_d;
        vy.data = info_data.chassis_vy.int16_d;
        vw.data = info_data.chassis_vw.int16_d;
        yaw_angle.data = info_data.yaw_angle.float_d;
        pitch_angle.data = info_data.pitch_angle.float_d;
        roll_angle.data = info_data.roll_angle.float_d;

        chassis_vx_pub.publish(vx);
        chassis_vy_pub.publish(vy);
        chassis_vw_pub.publish(vw);
        if (yaw_angle.data<180&&yaw_angle.data>-180)
        {
            if(yaw_angle.data>0.1||yaw_angle.data<-0.1){
                gimbal_yaw_pub.publish(yaw_angle);
            }
        }
        if (pitch_angle.data<180&&pitch_angle.data>-180&&(pitch_angle.data>0.1||pitch_angle.data<-0.1))
        {
            gimbal_pitch_pub.publish(pitch_angle);
        }
        if (roll_angle.data<180&&roll_angle.data>-180&&(roll_angle.data>0.1||roll_angle.data<-0.1))
        {
            gimbal_roll_pub.publish(roll_angle);
        }
        ros::spinOnce();
    }
    return 0;
}