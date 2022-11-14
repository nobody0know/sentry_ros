#include"serial_device_subscriber_node.h"
sentry_control ctr_data;
void cmdCallback(const geometry_msgs::Twist &cmd_data)
{
    ctr_data.chassis_vx.float_d = cmd_data.linear.x;
    ctr_data.chassis_vy.float_d = cmd_data.linear.y;
    ctr_data.chassis_vw.float_d = cmd_data.angular.z;
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"hard_ware_subscribe_node");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cmd_v",1000,cmdCallback);

    ros::spin();

    return 0;
}