#ifndef CHASSIS_ODOM
#define CHASSIS_ODOM

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"
namespace odom
{
    struct chassis_odom
    {
        float vx=0;
        float vy=0;
        float vw=0;

        float x_pos=0;
        float y_pos=0;
        float z_pos=0;

        ros::Publisher odom_pub;
        nav_msgs::Odometry odom;
        ros::Time current_time,last_time;
        tf::TransformBroadcaster odom_broadcaster;
    };

}
#endif