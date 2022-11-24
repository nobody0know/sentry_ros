#ifndef CHASSIS_ODOM
#define CHASSIS_ODOM

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
typedef struct
{
    int16_t vx=0;
    int16_t vy=0;
    int16_t vw=0;

    float x_pos=0;
    float y_pos=0;
    float z_pos=0;
}c_odom;

namespace odom
{

    class chassis_odom_pub
    {
    private:
        ros::Publisher odom_pub;
        nav_msgs::Odometry odom;
        ros::Time current_time,last_time;
        tf::TransformBroadcaster odom_broadcaster;
    public:
        chassis_odom_pub(ros::NodeHandle &n);
        void publish_odom();
    };

}
#endif