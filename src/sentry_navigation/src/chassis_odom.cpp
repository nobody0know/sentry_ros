#include"chassis_odom.h"

odom::chassis_odom chassis_odom;
void publish_odom(odom::chassis_odom &chassis_odom)
{
    chassis_odom.current_time = ros::Time::now();

    double dt = (chassis_odom.current_time - chassis_odom.last_time).toSec();
    double delta_x = (chassis_odom.vx * cos(chassis_odom.vw) - chassis_odom.vy * sin(chassis_odom.vy) * dt);
    double delta_y = (chassis_odom.vx * sin(chassis_odom.vw) + chassis_odom.vy * cos(chassis_odom.vy) * dt);
    double delta_th = chassis_odom.vw * dt;
    chassis_odom.x_pos += delta_x;
    chassis_odom.y_pos += delta_y;
    chassis_odom.z_pos += delta_th;
    
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(chassis_odom.vw);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = chassis_odom.current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = chassis_odom.x_pos;
    odom_trans.transform.translation.y = chassis_odom.y_pos;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    chassis_odom.odom_broadcaster.sendTransform(odom_trans);

    chassis_odom.odom.header.stamp = chassis_odom.current_time;
    chassis_odom.odom.header.frame_id = "odom";

    chassis_odom.odom.pose.pose.orientation.x = chassis_odom.x_pos;
    chassis_odom.odom.pose.pose.orientation.y = chassis_odom.y_pos;
    chassis_odom.odom.pose.pose.orientation.z = 0.0;
    chassis_odom.odom.pose.pose.orientation = odom_quat;
    
    chassis_odom.odom.child_frame_id = "base_link";
    chassis_odom.odom.twist.twist.linear.x = chassis_odom.vx;
    chassis_odom.odom.twist.twist.linear.y = chassis_odom.vy;
    chassis_odom.odom.twist.twist.angular.z = chassis_odom.vw;

    chassis_odom.odom_pub.publish(chassis_odom.odom);
    chassis_odom.last_time = chassis_odom.current_time;
}

void vx_Callback(const std_msgs::Float32 vx)
{
    chassis_odom.vx = vx.data;
}

void vy_Callback(const std_msgs::Float32 vy)
{
    chassis_odom.vy = vy.data;
}

void vw_Callback(const std_msgs::Float32 vw)
{
    chassis_odom.vw = vw.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Subscriber vx_sub = n.subscribe<std_msgs::Float32>("chassis_vx", 1000, vx_Callback);
    ros::Subscriber vy_sub = n.subscribe<std_msgs::Float32>("chassis_vy", 1000, vy_Callback);
    ros::Subscriber vw_sub = n.subscribe<std_msgs::Float32>("chassis_vw", 1000, vw_Callback);

    chassis_odom.odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    chassis_odom.current_time = ros::Time::now();
    chassis_odom.last_time = ros::Time::now();

    ros::Rate r(1);
    while (n.ok())
    {
        publish_odom(chassis_odom);
        r.sleep();   
    }
    
}