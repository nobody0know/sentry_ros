#include"chassis_odom.h"
c_odom chassis_odom;
void vx_Callback(const std_msgs::Int16 vx)
{
    chassis_odom.vx = vx.data;
}

void vy_Callback(const std_msgs::Int16 vy)
{
    chassis_odom.vy = vy.data;
}

void vw_Callback(const std_msgs::Int16 vw)
{
    chassis_odom.vw = vw.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    odom::chassis_odom_pub chassis_odom_pub(n);
    ros::Subscriber vx_sub = n.subscribe<std_msgs::Int16>("chassis_vx", 1000, vx_Callback);
    ros::Subscriber vy_sub = n.subscribe<std_msgs::Int16>("chassis_vy", 1000, vy_Callback);
    ros::Subscriber vw_sub = n.subscribe<std_msgs::Int16>("chassis_vw", 1000, vw_Callback);
    while (n.ok())
    {
        chassis_odom_pub.publish_odom();
        ros::spinOnce();
    }
    
}
namespace odom{
    chassis_odom_pub::chassis_odom_pub(ros::NodeHandle &n)
    {
        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
        current_time = ros::Time::now();
        last_time = ros::Time::now();
    }
    void chassis_odom_pub::publish_odom()
    {
        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        double delta_x = (chassis_odom.vx * cos(chassis_odom.vw) - chassis_odom.vy * sin(chassis_odom.vy) * dt);
        double delta_y = (chassis_odom.vx * sin(chassis_odom.vw) + chassis_odom.vy * cos(chassis_odom.vy) * dt);
        double delta_th = chassis_odom.vw * dt;
        chassis_odom.x_pos += delta_x;
        chassis_odom.y_pos += delta_y;
        chassis_odom.z_pos += delta_th;
        
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(chassis_odom.vw);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = chassis_odom.x_pos;
        odom_trans.transform.translation.y = chassis_odom.y_pos;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.orientation.x = chassis_odom.x_pos;
        odom.pose.pose.orientation.y = chassis_odom.y_pos;
        odom.pose.pose.orientation.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = chassis_odom.vx;
        odom.twist.twist.linear.y = chassis_odom.vy;
        odom.twist.twist.angular.z = chassis_odom.vw;

        odom_pub.publish(odom);
        last_time = current_time;
    }
}

