#include"chassis_odom.h"
c_odom chassis_odom;
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
    odom::chassis_odom_pub chassis_odom_pub(n);
    ros::Subscriber vx_sub = n.subscribe<std_msgs::Float32>("chassis_vx", 100, vx_Callback);
    ros::Subscriber vy_sub = n.subscribe<std_msgs::Float32>("chassis_vy", 100, vy_Callback);
    ros::Subscriber vw_sub = n.subscribe<std_msgs::Float32>("chassis_vw", 100, vw_Callback);
    ros::Rate loop_rate(100);
    while (n.ok())
    {
        chassis_odom_pub.publish_odom();
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}
namespace odom{
    chassis_odom_pub::chassis_odom_pub(ros::NodeHandle &n)
    {
        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
        current_time = ros::Time::now();
        last_time = ros::Time::now();
    }
    void chassis_odom_pub::publish_odom()
    {
        current_time = ros::Time::now();

        float dt = (current_time - last_time).toSec();
        float delta_x = (chassis_odom.vx * cos(chassis_odom.vw) - chassis_odom.vy * sin(chassis_odom.vw) * dt);
        float delta_y = (chassis_odom.vx * sin(chassis_odom.vw) + chassis_odom.vy * cos(chassis_odom.vw) * dt);
        float delta_th = chassis_odom.vw * dt;
        chassis_odom.x_pos += delta_x;
        chassis_odom.y_pos += delta_y;
        chassis_odom.z_pos += delta_th;
//      因为里程计使用麦轮解算得到,无需里程计到base_footprint的tf变换,直接从融合算法ekf发布tf变换即可    
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(chassis_odom.vw);

        odom.header.stamp = current_time;
        odom.header.frame_id = "odom_combined";

        odom.pose.pose.orientation.x = chassis_odom.x_pos;
        odom.pose.pose.orientation.y = chassis_odom.y_pos;
        odom.pose.pose.orientation.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = chassis_odom.vx;
        odom.twist.twist.linear.y = chassis_odom.vy;
        odom.twist.twist.angular.z = chassis_odom.vw;
        if(chassis_odom.vx== 0&&chassis_odom.vy== 0&&chassis_odom.vw== 0)
        {
            //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
            memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
            memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
        }
        else
        {
            //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
            memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
            memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));       
        }
        odom_pub.publish(odom);
        last_time = current_time;
    }
}

