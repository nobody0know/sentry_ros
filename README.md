# sentry_ros

悍匠哨兵——ROS代码部分 🤣 👉 🤡

使用Robomaster-C型开发板上的虚拟串口功能使用“simple-robot”功能包进行与嵌入式下位机的通讯

可能会需要的与底层的接口有： 

file&address：sentry_ros/src/simple_robot/msg/vision.msg  

topic： vision_data

data：uint16 id （哨兵此时为红/蓝方）
            uint16 shoot_sta （发射控制位）
            float32 pitch （上正下负）
            float32 yaw （右手向上螺旋方向为正）
            float32 roll （右手拇指朝前螺旋方向为正）
            float32 shoot （射速）
