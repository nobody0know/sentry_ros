#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import functools
import py_trees
import rospy
from robot_msgs.msg import robot_ctrl
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np 
import move_base_msgs.msg as move_base_msgs
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from geometry_msgs.msg import Quaternion
import actionlib
from actionlib_msgs.msg import GoalStatus
import math
class Stop_behaviour(py_trees.behaviour.Behaviour):
    def __init__(self,name="Stop"):
        super().__init__(name)
    
    def update(self):
        return py_trees.Status.RUNNING
class pursue(py_trees.behaviour.Behaviour):
    def __init__(self,name="pursue",distance=0):
        self.distance=distance
        super().__init__(name)
    
    def creat_goal(self,x,y,yaw):
        goal_pose=move_base_msgs.MoveBaseGoal()
        # 将上面的Euler angles转换成Quaternion的格式
        q_angle=quaternion_from_euler(0, 0, yaw,axes='sxyz')
        q=Quaternion(*q_angle)
        goal_pose.target_pose.header.frame_id='map'
        goal_pose.target_pose.pose.position.x=x
        goal_pose.target_pose.pose.position.y=y
        goal_pose.target_pose.pose.position.z=0
        goal_pose.target_pose.pose.orientation=q
        return goal_pose
        
    def setup(self, timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client=actionlib.SimpleActionClient("/move_base",move_base_msgs.MoveBaseAction)
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False
        return True
    def initialise(self):
        rospy.Subscriber("aim_point", PointStamped, self.aim_point_callback)
        self.tfBuffer=tf2_ros.buffer()
        self.posincam=PointStamped()
        self.begin_time=rospy.Time(0)
        self.last_send_time=0
    
        
    def aim_point_callback(self,msg):
        self.posincam=msg.point
    def caculate_pose_in_map(self):
        #transform camera to map 
        transform=self.tfBuffer.lookup_transform("map","camera",rospy.Time(0),rospy.Duration(1.0))
        posestamp_incam=tf2_geometry_msgs.PoseStamped()
        posestamp_incam.header.stamp=rospy.Time(0)
        posestamp_incam.header.frame_id="camera"
        posestamp_incam.pose.position.x=self.posincam.x
        posestamp_incam.pose.position.y=self.posincam.y
        posestamp_incam.pose.position.z=self.posincam.z
        #may be we can get opposite robot angle after lmx great effort
        posestamp_incam.pose.orientation=np.array([0,0,0,0])
        posestamp_inmap=tf2_geometry_msgs.do_transorm_pose(posestamp_incam,transform)
        
        quat=np.array([
            posestamp_inmap.pose.orientation.x,
            posestamp_inmap.pose.orientation.y,
            posestamp_inmap.pose.orientation.z,
            posestamp_inmap.pose.orientation.w
        ])
        pose_angle=euler_from_quaternion(quat)[2]
        poseinmap=[posestamp_inmap.pose.position.x,posestamp_inmap.pose.position.y,pose_angle]
        return poseinmap
    def update(self):
        if self.posincam.x==0 and self.posincam.y==0:
            return py_trees.Status.FAILURE
        pos_inmap=self.caculate_pose_in_map()
        self.goodpose=[0,0,0]
        #caculate a pose that whit a bit distance between the robot
        self.goodpose[0]=pos_inmap[0]+self.distance*math.cos(pos_inmap[2]+math.pi)
        self.goodpose[1]=pos_inmap[1]+self.distance*math.sin(pos_inmap[2]+math.pi)
        self.goodpose[2]=pos_inmap[2] 
        #not always sent goal 
        if (rospy.Time(0)-self.last_send_time)<3:
            self.action_client.send_goal(self.creat_goal(self.goodpose[0], self.goodpose[1],self.goodpose[2]))
        self.last_send_time=rospy.Time(0)
        return py_trees.Status.RUNNING

class goback(py_trees.behaviour.Behaviour):
    def __init__(self, name="goback", *args, **kwargs):
        super().__init__(name, *args, **kwargs)
    def initialise(self):
        rospy.Subscriber("")
        return super().initialise()
class detect(py_trees.behaviour.Behaviour):
    def __init__(self,name="detect") :
        super().__init__(name)
        
        
    def id_callback(self,msg):
        self.aim_id=msg.aim_id
    def initialise(self):
        self.aim_id=0
        rospy.Subscriber("robot_ctrl",robot_ctrl , self.id_callback)
    def update(self):
        if self.aim_id==0:
            return py_trees.Status.RUNNING
        else :
            rospy.INFO("now detect robot is %s",self.aim_id)
            return py_trees.Status.SUCCESS


                
