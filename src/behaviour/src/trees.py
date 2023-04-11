#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import functools
import py_trees
import py_trees_ros
import py_trees.console as console
from actionlib_msgs.msg import GoalStatus
import rospy
import sys
from math import pi
from travel import action_behavior,check_place,goto_place
from composites import Stop_behaviour,pursue,detect
import move_base_msgs.msg as move_base_msgs
import std_msgs.msg as std_msgs
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt8MultiArray
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped,TransformStamped
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import math

from nav_msgs.msg import Odometry

def create_root():
    #前哨站
    # check_place_=check_place(goal_places=[[12.560, -12.010, -0.906],[21.0, -6.75, -0]])
    check_place_=check_place(goal_places=[[4.452, 0.810, -0],[1.5, -0.44, -0]])
    goto_place_=goto_place()
    root=py_trees.composites.Sequence("root")
    travel=py_trees.composites.Parallel("travel",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
    stop=Stop_behaviour()
    detect_=detect()
    pursue_=pursue()
    
    check_all_place=py_trees.behaviours.Running(name='check_all_place')
    start_travel=py_trees.composites.Sequence("start_travel")
    start_travel.add_children([check_place_,goto_place_])
    #travel behaviour never will success
    travel.add_children([start_travel,check_all_place])

    travel_and_detect=py_trees.composites.Parallel("travel_and_detect",py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    ## you had better put the travel in a parallel behaviour
    travel_and_detect.add_children([travel,detect_])

    root.add_children([travel_and_detect,pursue_,stop])
    
    # travel.add_children([aim])

    return root
def creat_goal(x,y,yaw):
    goal_pose=move_base_msgs.MoveBaseGoal()
    # 将上面的Euler angles转换成Quaternion的格式
    q_angle=quaternion_from_euler(0, 0, yaw,axes='sxyz')
    q=Quaternion(*q_angle)
    goal_pose.target_pose.header.frame_id='map'
    goal_pose.target_pose.pose.position.x=x
    goal_pose.target_pose.pose.position.y=y
    goal_pose.target_pose.pose.ponnjsition.z=0
    goal_pose.target_pose.pose.orientation=q
    return goal_pose

def shutdown(behaviour_tree):
    behaviour_tree.interrupt()
    
if __name__=="__main__":
    rospy.init_node("tree")
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(60)
    # client=actionlib.SimpleActionClient('move_base',move_base_msgs.MoveBaseAction)
    # client.wait_for_server()
    # client.send_goal(creat_goal(0,0,0))
    # client.wait_for_result()
    # if client.get_state()==GoalStatus.SUCCEEDED:
    #     rospy.loginfo("gaol_reach")
    # else :
    #     rospy.logerr("fuck")
    
