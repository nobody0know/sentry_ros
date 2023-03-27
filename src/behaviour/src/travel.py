#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import functools
import py_trees

from actionlib_msgs.msg import GoalStatus
import rospy
import math
import actionlib
import move_base_msgs.msg as move_base_msgs
import std_msgs.msg as std_msgs
import numpy as np
import tf2_ros
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from geometry_msgs.msg import Quaternion,Twist 
from nav_msgs.msg import Odometry
class action_behavior(py_trees.behaviour.Behaviour):
    def __init__(self,name="action client",action_goal=None,action_namespace="/move_base",override_feedback_message_on_running="moving"):
        super().__init__(name)
        self.action_client=None
        self.sent_goal=False
        self.action_goal=action_goal
        self.action_namespace=action_namespace
        self.override_feedback_message_on_running=override_feedback_message_on_running

    def setup(self,timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client=actionlib.SimpleActionClient(self.action_namespace,move_base_msgs.MoveBaseAction)
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False
        return True
    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal=False
    
    def update(self):
        if not self.action_client:
            self.feedback_message="no action,did u call setup() on your tree?"
            return py_trees.Status.INVALID
        if not self.sent_goal:
            self.action_client.send_goal(self.action_goal)
            self.sent_goal=True
            self.feedback_message="sent goal to the action server"
            return py_trees.Status.RUNNING
        self.feedback_message=self.action_client.get_goal_status_text()
        if self.action_client.get_state in  [GoalStatus.ABORTED,GoalStatus.PREEMPTED]:
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        if result:
           
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = self.override_feedback_message_on_running
            return py_trees.Status.RUNNING
    def terminate(self, new_status):
        """
        If running and the current goal has not already succeeded, cancel it.

        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if ((motion_state == GoalStatus.PENDING) or (motion_state == GoalStatus.ACTIVE) or
               (motion_state == GoalStatus.PREEMPTING) or (motion_state == GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False

class check_place(py_trees.behaviour.Behaviour):
    def __init__(self,name="check_palce",goal_places=[]):
        super().__init__(name)
 
        places_inMB=[]
        self.blackboard=py_trees.blackboard.Blackboard()
        for goal_place in goal_places:
            places_inMB.append([False,self.creat_goal(goal_place[0],goal_place[1],goal_place[2])])
        self.blackboard.set("places_inMB",places_inMB)       
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
    def initialise(self):
        self.index=0
    #     print("init")
    def update(self):
        now_place_inMB=self.blackboard.get("places_inMB")
        if now_place_inMB[self.index][0]==False:
            self.blackboard.set("next_goal",now_place_inMB[self.index][1])
            now_place_inMB[self.index][0]=True
            self.blackboard.set("places_inMB",now_place_inMB)
            self.blackboard.set("places_index",self.index) 
            return py_trees.Status.SUCCESS
        else:
            
            if self.index<len(now_place_inMB)-1:
                self.index+=1
                return py_trees.Status.RUNNING
            else :
                self.index=0
                for i in range(0,len(now_place_inMB)):
                    now_place_inMB[i][0]=False
                self.blackboard.set("places_inMB",now_place_inMB) 
                return py_trees.Status.RUNNING
        


class goto_place(action_behavior):
    def __init__(self):
        super().__init__(name="goto_place",action_namespace="/move_base",override_feedback_message_on_running="moving")
    def initialise(self):
        blackboard=py_trees.blackboard.Blackboard()
        self.action_goal=blackboard.get("next_goal")
        super().initialise()

