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
from robot_msgs.msg import vision
from robot_msgs.msg import op_command
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

class check_placeRed(py_trees.behaviour.Behaviour):
    def __init__(self,name="check_palce",goal_places=[]):
        super().__init__(name)
        places_inMB=[]
        self.blackboard=py_trees.blackboard.Blackboard()
        for goal_place in goal_places:
            places_inMB.append([False,self.creat_goal(goal_place[0],goal_place[1],goal_place[2])])
        self.blackboard.set("places_inMB",places_inMB)       
    def game_id_callback(self,msg):
        self.game_id=int(msg.id)
    
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
        self.game_id=None
        rospy.Subscriber("vision_data",vision,self.game_id_callback)
    #     print("init")
    def update(self):

        if self.game_id==107:
            return py_trees.Status.FAILURE
        elif self.game_id==None:
            return py_trees.Status.RUNNING
        elif self.game_id==7:
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
class check_placeBlue(py_trees.behaviour.Behaviour):
    def __init__(self,name="check_palce",goal_places=[]):
        super().__init__(name)
        places_inMB=[]
        self.blackboard=py_trees.blackboard.Blackboard()
        for goal_place in goal_places:
            places_inMB.append([False,self.creat_goal(goal_place[0],goal_place[1],goal_place[2])])
        self.blackboard.set("places_inMB_blue",places_inMB)       
    
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
        now_place_inMB=self.blackboard.get("places_inMB_blue")
        if now_place_inMB[self.index][0]==False:
            self.blackboard.set("next_goal",now_place_inMB[self.index][1])
            now_place_inMB[self.index][0]=True
            self.blackboard.set("places_inMB_blue",now_place_inMB)
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
                self.blackboard.set("places_inMB_blue",now_place_inMB) 
                return py_trees.Status.RUNNING


class goto_place(action_behavior):
    def __init__(self):
        super().__init__(name="goto_place",action_namespace="/move_base",override_feedback_message_on_running="moving")
    def initialise(self):
        blackboard=py_trees.blackboard.Blackboard()
        self.action_goal=blackboard.get("next_goal")
        super().initialise()

class start_game(py_trees.behaviour.Behaviour):
    def __init__(self,name="startGame") -> None:
        super().__init__(name)
        
        
    def gameStateCallbacl(self,msg):
        self.gameState=msg.game_state
        
    def game_id_callback(self,msg):
        self.game_id=msg.id
    
    def initialise(self):
        self.gameState=None
        self.palceSwitch=1
        self.sendState=False
        
        rospy.Subscriber("operator_command",op_command,self.gameStateCallbacl)
        rospy.Subscriber("vision_data",vision,self.game_id_callback)
  
    def update(self):
        if self.gameState==None:
            return py_trees.Status.RUNNING
        elif self.gameState!=0:
            return py_trees.Status.SUCCESS
        return py_trees.Status.RUNNING
   

class isDefend(py_trees.behaviour.Behaviour):
    def __init__(self,name="defend") -> None:
        super().__init__(name)
        
        
    def gameStateCallbacl(self,msg):
        self.gameState=msg.game_state
        
    def game_id_callback(self,msg):
        self.game_id=msg.id
    
    def initialise(self):
        self.gameState=None
        self.palceSwitch=1
        self.sendState=False
        
        rospy.Subscriber("operator_command",op_command,self.gameStateCallbacl)
        rospy.Subscriber("vision_data",vision,self.game_id_callback)
  
    def update(self):
        if self.gameState==2:
            return py_trees.Status.SUCCESS
        return py_trees.Status.RUNNING
      
class defend(py_trees.behaviour.Behaviour):
    def __init__(self,name="defend",redGoal=[[],[]],blueGoal=[[],[]]) -> None:
        self.redGoal=redGoal
        self.blueGoal=blueGoal
        super().__init__(name)
        
    def gameStateCallbacl(self,msg):
        self.gameState=msg.game_state
        
    def game_id_callback(self,msg):
        self.game_id=msg.id
    def setup(self, timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client=actionlib.SimpleActionClient("/move_base",move_base_msgs.MoveBaseAction)
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False
        return True
    def initialise(self):
        self.gameState=None
        self.palceSwitch=1
        self.sendState=False
        self.game_id=None
        rospy.Subscriber("operator_command",op_command,self.gameStateCallbacl)
        rospy.Subscriber("vision_data",vision,self.game_id_callback)
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
    def update(self):
        if self.game_id==None:
            return py_trees.Status.RUNNING
        elif self.game_id==7:
            #red
            if self.palceSwitch==1 and (not self.sendState) :
                self.action_client.send_goal(self.creat_goal(self.redGoal[0][0], self.redGoal[0][1],self.redGoal[0][2]))
                self.sendState=True
                self.palceSwitch=2
            elif self.palceSwitch==2 and (not self.sendState):
                self.action_client.send_goal(self.creat_goal(self.redGoal[1][0], self.redGoal[1][1],self.redGoal[1][2]))
                self.sendState=True
                self.palceSwitch=1
        elif self.game_id==107:
            if self.palceSwitch==1 and (not self.sendState) :
                self.action_client.send_goal(self.creat_goal(self.blueGoal[0][0], self.blueGoal[0][1],self.blueGoal[0][2]))
                self.sendState=True
                self.palceSwitch=2
            elif self.palceSwitch==2 and (not self.sendState):
                self.action_client.send_goal(self.creat_goal(self.blueGoal[1][0], self.blueGoal[1][1],self.blueGoal[1][2]))
                self.sendState=True
                self.palceSwitch=1
        if self.action_client.get_state in  [GoalStatus.ABORTED,GoalStatus.PREEMPTED]:
                return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        if result:
            self.sendState=False
        return py_trees.Status.RUNNING
  