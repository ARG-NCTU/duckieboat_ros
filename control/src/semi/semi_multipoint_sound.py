#! /usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header, Bool, Int32
from nav_msgs.msg import Odometry
from behavior_tree_msgs.msg import Active
import numpy as np
import queue
import pyivp
import csv
from std_srvs.srv import Empty
import yaml

class sound_task():
    def __init__(self):
        self.node_name = rospy.get_name()
        # initiallize boat status

        self.radius = rospy.get_param("~goal_dis", 4)
        self.pt_list = []
        self.goal = [0, 0]
        self.gate_goal = [0, 0]
        self.start_sound = False
        self.sub_goal_num = 3
        self.sub_goal_points = queue.Queue(maxsize=20)
        self.far_totem_pose = [0, 0]
        self.x = 0
        self.y = 0
        self.task_state = Int32()
        self.nav_to_gate_state = Bool()
        self.nav_to_gate_state.data = False

        self.nav_return_state = Bool()
        self.nav_return_state.data = False
        self.return_gate_action = False
        self.gate_action = False
        ## 0: nav to gate, 1: return, 2: done
        self.task_state.data = 0
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped,queue_size=1)
        self.pub_task_state = rospy.Publisher("sound_task_state", Int32, queue_size=1)
        self.pub_nav_to_gate_state = rospy.Publisher("/nav_perception_gate_finished_success", Bool, queue_size=1)
        self.pub_nav_return_state = rospy.Publisher("/nav_gate_finished_success", Bool, queue_size=1)
        
        self.sub_odom = rospy.Subscriber("localization_gps_imu/odometry", Odometry, self.cb_odom, queue_size=1)
        self.sub_gate_action = rospy.Subscriber("/nav_perception_gate_active", Active, self.cb_gate_tree, queue_size=1)
        self.sub_return = rospy.Subscriber("/nav_gate_active", Active, self.cb_return_tree, queue_size=1)
        # self.sub_pose = rospy.Subscriber("sound_pose", PoseStamped, self.cb_sound, queue_size=1)
        rospy.loginfo("Wait for gate pose")
        sound_pose_info = rospy.wait_for_message('sound_pose', PoseStamped)
        if(self.task_state.data == 0):
            print("get gate pose")
            self.gate_goal[0] = sound_pose_info.pose.position.x
            self.gate_goal[1] = sound_pose_info.pose.position.y-5
            self.goal[0] = sound_pose_info.pose.position.x
            self.goal[1] = sound_pose_info.pose.position.y

    # def cb_sound(self, msg):
    #     if(self.task_state.data == 0 and self.start_sound==False):
    #         self.gate_goal[0] = msg.pose.position.x
    #         self.gate_goal[1] = msg.pose.position.y-5
    #         self.goal[0] = msg.pose.position.x
    #         self.goal[1] = msg.pose.position.y
    #         self.start_sound = True

    def cb_gate_tree(self, msg):
        self.gate_action = msg.active


    def cb_return_tree(self, msg):
        self.return_gate_action = msg.active
        if(self.return_gate_action == True and self.task_state.data<=1):
            self.task_state.data = 1
            self.goal = self.gate_goal

    def cb_odom(self, msg):
        odom = msg
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        dis = math.sqrt(math.pow(self.x- self.goal[0],2)+math.pow(self.y- self.goal[1],2))
        if(self.task_state.data == 0):
            if dis< self.radius:
                print("gate reach")
                self.nav_to_gate_state.data = True
                
        elif(self.task_state.data == 1):
            if dis< self.radius:
                print("return end")
                self.nav_return_state.data = True
                self.task_state.data = 2
        elif(self.task_state.data == 2):
            print("semi running end")
        
        if(self.task_state.data < 2):
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.goal[0]
            pose.pose.position.y = self.goal[1]
            if(self.gate_action == True or self.return_gate_action == True):
                self.pub_goal.publish(pose)

        self.pub_task_state.publish(self.task_state)
        self.pub_nav_to_gate_state.publish(self.nav_to_gate_state)
        self.pub_nav_return_state.publish(self.nav_return_state)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('sound_task',anonymous=False)
    sound_task_node = sound_task()
    rospy.on_shutdown(sound_task_node.on_shutdown)
    rospy.spin()