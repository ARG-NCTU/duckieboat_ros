#! /usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header, Bool, Int32
from nav_msgs.msg import Odometry
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
        self.last_goal = False
        self.no_dock_pose = True
        self.nav_to_gate_action = True
        # self.points = Queue.Queue(maxsize=20)
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
        ## 0: nav to gate, 1: circular, 2: return, 3: done
        self.task_state.data = 0
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped,queue_size=1)
        self.pub_task_state = rospy.Publisher("sound_task_state", Int32, queue_size=1)
        self.pub_nav_to_gate_state = rospy.Publisher("nav_to_gate_state", Bool, queue_size=1)
        
        self.sub_odom = rospy.Subscriber("localization_gps_imu/odometry", Odometry, self.cb_odom, queue_size=1)
        # self.sub_pose = rospy.Subscriber("sound_pose", PoseStamped, self.cb_sound, queue_size=1)
        self.sub_far_pose = rospy.Subscriber("sound_far_pose", PoseStamped, self.cb_far_sound, queue_size=1)
        rospy.loginfo("Wait for gate pose")
        sound_pose_info = rospy.wait_for_message('sound_pose', PoseStamped)
        if(self.task_state.data == 0):
            self.gate_goal[0] = sound_pose_info.pose.position.x
            self.gate_goal[1] = sound_pose_info.pose.position.y-5
            self.goal[0] = sound_pose_info.pose.position.x
            self.goal[1] = sound_pose_info.pose.position.y


    def cb_far_sound(self, msg):
        dis_far_totem = math.sqrt(math.pow(self.x-msg.pose.position.x,2)+math.pow(self.y-msg.pose.position.y,2))
        if(dis_far_totem > 20 and self.task_state.data<=1):
            self.task_state.data = 1
            self.far_totem_pose[0] = msg.pose.position.x
            self.far_totem_pose[1] = msg.pose.position.y
            str_info = "x="+ str(self.far_totem_pose[0])+", y="+str(self.far_totem_pose[1])+", format=radial, radius=12.5, pts="+str(self.sub_goal_num) 
            waypointploy = pyivp.string2Poly(str_info)
            waypointseg = waypointploy.exportSegList(self.x, self.y)
            sub_goal = [[0 ,0] for i in range(self.sub_goal_num)]
            for i in range(self.sub_goal_num):
                sub_goal[i][0] = waypointseg.get_vx(i)
                sub_goal[i][1] = waypointseg.get_vy(i)
                    
            self.sub_goal_points = queue.Queue(maxsize=20)
            for i in range(self.sub_goal_num):
                self.sub_goal_points.put(sub_goal[i])
            self.goal = self.sub_goal_points.get()


    # def cb_sound(self, msg):
    #     if(self.task_state.data == 0 and self.start_sound==False):
    #         self.gate_goal[0] = msg.pose.position.x
    #         self.gate_goal[1] = msg.pose.position.y-5
    #         self.goal[0] = msg.pose.position.x
    #         self.goal[1] = msg.pose.position.y
    #         self.start_sound = True


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
                print("circling")
                if(self.sub_goal_points.empty()==True):
                    print("circling done")
                    self.task_state.data = 2
                    self.goal = self.gate_goal
                else:
                    self.goal = self.sub_goal_points.get()
        elif(self.task_state.data == 2):
            if dis< self.radius:
                print("end")
                self.task_state.data = 3
        
        if(self.task_state.data < 3):
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.goal[0]
            pose.pose.position.y = self.goal[1]
            self.pub_goal.publish(pose)

        self.pub_task_state.publish(self.task_state)
        self.pub_nav_to_gate_state.publish(self.nav_to_gate_state)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('sound_task',anonymous=False)
    sound_task_node = sound_task()
    rospy.on_shutdown(sound_task_node.on_shutdown)
    rospy.spin()