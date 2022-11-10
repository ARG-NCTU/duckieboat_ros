#! /usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, String, Bool, Int32MultiArray
from nav_msgs.msg import Odometry
import numpy as np
import random
import queue
import pyivp
import csv
from std_srvs.srv import Empty
from gazebo_msgs.msg import ContactsState, ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
import yaml

class multipoint_wildlife():
    def __init__(self):
        self.node_name = rospy.get_name()
        # initiallize boat status
        
        self.pi2 = math.radians(360)
        
        
        self.radius = rospy.get_param("~goal_dis", 4) 
        print("Robot Radius: ", self.radius)
        
        self.s_shape_points = queue.Queue(maxsize=20)
        self.sub_goal_points = queue.Queue(maxsize=20)
        self.sub_goal_num = 4
        self.sub_goal_range = 10
        self.x = 0
        self.y = 0
        self.angle = 0
        self.goal_distance = 100
        self.platypus_index = 0
        self.turtle_index = 1
        self.croc_index = 2
        self.goal = None
        self.tmp_goal = None
        self.sub_task_state = Int32MultiArray()
        
        self.start_x = 10
        self.start_y = 15
        
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped,queue_size=1)
        self.pub_sub_task_state = rospy.Publisher("wildlife_task_state", Int32MultiArray, queue_size=1)
        
        self.sub_hsi_name = rospy.Subscriber("hsi/scan_result", String, self.cb_hsi_name, queue_size=1)
        self.sub_zed_goal = rospy.Subscriber("wildlife_pose", PoseStamped, self.cb_zed_goal, queue_size=1)
        self.sub_odom = rospy.Subscriber("localization_gps_imu/pose", PoseStamped, self.cb_odom, queue_size=1)

        self.task_navi_action = False
        self.wildlife_detect = False
        self.pb_action = False
        self.alignment_action = False
        self.sub_pb_action = rospy.Subscriber("patternblock_action", Bool, self.cb_pb, queue_size=1)
        self.sub_task_nav_action = rospy.Subscriber("task_navi_action", Bool, self.cb_task_navi, queue_size=1)
        self.sub_nav_wildlife_action = rospy.Subscriber("alignment_action", Bool, self.cb_alignment_nav, queue_size=1)

        self.s_shape_points = queue.Queue(maxsize=20)
        self.sub_goal_points = queue.Queue(maxsize=20)
        str_info = "format=lawnmower, x=30, y=35, height=30, width=40, lane_width=20, rows=east-west, startx=" + str(self.start_x)+", starty=" + str(self.start_y)+ ", degs=90"
        pattern = pyivp.string2SegList(str_info)
        for i in range(1, 6):
            s_x = pattern.get_vx(i)
            s_y = pattern.get_vy(i)
            self.s_shape_points.put([s_x, s_y])
        self.goal = self.s_shape_points.get()
        self.zed_goal = [self.goal[0], self.goal[1]]
        self.tmp_goal = self.goal
        self.sub_task_state.data = [0 ,0, 0]
        
    def cb_alignment_nav(self, msg):
        self.alignment_action = msg.data

    def cb_pb(self, msg):
        self.pb_action = msg.data
    
    def cb_task_navi(self, msg):
        self.task_navi_action = msg.data

    def cb_hsi_name(self, msg):
        if(msg.data == "platypus" and self.sub_task_state.data[self.platypus_index] == 0):
            rospy.sleep(0.3)
            self.sub_task_state.data[self.platypus_index] = 1
            str_info = "x="+ str(self.zed_goal[0])+", y="+str(self.zed_goal[1])+", format=radial, radius=10, pts="+str(self.sub_goal_num)
            waypointploy = pyivp.string2Poly(str_info)
            waypointseg = waypointploy.exportSegList(self.x, self.y)
            sub_goal = [[0 ,0] for i in range(self.sub_goal_num)]
            for i in range(self.sub_goal_num):
                if i==self.sub_goal_num-1:
                    sub_goal[i][0] = waypointseg.get_vx(0)
                    sub_goal[i][1] = waypointseg.get_vy(0)
                else:
                    sub_goal[i][0] = waypointseg.get_vx(i+1)
                    sub_goal[i][1] = waypointseg.get_vy(i+1)
            
            for i in range(self.sub_goal_num):
                self.sub_goal_points.put(sub_goal[i])
            self.goal = self.sub_goal_points.get()
            # print("new goal")

        elif(msg.data == "turtle" and self.sub_task_state.data[self.turtle_index] == 0):
            rospy.sleep(0.3)
            print("find turtle")
            self.sub_task_state.data[self.turtle_index] = 1
            str_info = "x="+ str(self.zed_goal[0])+", y="+str(self.zed_goal[1])+", format=radial, radius=10, pts="+str(self.sub_goal_num)
            waypointploy = pyivp.string2Poly(str_info)
            waypointseg = waypointploy.exportSegList(self.x, self.y)
            waypointseg.reverse()
            sub_goal = [[0 ,0] for i in range(self.sub_goal_num)]
            for i in range(self.sub_goal_num):
                sub_goal[i][0] = waypointseg.get_vx(i)
                sub_goal[i][1] = waypointseg.get_vy(i)
        
            for i in range(self.sub_goal_num):
                self.sub_goal_points.put(sub_goal[i])
            self.goal = self.sub_goal_points.get()
            # print("new goal")

        elif(msg.data == "crocodile" and self.sub_task_state.data[self.croc_index] == 0):
            rospy.sleep(0.3)
            self.sub_task_state.data[self.croc_index] = 1
            str_info = "x="+ str(self.zed_goal[0])+", y="+str(self.zed_goal[1])+", format=radial, radius=10, pts="+str(self.sub_goal_num)
            waypointploy = pyivp.string2Poly(str_info)
            waypointseg = waypointploy.exportSegList(self.x, self.y)
            sub_goal = [[0 ,0] for i in range(self.sub_goal_num)]
            for i in range(self.sub_goal_num):
                if i==self.sub_goal_num-1:
                    sub_goal[i][0] = waypointseg.get_vx(0)
                    sub_goal[i][1] = waypointseg.get_vy(0)
                else:
                    sub_goal[i][0] = waypointseg.get_vx(i+1)
                    sub_goal[i][1] = waypointseg.get_vy(i+1)
            
            for i in range(self.sub_goal_num):
                self.sub_goal_points.put(sub_goal[i])
            for i in range(self.sub_goal_num):
                self.sub_goal_points.put(sub_goal[i])
            self.goal = self.sub_goal_points.get()

    def cb_zed_goal(self, msg):
        if(self.sub_task_state.data[self.platypus_index]!=1 and self.sub_task_state.data[self.turtle_index]!=1 and self.sub_task_state.data[self.croc_index]!=1):
            x_1 = msg.pose.position.x - 10* math.cos(self.angle)
            y_1 = msg.pose.position.y - 10* math.sin(self.angle)
            self.goal = [x_1, y_1]
            self.zed_goal = [msg.pose.position.x, msg.pose.position.y]
            self.wildlife_detect = True


    def cb_odom(self, msg):
        if self.goal is None or self.zed_goal is None:
            return
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.angle = math.atan2(self.zed_goal[1]-self.y, self.zed_goal[0]-self.x)
        dis = math.sqrt(math.pow(self.x- self.goal[0],2)+math.pow(self.y- self.goal[1],2))
        if(self.sub_task_state.data[self.platypus_index]==1 and self.task_navi_action==True):
            print("platypus checking")
            if(dis< self.radius):
                if(self.sub_goal_points.empty()==True):
                    print("platypus sub goal done")
                    self.sub_task_state.data[self.platypus_index] = 2
                    if(self.sub_task_state.data[self.platypus_index]==2 and self.sub_task_state.data[self.turtle_index]==2 and self.sub_task_state.data[self.croc_index]==2):
                        print("finish")
                        return
                    self.goal = self.tmp_goal
                    self.wildlife_detect = False
                else:
                    self.goal = self.sub_goal_points.get()

        elif(self.sub_task_state.data[self.turtle_index]==1 and self.task_navi_action==True):
            print("turtle checking")
            if(dis< self.radius):
                if(self.sub_goal_points.empty()==True):
                    print("turtle sub goal done")
                    self.sub_task_state.data[self.turtle_index] = 2
                    if(self.sub_task_state.data[self.platypus_index]==2 and self.sub_task_state.data[self.turtle_index]==2 and self.sub_task_state.data[self.croc_index]==2):
                        print("finish")
                        return
                    self.goal = self.tmp_goal
                    self.wildlife_detect = False
                else:
                    self.goal = self.sub_goal_points.get()

        elif(self.sub_task_state.data[self.croc_index]==1 and self.task_navi_action==True):
            print("crocodile checking")
            if(dis< self.radius):
                if(self.sub_goal_points.empty()==True):
                    print("crocodile sub goal done")
                    self.sub_task_state.data[self.croc_index] = 2
                    if(self.sub_task_state.data[self.platypus_index]==2 and self.sub_task_state.data[self.turtle_index]==2 and self.sub_task_state.data[self.croc_index]==2):
                        print("finish")
                        return
                    self.goal = self.tmp_goal
                    self.wildlife_detect = False
                else:
                    self.goal = self.sub_goal_points.get()

        elif(self.pb_action==True and self.wildlife_detect == False):
            print("searching")
            if dis< self.radius:
                print("switch_goal")
                self.goal = self.s_shape_points.get()
                self.tmp_goal = self.goal

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.goal[0]
        pose.pose.position.y = self.goal[1]
        if self.alignment_action == False:
            self.pub_goal.publish(pose)
        self.pub_sub_task_state.publish(self.sub_task_state)

        # print(self.goal)

    def get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('multipoint_wildlife_node',anonymous=False)
    multipoint_wildlife_node = multipoint_wildlife()
    rospy.on_shutdown(multipoint_wildlife_node.on_shutdown)
    rospy.spin()