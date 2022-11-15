#! /usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header, Bool
from nav_msgs.msg import Odometry
import numpy as np
import Queue
import tf
import csv
from scipy.spatial.transform import Rotation as R
from behavior_tree_msgs.msg import Active
from std_srvs.srv import Empty
import yaml

class docking_task():
    def __init__(self):
        self.node_name = rospy.get_name()
        # initiallize boat status

        self.radius = rospy.get_param("~goal_dis", 4)
        self.last_goal = False
        self.no_dock_pose = True
        self.docking_action = True
        # self.points = Queue.Queue(maxsize=20)
        self.goal = [0, 0]
        
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped,queue_size=1)
        
        self.sub_dock_action = rospy.Subscriber("/docking_active", Active, self.cb_docking_action, queue_size=1)
        self.sub_odom = rospy.Subscriber("localization_gps_imu/odometry", Odometry, self.cb_odom, queue_size=1)
        self.sub_pose = rospy.Subscriber("docking_pose", PoseStamped, self.cb_dock, queue_size=1)

    def cb_docking_action(self, msg):
        self.docking_action = msg.active

    def cb_dock(self, msg):
        self.no_dock_pose = False
        # if self.last_goal == False:
        #     print("Position goal")
        #     self.goal[0] = msg.pose.position.x
        #     self.goal[1] = msg.pose.position.y-12

        # elif self.last_goal == True:
        #     print("Dock position goal")
        #     self.goal[0] = msg.pose.position.x
        #     self.goal[1] = msg.pose.position.y-2
        self.goal[0] = msg.pose.position.x
        self.goal[1] = msg.pose.position.y


    def cb_odom(self, msg):
        # if self.no_dock_pose == True:
        #     self.goal = [msg.pose.pose.position.x+3, msg.pose.pose.position.y+10]

        odom = msg
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        dis = math.sqrt(math.pow(x- self.goal[0],2)+math.pow(y- self.goal[1],2))
        if dis< self.radius:
            print("Dock finish")
            # elif self.last_goal == False:
            #     self.last_goal = True
                # print("Position Ready, Start dock")
        
        if self.docking_action == True and self.no_dock_pose==False:
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.goal[0]
            pose.pose.position.y = self.goal[1]
            self.pub_goal.publish(pose)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('docking_task',anonymous=False)
    docking_task_node = docking_task()
    rospy.on_shutdown(docking_task_node.on_shutdown)
    rospy.spin()