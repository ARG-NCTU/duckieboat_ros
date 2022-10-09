#! /usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import numpy as np
import Queue
import tf
import csv
from std_srvs.srv import Empty
import yaml

class docking_task():
    def __init__(self):
        self.node_name = rospy.get_name()
        # initiallize boat status

        self.radius = rospy.get_param("~goal_dis", 3.5)
        self.last_goal = False
        self.no_dock_pose = True
        # self.points = Queue.Queue(maxsize=20)
        self.goal = [0, 0]
        
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped,queue_size=1)
        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        
        self.sub = rospy.Subscriber("localization_gps_imu/odometry", Odometry, self.cb_odom, queue_size=1)
        self.sub_pose = rospy.Subscriber("docking_pose", PoseStamped, self.cb_dock, queue_size=1)

    def cb_dock(self, msg):
        self.no_dock_pose = False
        if self.last_goal == False:
            self.goal[0] = msg.pose.position.x+12
            self.goal[1] = msg.pose.position.y

        elif self.last_goal == True:
            self.goal[0] = msg.pose.position.x+3
            self.goal[1] = msg.pose.position.y


    def cb_odom(self, msg):
        if self.no_dock_pose == True:
            # cmd_vel = Twist()
            # cmd_vel.linear.x = 0.3
            # self.pub_cmd.publish(cmd_vel)
            self.goal = [msg.pose.pose.position.x-10, msg.pose.pose.position.y-1]
            # return
        # if(self.goal == [0, 0]):
        #     return
        odom = msg
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        dis = math.sqrt(math.pow(x- self.goal[0],2)+math.pow(y- self.goal[1],2))
        if dis< self.radius:
            if self.last_goal == True:
                print("Dock reach")

            elif self.last_goal == False:
                self.last_goal = True
                print("Position ready, start docking")

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