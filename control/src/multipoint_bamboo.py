#! /usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import numpy as np
import Queue
import tf
import csv
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray
import yaml

class navigation_task():
    def __init__(self):
        self.node_name = rospy.get_name()
        # initiallize boat status
        self.auto = 0
        self.epoch = 0
        self.count = 0
        self.obs_list = []
        # with open("/home/argrobotx/robotx-2022/duckiepond-devices/obs_pos.yaml") as file:
        #     total_obs_list = yaml.safe_load(file)

        # for key, value in total_obs_list.items():
        #     for pos_key, pos_value in value.items():
        #         self.obs_list.append(pos_value)
        # print(len(self.obs_list))

        self.pt_list = [(25, -10), (25, -35), (10, -35), (10, -10), (-5, -10), (-5, -35), (-20, -35), (-20, -10)]

        self.line_pt_list = [[(25, -10), (25, -35)], [(25, -35), (10, -35)], [(10, -10), (10, -35)], [(10, -10), (-5, -10)], [(-5, -10), (-5, -35)], \
        [(-5, -35), (-20, -35)], [(-20, -10), (-20, -35)]]
        # print(len(self.pt_list))
        # print(len(self.line_pt_list))
        
        self.pt_length = len(self.pt_list)
        self.stop = False
        self.last = False
        self.radius = 6

        print("Robot Radius: ", self.radius)
        
        self.points = Queue.Queue(maxsize=50)
        for i,pt in enumerate(self.pt_list):
            self.points.put(pt)

        self.goal = self.points.get()


        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped,queue_size=1)
        
        self.pub_points = rospy.Publisher("visualization_marker", MarkerArray, queue_size=1)
        self.pub_line = rospy.Publisher("visualization_line", MarkerArray, queue_size=1)
        self.Markers = MarkerArray()
        self.Markers.markers = []
        self.Line_markers = MarkerArray()
        self.Line_markers.markers = []

        self.sub = rospy.Subscriber("localization_gps_imu/pose", PoseStamped, self.cb_odom, queue_size=1)
    
        self.get_marker()
        self.get_line()


    def cb_odom(self, msg):
        odom = msg
        x = odom.pose.position.x
        y = odom.pose.position.y
        dis = math.sqrt(math.pow(x- self.goal[0],2)+math.pow(y- self.goal[1],2))


        if dis< self.radius and self.stop == False:
            self.points.put(self.goal)
            if self.goal == self.pt_list[self.pt_length-1]:
                self.last = True
                print("last goal reach")

            elif self.last == False:
                self.goal = self.points.get()
                print("next goal")

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.goal[0]
        pose.pose.position.y = self.goal[1]
        self.pub_goal.publish(pose)
        self.pub_points.publish(self.Markers)
        self.pub_line.publish(self.Line_markers)
        # print(self.goal)

    def get_marker(self):
        i = 1
        for pt in self.pt_list:
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'map'
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.orientation.w = 1
            marker.pose.position.x = pt[0]
            marker.pose.position.y = pt[1]
            marker.id = i
            i = i+1
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 1
            self.Markers.markers.append(marker)

    def get_line(self):
        count = 1
        for line_pair in self.line_pt_list:
            line_marker = Marker()
            line_marker.header.stamp = rospy.Time.now()
            line_marker.header.frame_id = "map"
            line_marker.type = line_marker.LINE_STRIP
            line_marker.action = line_marker.ADD
            line_marker.pose.orientation.w = 1
            # marker scale
            line_marker.scale.x = 0.1
            line_marker.scale.y = 0.1
            line_marker.scale.z = 0.1
            # marker color
            line_marker.color.a = 1.0
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.id = count
            count += 1
            line_marker.points = []
            first_line_point = Point()
            first_line_point.x = line_pair[0][0]
            first_line_point.y = line_pair[0][1]
            first_line_point.z = 0.0
            line_marker.points.append(first_line_point)
            # second point
            second_line_point = Point()
            second_line_point.x = line_pair[1][0]
            second_line_point.y = line_pair[1][1]
            second_line_point.z = 0.0
            line_marker.points.append(second_line_point)
            self.Line_markers.markers.append(line_marker)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('navigation_task',anonymous=False)
    navigation_task_node = navigation_task()
    rospy.on_shutdown(navigation_task_node.on_shutdown)
    rospy.spin()