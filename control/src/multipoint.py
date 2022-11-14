#! /usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import Queue

r = rospy.get_param("~goal_dis", 4.5)
points = Queue.Queue(maxsize=20)

reverse = False
pt_list =  [(340, 188), (328, 200),(311, 204),(297, 203),(290, 204),(277, 209),(265, 212),(256, 210)]
p_list = []

if reverse :
    while len(pt_list) != 0:
        p_list.append(pt_list.pop())
else :
    p_list = pt_list


print(p_list)
for i,point in enumerate(p_list):
    points.put(point)


goal = points.get()
rospy.init_node("multi_waypoint")
sim = rospy.get_param("~nav/sim",False)
pub = rospy.Publisher("/move_base_simple/goal", PoseStamped,queue_size=1)

def cb_odom(msg):
    odom = msg
    global goal
    x = odom.pose.position.x
    y = odom.pose.position.y
    dis = math.sqrt(math.pow(x- goal[0],2)+math.pow(y- goal[1],2))
    if dis<r:
        points.put(goal)
        goal = points.get()

    pose = PoseStamped()
    pose.header = Header()
    pose.header.frame_id = "map"
    pose.pose.position.x = goal[0]
    pose.pose.position.y = goal[1]
    pub.publish(pose)
    print goal

odomerty_name=''
if sim:
    odomerty_name = "p3d_odom"
else:
    odomerty_name = "localization_gps_imu/pose"

sub = rospy.Subscriber(odomerty_name,PoseStamped,cb_odom,queue_size=1)

rospy.spin()
