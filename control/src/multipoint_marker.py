#! /usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import queue

r = 7.5
points = queue.Queue(maxsize=20)

reverse = False
# pt_list =  [(-3,-1),(-0.5,-13),(2,-25),(12,-23),(4.5,-12.5),(3,-2.5),(9,-4),(15.5,-12.5),(22,-21),(32,-19),(23.5,-12),(15,-5.5),(20,-7),(31,-12),(42,-17),(2,-25),(-3,-1)]
pt_list = [(-71, 181), (-71, 151), (-41, 151), (-41, 181)]
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
pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
pub_points = rospy.Publisher("visualization_marker", MarkerArray, queue_size=1)
Markers = MarkerArray()
Markers.markers = []

def get_marker():
    i = 0
    for pt in pt_list:
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'odom'
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1
        marker.pose.position.x = pt[0]
        marker.pose.position.y = pt[1]
        marker.id = i
        i = i+1
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 1.0
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        Markers.markers.append(marker)

get_marker()

def cb_odom(msg):
    odom = msg
    global goal
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    dis = math.sqrt(math.pow(x- goal[0],2)+math.pow(y- goal[1],2))
    if dis<r:
        points.put(goal)
        goal = points.get()

    pose = PoseStamped()
    pose.header = Header()
    pose.header.frame_id = "odom"
    pose.pose.position.x = goal[0]
    pose.pose.position.y = goal[1]
    pub.publish(pose)
    
    pub_points.publish(Markers)
    print (goal)


odomerty_name=''
if sim:
    odomerty_name = "p3d_odom"
else:
    odomerty_name = "localization_gps_imu/odometry"

sub = rospy.Subscriber("localization_gps_imu/odometry",Odometry,cb_odom,queue_size=1)

rospy.spin()
