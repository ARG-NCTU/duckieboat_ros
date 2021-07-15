#!/usr/bin/env python
import rospy
import math

from duckieboat_msgs.msg import MotorCmd,Heading
from robotx_msgs.msg import roboteq_drive
from std_msgs.msg import Float64

class WamvMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        # Publications
        self.pub_wamv_cmd_drive = rospy.Publisher("cmd_drive_wamv", roboteq_drive, queue_size=1)

        # Subscriptions
        self.sub_cmd_drive = rospy.Subscriber("cmd_drive",MotorCmd,self.cbCmd,queue_size=1)

        #self.sub_right = rospy.Subscriber("pub2ros/thrusters/right_thrust_cmd",Float64,self.rcmd,queue_size=1)
        #self.sub_left = rospy.Subscriber("pub2ros/thrusters/left_thrust_cmd",Float64,self.lcmd,queue_size=1)
        #varibles
        self.wamv_msg = roboteq_drive()
        self.wamv_msg.right = 0
        self.wamv_msg.left = 0
-

        #timer
        self.timer = rospy.Timer(rospy.Duration(0.2),self.cb_publish)
            
    def cbCmd(self, cmd_msg):
        if not self.emergencyStop and self.autoMode:
            self.wamv_msg.right = max(min(cmd_msg.left,1),-1)
            self.wamv_msg.left = max(min(cmd_msg.right,1),-1)
        
        self.pub_wamv_cmd_drive.publish(wamv_msg)


    def on_shutdown(self):
        self.motor_msg.right = 0
        self.motor_msg.left = 0
        self.pub_motor_cmd.publish(self.motor_msg)
        rospy.loginfo("shutting down [%s]" %(self.node_name))

if __name__ == "__main__":
    rospy.init_node("wamv_mapper",anonymous=False)
    wamv_mapper = WamvMapper()
    rospy.on_shutdown(wamv_mapper.on_shutdown)
    rospy.spin()
