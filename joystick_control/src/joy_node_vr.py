#!/usr/bin/env python
import rospy
import math

from sensor_msgs.msg import Joy
from duckieboat_msgs.msg import MotorCmd,Heading
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        # Publications
        self.pub_motor_cmd = rospy.Publisher("motor_cmd", MotorCmd, queue_size=1)
        self.pub_moos_auto = rospy.Publisher("/boat/change_mode", String, queue_size=1)
        
        # pub for testing
        self.pub_joy_transfer = rospy.Publisher("/boat/vr_joy", Float32MultiArray, queue_size=1)

        # Subscriptions
        self.sub_cmd_drive = rospy.Subscriber("/boat/pub2ros/thrusters/rudder",Float64, self.cbCmd, queue_size=1)
        self.sub_joy = rospy.Subscriber("/boat/vr_joy", Float32MultiArray, self.cbJoy, queue_size=1)
        
        # sub for testing
        self.sub_joy_transfer = rospy.Subscriber("/boat/joy", Joy, self.cbJoy_transfer, queue_size=1)
        
        #varibles
        self.emergencyStop = False
        self.autoMode = False
        self.motor_msg = MotorCmd()
        self.motor_msg.right = 0
        self.motor_msg.left = 0
        self.speed = 0

        #timer
        self.timer = rospy.Timer(rospy.Duration(0.2), self.cb_publish)
        self.count = 0

    def cbJoy_transfer(self, joy_msg):
        self.joy = joy_msg
        boat_heading_msg = Heading()

        arr_msg = Float32MultiArray()
        arr_msg.data = [0]*4

        arr_msg.data[0] = self.joy.axes[1]
        arr_msg.data[1] = self.joy.axes[3]
        arr_msg.data[2] = joy_msg.buttons[7]
        arr_msg.data[3] = joy_msg.buttons[8]

        self.pub_joy_transfer.publish(arr_msg)

            
            

    def cb_publish(self,event):
        if self.emergencyStop:
            self.motor_msg.right = 0
            self.motor_msg.left = 0

        print("small motor : " + str(self.motor_msg.right))

    	self.motor_msg.right = -0.7*self.speed
	self.motor_msg.left = -self.motor_msg.right        
        self.pub_motor_cmd.publish(self.motor_msg)

    def cbCmd(self, cmd_msg):
        if not self.emergencyStop and self.autoMode: 
            rudder = cmd_msg.data/100
            self.motor_msg.right = max(min(rudder,1),-1)
            self.motor_msg.left = -self.motor_msg.right

    def cbJoy(self, joy_msg):
        # mode button TBD

        if (joy_msg.data[2] == 1):
            self.autoMode = not self.autoMode
            if self.autoMode:
                rospy.loginfo('going auto')
                self.pub_moos_auto.publish("STATION-KEEPING")

            else:
                rospy.loginfo('going manual')
                self.pub_moos_auto.publish("LOITERING")

        elif (joy_msg.data[3] == 1):
            self.emergencyStop = not self.emergencyStop
            if self.emergencyStop:
                rospy.loginfo('emergency stop activate')
                self.motor_msg.right = 0
                self.motor_msg.left = 0
            else:
                rospy.loginfo('emergency stop release')

        if self.emergencyStop:
            self.ch1_pwm = 0
            self.ch2_pwm = 0

        if not self.emergencyStop and not self.autoMode:
            self.joy = joy_msg
            boat_heading_msg = Heading()

            self.speed = self.joy.data[1]
            self.speed = max(min(self.speed, 1),-1)
        
    def on_shutdown(self):

        self.motor_msg.right = 0
        self.motor_msg.left = 0
        self.pub_motor_cmd.publish(self.motor_msg)
        rospy.loginfo("shutting down [%s]" %(self.node_name))

if __name__ == "__main__":
    rospy.init_node("joy_mapper",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.on_shutdown(joy_mapper.on_shutdown)
    rospy.spin()
