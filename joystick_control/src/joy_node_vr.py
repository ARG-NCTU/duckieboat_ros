#!/usr/bin/env python
import rospy
import math

from sensor_msgs.msg import Joy
from duckieboat_msgs.msg import MotorCmd,Heading
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        # Publications
        self.pub_motor_cmd = rospy.Publisher("motor_cmd", MotorCmd, queue_size=1)

        # Subscriptions
        self.sub_cmd_drive = rospy.Subscriber("cmd_drive",MotorCmd,self.cbCmd,queue_size=1)
        self.sub_joy = rospy.Subscriber("/boat/vr_joy", Float32MultiArray, self.cbJoy, queue_size=1)
        #self.sub_right = rospy.Subscriber("pub2ros/thrusters/right_thrust_cmd",Float64,self.rcmd,queue_size=1)
        #self.sub_left = rospy.Subscriber("pub2ros/thrusters/left_thrust_cmd",Float64,self.lcmd,queue_size=1)
        #varibles
        self.emergencyStop = False
        self.autoMode = False
        self.motor_msg = MotorCmd()
        self.motor_msg.right = 0
        self.motor_msg.left = 0

        #timer
        self.timer = rospy.Timer(rospy.Duration(0.2),self.cb_publish)
        self.count = 0

    def cb_publish(self,event):
        if self.emergencyStop:
            self.motor_msg.right = 0
            self.motor_msg.left = 0

	self.motor_msg.right = 0.85*self.motor_msg.right
	self.motor_msg.left = 0.85*self.motor_msg.left        
        self.pub_motor_cmd.publish(self.motor_msg)

    #def rcmd(self, rcmd_msg):
        #if not self.emergencyStop and self.autoMode:
            #self.motor_msg.right = max(min((rcmd_msg/100),1),-1)

    #def lcmd(self, lcmd_msg):
        #if not self.emergencyStop and self.autoMode:
            #self.motor_msg.left = max(min((lcmd_msg/100),1),-1)   
            

    def cbCmd(self, cmd_msg):
        if not self.emergencyStop and self.autoMode:
            self.motor_msg.right = max(min(cmd_msg.angular.z,1),-1)
            self.motor_msg.left = - self.motor_msg.right

    def cbJoy(self, joy_msg):
        # mode button TBD
        #self.processButtons(joy_msg)
        if not self.emergencyStop and not self.autoMode:
            self.joy = joy_msg
            boat_heading_msg = Heading()

            speed = self.joy.data[1]

            self.count = self.count + 1
            if self.count % 20 == 0 :
                print("small motor : " + str(speed))
                self.count = 0

            self.motor_msg.right = max(min(speed, 1),-1)
            self.motor_msg.left = -self.motor_msg.right

    # buttons TBD
    def processButtons(self, joy_msg):
        # Button A
        if (joy_msg.buttons[0] == 1):
            rospy.loginfo('A button')
            
        # Y button
        elif (joy_msg.buttons[3] == 1):
            rospy.loginfo('Y button')

        # Left back button
        elif (joy_msg.buttons[4] == 1):
            rospy.loginfo('left back button')

        # Right back button
        elif (joy_msg.buttons[5] == 1):
            rospy.loginfo('right back button')

        # Back button
        elif (joy_msg.buttons[6] == 1):
            rospy.loginfo('back button')
            
        # Start button
        elif (joy_msg.buttons[7] == 1):
            self.autoMode = not self.autoMode
            if self.autoMode:
                rospy.loginfo('going auto')
            else:
                rospy.loginfo('going manual')

        # Power/middle button
        elif (joy_msg.buttons[8] == 1):
            self.emergencyStop = not self.emergencyStop
            if self.emergencyStop:
                rospy.loginfo('emergency stop activate')
                self.motor_msg.right = 0
                self.motor_msg.left = 0
            else:
                rospy.loginfo('emergency stop release')
        # Left joystick button
        elif (joy_msg.buttons[9] == 1):
            rospy.loginfo('left joystick button')

        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                rospy.loginfo('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))

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
