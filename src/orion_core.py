#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float32,Int32,Int16
from geometry_msgs.msg import Twist

import math


class Robot():

    def __init__(self):
       
        
        self.wheel_radius = 0.0321 #m
        self.wheelbase = 0.1175 #m
      
       
        
        self.leftwheel_power_pub = rospy.Publisher('wheel_power_left',Float32, queue_size = 10)
        self.rightwheel_power_pub = rospy.Publisher('wheel_power_right',Float32, queue_size =10)
        
        self.vel_sub = rospy.Subscriber('/orion/cmd_vel', Twist, self.vel_callback,queue_size = 10)
        self.v = 0
        self.w = 0
  
        self.min_speed = 6.23 #rad/s
        self.max_speed = 9.345
        
        self.min_speed_power = 0.75
        self.max_speed_power = 1.0
        
        self._wheel_vel = {"r":0.0,"l":0.0}
        self.power_left = Float32()
        self.power_right = Float32()
        self.power_left.data = 0.0
        self.power_right.data = 0.0
        
        rospy.loginfo("start orion node")
        
    
        
    def vel_callback(self,vel):
    	self.v = vel.linear.x
    	self.w = vel.angular.z
    	L = self.wheelbase
    	R = self.wheel_radius
    	
    	#convert unicycle to diff drive
    	v_l = ((2.0*self.v)+(self.w*L))/(2.0*R)
    	v_r = ((2.0*self.v)-(self.w*L))/(2.0*R)
    	
    	self.wheel_speed_setpoint(v_r,v_l)
    	
    
    def convert_vel_to_power(self,v):
    	v_mag = abs(v)
    	
    	if v_mag < self.min_speed:
    		return 0.0
    	
    	vel_factor = (v_mag-self.min_speed)/(self.max_speed-self.min_speed)
    	power = (vel_factor*(self.max_speed_power-self.min_speed_power))+self.min_speed_power
    	
    	if v<0:
    		power *=-1.0
    	return power
    
    def wheel_speed_setpoint(self,vr,vl):
    	 vr = max(min(vr,self.max_speed),self.max_speed*-1)
    	 vl = max(min(vl,self.max_speed),self.max_speed*-1)
    	 if vr == 0.0 and vl == 0.0:
    	 	for index in ["r","l"]:
    	 		self._wheel_vel[index] = 0.0
    	 	self.power_left.data = 0.0
    	 	self.power_right.data = 0.0
    	 	
    	 self.power_left.data = self.convert_vel_to_power(vr)
    	 self.power_right.data = self.convert_vel_to_power(vl)
    	 
    	 self.leftwheel_power_pub.publish(self.power_left.data)
    	 self.rightwheel_power_pub.publish(self.power_right.data) 
    	 

def main(args=None):
    rospy.init_node('orion_core', anonymous=True)
    robot = Robot()
    rospy.spin()


if __name__ == '__main__':
    main()
