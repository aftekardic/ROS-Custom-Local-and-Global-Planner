#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopJoy():
	def __init__(self):
		rospy.init_node("teleop_husky")
		rospy.Subscriber("/joy", Joy, self.joy_cb) #Subscribe to joy topic to get joystick data
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10) #Publish velocity data to /cmd_vel topic

		self.twist = Twist() #Create empty twist

		self.angular_axis = 0 #Zero is left analog stick's right and left axes
		self.linear_axis = 1 #One is left analog stick's up and down axes
		self.rb = 5 #R1 button -> turbo mode
		self.lb = 4 #L1 button -> normal mode
		self.turbo_multiplier = 2 #Turbo mode velocity multiplier

		self.last_angular = 0.0 #Keep last angular and linear velocities to publish if no change in axes
		self.last_linear = 0.0
		self.rate = rospy.Rate(25) #Publish 25 data every second

		self.joy_publisher()

	def joy_cb(self, data):
		if data.buttons[self.lb]:
			self.twist.linear.x = data.axes[self.linear_axis] / 2
			self.twist.angular.z = data.axes[self.angular_axis] / 2
			# print('x: {}'.format(self.twist.linear.x))
			# print('z: {}'.format(self.twist.angular.z))
		
		elif data.buttons[self.rb]:
			self.twist.linear.x = data.axes[self.linear_axis] * self.turbo_multiplier / 2
			self.twist.angular.z = data.axes[self.angular_axis] * self.turbo_multiplier / 2
			# print('x: {}'.format(self.twist.linear.x))
			# print('z: {}'.format(self.twist.angular.z))	
			
		else: #If RB or LB is not pressed, stop vehicle
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0

	def joy_publisher(self):
		while not rospy.is_shutdown():
			self.pub.publish(self.twist)
			self.rate.sleep()

if __name__ == "__main__":
	
	teleop = TeleopJoy()
	rospy.spin()
