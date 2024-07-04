#! /usr/bin/env python3

# Script to demonstrate how a single robot team should behave in competition, 
# This version includes a physical switch to tell the difference between team 1 and team 2
# Written by Jasper Grant

import sys

import serial
import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Int32
from geometry_msgs.msg import Pose, Twist

example_group_number = 0

# Class to act as a team's robot
class TestStudent(Node):

	# Constructor using number entered in console
	def __init__(self, team_number):
		super().__init__(f"test_student_{team_number}")
		self.publisher_ = self.create_publisher(Pose, f"team_{team_number}_pose", 10) # fstring lets you format a string in its statement
		self.other_publishers = self.create_publisher(Empty, "other_robot_stuff", 10)
		self.ready_publisher = self.create_publisher(Int32, f"team_{team_number}_ready", 10)
		self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
		timer_period = 0.5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.competition_start_subscription = self.create_subscription(
			Empty,
			'CompetitionStart',
			self.start_callback,
			10
		)
		self.start = False
	
	# Publish pose (you can do this at any rate you want)
	def timer_callback(self):
	
		ready_msg = Int32()
		ready_msg.data = example_group_number
		self.ready_publisher.publish(ready_msg)
	
		# If start has not been received
		if  not self.start:
			return
	
		# Create pose message
		msg = Pose()
		
		# Populate with unique data
		msg.position.x = 0.0
		msg.position.x = 0.1
		msg.position.x = 0.2
		
		msg.orientation.x = 0.3
		msg.orientation.y = 0.4
		msg.orientation.z = 0.5
		msg.orientation.w = 0.6
		
		# Publish pose message
		self.publisher_.publish(msg)
		
		vel_msg = Twist()
		vel_msg.linear.x = 1.0
		self.vel_pub.publish(vel_msg)
		
	# Receive empty start message and flip on switch
	def start_callback(self, msg):
		self.start = True
		
	 
		
def main():
	# No arguments handled any more
	
	# Initiate serial
	ser = serial.Serial(
			port='/dev/ttyUSB0', # USB number could change depending on what port the USB is plugged into. DOUBLE CHECK THIS
			baudrate=9600)
			
	team_number = chr(ser.read()[-1])
	
	# Swap example group number with your group number (and remove the IF statement) when ready
	global example_group_number
	example_group_number = 11 if team_number == '1' else 15 
			
	# Init ros2
	rclpy.init()
	
	# Create class instance; stuff in init of the class runs. 
	test_student = TestStudent(team_number)
	
	# Spin continously
	rclpy.spin(test_student)
	
	# Gracefully kill
	test_student.destroy_node()
	rclpy.shutdown()
		
if __name__ == '__main__':
	main()
