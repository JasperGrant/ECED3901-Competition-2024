#! /usr/bin/env python3

# Script to demonstrate how a single robot team should behave in competition
# Written by Jasper Grant

import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose

# Class to act as a team's robot
class TestStudent(Node):

	# Constructor using number entered in console
	def __init__(self, team_number):
		super().__init__(f"test_student_{team_number}")
		self.publisher_ = self.create_publisher(Pose, f"team_{team_number}_pose", 10)
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
		
	# Receive empty start message and flip on switch
	def start_callback(self, msg):
		self.start = True
		
	 
		
def main():

	# Handle user input of arguments
	if len(sys.argv) != 2:
		print("Do not forget to give a team number argument, 1 or 2!\n")
		return -1
	try:
		team_number = int(sys.argv[1])
	except: 
		print("Ensure your team number is an integer!\n")
		return -2
	if int(team_number) not in [1,2]:
		print("Team number must be either 1 or 2!\n")
		return -3

			
	# Init ros2
	rclpy.init()
	
	# Create class instance
	test_student = TestStudent(team_number)
	
	# Spin continously
	rclpy.spin(test_student)
	
	# Gracefully kill
	test_student.destroy_node()
	rclpy.shutdown()
		
if __name__ == '__main__':
	main()
