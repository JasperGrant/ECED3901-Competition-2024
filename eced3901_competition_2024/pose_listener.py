#! /usr/bin/env python3

# Script to subscribe to both competition robot's pose topics
# Written by Jasper Grant

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

# Class to print to console every pose update
class PoseListener(Node):
	
	# Constuctor
	def __init__(self):
		super().__init__('pose_listener')
		# Set up a subscriber for each team
		self.team_1_subscription = self.create_subscription(
			Pose,
			'team_1_pose',
			self.team_1_callback,
			10
		)
		self.team_2_subscription = self.create_subscription(
			Pose,
			'team_2_pose',
			self.team_2_callback,
			10
		)
		
	# Callback for team 1's pose updates
	def team_1_callback(self, msg):
		self.get_logger().info(f"Team 1 position: {msg.position.x},{msg.position.y},{msg.position.z}, orientation: {msg.orientation.x},{msg.orientation.y},{msg.orientation.z},{msg.orientation.w}\n")
		
	# Callback for team 2's pose updates
	def team_2_callback(self, msg):
		# This will be flipped in the actual competition but is taken as is now.
		self.get_logger().info(f"Team 2 position: {msg.position.x},{msg.position.y},{msg.position.z}, orientation: {msg.orientation.x},{msg.orientation.y},{msg.orientation.z},{msg.orientation.w}\n")

		
def main():
	# Init ros2
	rclpy.init()
	
	# Create class instance
	pose_listener = PoseListener()
	
	# Spin continuously
	rclpy.spin(pose_listener)
	
	# Gracefully kill
	pose_listener.destroy_node()
	rclpy.shutdown()
		
if __name__ == '__main__':
	main()
