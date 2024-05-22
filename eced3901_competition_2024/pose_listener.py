#! /usr/bin/env python3

# Script to subscribe to both competition robot's pose topics
# Written by Jasper Grant

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

class PoseListener(Node):

	def __init__(self):
		super().__init__('pose_listener')
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
		
	def team_1_callback(self, msg):
		print(f"Team 1 position: {msg.position.x},{msg.position.y},{msg.position.z}, orientation: {msg.orientation.x},{msg.orientation.y},{msg.orientation.z},{msg.orientation.w}\n")
		
	def team_2_callback(self, msg):
		print(f"Team 2 position: {msg.position.x},{msg.position.y},{msg.position.z}, orientation: {msg.orientation.x},{msg.orientation.y},{msg.orientation.z},{msg.orientation.w}\n")
		
def main():
	rclpy.init()
	
	pose_listener = PoseListener()
	
	rclpy.spin(pose_listener)
	
	pose_listener.destroy_node()
	rclpy.shutdown()
		
if __name__ == '__main__':
	main()
