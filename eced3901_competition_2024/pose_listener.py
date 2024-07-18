#! /usr/bin/env python3

# Script to subscribe to both competition robot's pose topics
# Written by Jasper Grant

import rclpy
from rclpy.node import Node

import socket

from geometry_msgs.msg import Pose

udp_ip = "10.0.0.2"
udp_port = 9090

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
		# Send UDP packet to main system in json format
		# Format:
		# {
		# 	"team": 1,
		# 	"x": msg.position.x,
		# 	"y": msg.position.y,
		# 	"theta": msg.orientation.z,
		# }
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

		data = {
			"team": 1,
			"x": msg.position.x,
			"y": msg.position.y,
			"theta": msg.orientation.z,
		}
		sock.sendto(str(data).encode(), (udp_ip, udp_port))
		
		
	# Callback for team 2's pose updates
	def team_2_callback(self, msg):
		# This will be flipped in the actual competition but is taken as is now.
		self.get_logger().info(f"Team 2 position: {msg.position.x},{msg.position.y},{msg.position.z}, orientation: {msg.orientation.x},{msg.orientation.y},{msg.orientation.z},{msg.orientation.w}\n")
		# Send UDP packet to main system in json format
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

		data = {
			"team": 2,
			"x": 193.5 - msg.position.x,
			"y": 145.0 - msg.position.y,
			"theta": msg.orientation.z + 3.14159,
		}
		sock.sendto(str(data).encode(), (udp_ip, udp_port))

		
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
