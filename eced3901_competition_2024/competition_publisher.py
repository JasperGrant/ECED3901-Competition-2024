#! /usr/bin/env python3

# Script to publish continuously to the CompetitionStart topic
# Written by Jasper Grant

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty

# Class which represents a publisher which never stops publishing empty msgs on CompetitionStart
class CompetitionPublisher(Node):

	# Constructor
	def __init__(self):
		super().__init__('competition_publisher')
		self.publisher_ = self.create_publisher(Empty, 'CompetitionStart', 10)
		# Set interval for publishing
		timer_period = 0.5
		self.timer = self.create_timer(timer_period, self.timer_callback)
	
	# Function called on every timer interval
	def timer_callback(self):
		# Publish empty msg
		msg = Empty()
		self.publisher_.publish(msg)
		
def main():
	# Init ros2
	rclpy.init()
	
	# Create class instance
	competition_publisher = CompetitionPublisher()
	
	# Spin continously
	rclpy.spin(competition_publisher)
	
	# Gracefully kill
	competition_publisher.destroy_node()
	rclpy.shutdown()
		
if __name__ == '__main__':
	main()
