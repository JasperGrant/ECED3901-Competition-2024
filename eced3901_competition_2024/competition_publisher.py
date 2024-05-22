#! /usr/bin/env python3

# Script to publish continuosly to the CompetitionStart topic
# Written by Jasper Grant

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty

class CompetitionPublisher(Node):

	def __init__(self):
		super().__init__('competition_publisher')
		self.publisher_ = self.create_publisher(Empty, 'CompetitionStart', 10)
		timer_period = 0.5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		
	def timer_callback(self):
		msg = Empty()
		self.publisher_.publish(msg)
		
def main():
	rclpy.init()
	
	competition_publisher = CompetitionPublisher()
	
	rclpy.spin(competition_publisher)
	
	competition_publisher.destroy_node()
	rclpy.shutdown()
		
if __name__ == '__main__':
	main()
