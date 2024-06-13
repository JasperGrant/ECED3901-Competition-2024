# Launch file to demonstrate namespaces and remapping
# Written by Jasper Grant

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='eced3901_competition_2024',
			namespace='team_11_namespace',
			executable='test_student_with_switch',
			name='team_11_robot',
			remappings=[
				('/team_11_namespace/team_1_pose', '/team_1_pose'),
				('/team_11_namespace/team_2_pose', '/team_2_pose'),
				('/team_11_namespace/CompetitionStart', '/CompetitionStart'),
				('/team_11_namespace/team_1_ready', '/team_1_ready'),
				('/team_11_namespace/team_2_ready', '/team_2_ready')
				
			],
			output='screen'
		),
		
		Node(
			package='dalmotor',
			namespace='team_11_namespace',
			executable='dalmotor',
			name='team_11_motor',
			output='screen'
		)
	])

