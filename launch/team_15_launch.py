# Launch file to demonstrate namespaces and remapping
# Written by Jasper Grant

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='eced3901_competition_2024',
			namespace='namespace_team_15',
			executable='test_student_with_switch',
			name='team_15_robot',
			remappings=[
				('/namespace_team_15/team_2_pose', '/team_2_pose'),
				('/namespace_team_15/team_1_pose', '/team_1_pose'),
				('/namespace_team_15/CompetitionStart', '/CompetitionStart'),
				('/namespace_team_15/team_1_ready', '/team_1_ready'),
				('/namespace_team_15/team_2_ready', '/team_2_ready')
			],
			output='screen'
		),
		Node(
			package='dalmotor',
			namespace='namespace_team_15',
			executable='dalmotor',
			name='team_15_motor',
			output='screen'
		)
	])

