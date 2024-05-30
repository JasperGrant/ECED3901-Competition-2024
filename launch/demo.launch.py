# Launch file to demonstrate namespaces and remapping
# Written by Jasper Grant

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
    
		Node(
			package='eced3901_competition_2024',
			executable='competition_publisher',
			name='competition_publisher',
			output='screen'
		),
		
		Node(
			package='eced3901_competition_2024',
			executable='pose_listener',
			name='pose_listener',
			output='screen'
		),
		Node(
			package='eced3901_competition_2024',
			namespace='team_11_namespace',
			executable='test_student',
			name='team_11_robot',
			arguments='1',
			remappings=[
				('/team_11_namespace/team_1_pose', '/team_1_pose'),
				('/team_11_namespace/CompetitionStart', '/CompetitionStart'),
			],
			output='screen'
		),
		Node(
			package='eced3901_competition_2024',
			namespace='namespace_team_15',
			executable='test_student',
			name='team_15_robot',
			arguments='2',
			remappings=[
				('/namespace_team_15/team_2_pose', '/team_2_pose'),
				('/namespace_team_15/CompetitionStart', '/CompetitionStart')
			],
			output='screen'
		)
	])

