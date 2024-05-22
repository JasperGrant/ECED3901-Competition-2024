# ECED3901 Competition 2024
A ROS2 package to be used to connect both teams to the UI in the ECED3901 2024 competition

This package contains three python files:

 - pose_listener: This node should subscribe to your team's robot pose as '/team_1_pose' or '/team_2_pose' and print to console whenever it receives a message.
 
 - competition_publisher: Once the competition is started this node will continuously publish empty messages to '/CompetitionStart'
 
 - test_student: This node demonstrates what your robot should be doing to interact with these nodes.
 
 The final competition will require each team to interface with both of these pose_listener and competition_publisher nodes to ensure simultaneous competition start and constantly monitor the pose of both robots.
