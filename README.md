# ECED3901 Competition 2024
A ROS2 package to be used to connect both teams to the UI in the ECED3901 2024 competition

## Description

This package contains three python files:

 - **pose_listener:** This node should subscribe to your team's robot pose as '/team_1_pose' or '/team_2_pose' and print to console whenever it receives a message.
 
 - **competition_publisher:** Once the competition is started this node will continuously publish empty messages to '/CompetitionStart'
 
 - **test_student:** This node demonstrates what your robot should be doing to interact with these nodes.
 
 The final competition will require each team to interface with both of these pose_listener and competition_publisher nodes to ensure simultaneous competition start and constantly monitor the pose of both robots.

 ## Testing

Follow the instructions below to test the communication between your robot and the competition nodes.

1. Start the pose_listener node by running the following command:

```bash
ros2 run eced3901_competition_2024 pose_listener
```

You should not see any output.

2. Start your robot node. The robot node should publish its pose to the topic '/team_1_pose' or '/team_2_pose' depending on your team number.

The test_student node can be used to simulate the robot node. To start the test_student node, run the following command:

```bash
ros2 run eced3901_competition_2024 test_student 1
```

**OR** 

```bash
ros2 run eced3901_competition_2024 test_student 2
```

You should still not see any output from the pose_listener node.

3. Start the competition_publisher node by running the following command:

```bash
ros2 run eced3901_competition_2024 competition_publisher
```

You should see the pose of your robot being printed to the console by the pose_listener node.

**NOTE:** Make sure that your robot can easily be changed from team 1 to team 2 by changing the topic it publishes to. This will be required for the final competition. A good way to do this is as an argument for the program or launch file of 1 or 2.



