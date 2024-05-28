# ECED3901 Competition 2024
A ROS2 package to be used to connect both teams to the UI in the ECED3901 2024 competition

## Description

This package contains three python files:

 - **pose_listener:** This node should subscribe to your team's robot pose as '/team_1_pose' or '/team_2_pose' and print to console whenever it receives a message.
 
 - **competition_publisher:** Once the competition is started this node will continuously publish empty messages to '/CompetitionStart'
 
 - **test_student:** This node demonstrates what your robot should be doing to interact with these nodes.
 
 The final competition will require each team to interface with both of these pose_listener and competition_publisher nodes to ensure simultaneous competition start and constantly monitor the pose of both robots.


## Namespaces

To avoid topic collision, each team must have all of their topics outside of /CompetitionStart and /team_1_pose or /team_2_pose inside their own namespace. The naming convention is not strict as long as it has your team number in it. This is left intentionally vague so that a team cannot just publish malicious data to every other team number's namespaces.

An example of how to set a namespace for your nodes is shown below:

```bash

ros2 run my_package my_executable --ros-args --remap --namespace=/team_1_cool_namespace
```

The problem with this remapping is that you will remap your CompetitionStart and pose topics to the /team_1_cool_namespace namespace. To avoid this you should have 1-2 nodes outside of the namespace that can communicate between the competition nodes and your robot nodes. These outside nodes can still communicate with nodes within the namespace by simply adding the namespace to the topic name.

```python
publisher = node.create_publisher(Pose, '/team_1_cool_namespace/secret_pose', 10)
```


## Pose Convention

To properly show the position of both robots on the UI a pose convention must be set. This convention is individual to each team so the opposing team will see the field with your (193,145) as their (0,0) and your 0 radians as their PI radians.

This is to say that each team will operate with their corner of origin as (0,0). We will perform the calculation to correct this in the UI, and you will only have to worry about the other team's orientation being flipped if you intend to use their pose data. from /team_1/2_pose in your operation.

The x-axis will be the long side of the field and the y-axis will be the short side of the field. All coordinates are in inches.

While the orientation member of the pose message is typically a quaternion, we will take the /team_x_pose/orientation/z member to mean the robot's orientation about the z axis in radians. Here the X-axis represents the origin of 0 degrees and the angle in radians is measured in the counter-clockwise direction. This should be wrapped between -PI and PI. Exactly like the unit circle. Keep in mind this will also be flipped between teams but you only need to worry about your own team's orientation.


![image](img/map.png)



## Testing

### Communication with Competition Nodes

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

### Correctly using Namespaces

To ensure no namespace collisions you should run the communication test above with the namespace remapping. If everything is set up correctly you should see with rostopic list that the the only topics that are not in the namespace are the /CompetitionStart and /team_1_pose or /team_2_pose topics.



