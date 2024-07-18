# ECED3901 Competition 2024
A ROS2 package to be used to connect both teams to the UI in the ECED3901 2024 competition

![image](img/competition_diagram.png)

# Running the Competition

1. Place both robots on the course and ensure they are powered on. They will each be on a ROS_DOMAIN_ID of their team number. Ideally systemD services will be used to start the robot nodes on boot.
2. Echo ready topics on either domain
```bash
ROS_DOMAIN_ID=GROUP_NUMBER_TEAM_1 ros2 topic echo /team_1_ready
```
```bash
ROS_DOMAIN_ID=GROUP_NUMBER_TEAM_2 ros2 topic echo /team_2_ready
```
Make sure that GROUP_NUMBER_TEAM_1 and GROUP_NUMBER_TEAM_2 are replaced with the actual group numbers of the teams. You should see the team numbers being published at 2Hz.
3. Start the pose listeners on both domains
```bash
ROS_DOMAIN_ID=GROUP_NUMBER_TEAM_1 ros2 run eced3901_competition_2024 pose_listener
```
```bash
ROS_DOMAIN_ID=GROUP_NUMBER_TEAM_2 ros2 run eced3901_competition_2024 pose_listener
```
4. Start an instance of the competition publisher on the overseert computer on each domain with a single command
```bash
ROS_DOMAIN_ID=GROUP_NUMBER_TEAM_1 ros2 run eced3901_competition_2024 competition_publisher && ROS_DOMAIN_ID=GROUP_NUMBER_TEAM_2 ros2 run eced3901_competition_2024 competition_publisher
```
5. When both robots have finished the competition close all terminals and repeat the process for the next round.

## A Note on Namespaces


Due to difficulties getting nav2, rviz, dalmotor, and ylidar mapped to namespaces, the competition integration setup has been changed to use ROS_DOMAIN_ID to separate the two teams. This means that each team will have their own ROS_DOMAIN_ID and will not be able to see the other team's topics and nodes. This unfortunately removes the possibility of spying on another teams's pose topic but means all your topics can be in the global namespace. Apart from the loss of namespaces no changes must be made to the competition setup. Each team will still publish and subscribe to the global topics in the same way. To make this work the third competition overseer computer will run an instance of the competition nodes for each team. This will allow the competition nodes to see both teams' topics and nodes. For the sake of previously done work the team 1/2 switch and topic requirements are still in place.
 

## Competition Integration Overview

Each team will be required to conform to the following standards to interface with the competition nodes.

1. A physical switch on the robot must be used to switch between team 1 and team 2.
2. Either the same or a second switch must be used to switch between the "green" 80Hz team 1 and the "blue" 40Hz team 2.
3. The robot must publish it's team number, n, in the form of an std_msgs/Int32 at 2Hz to the topic '/team_x_ready' where x is team 1 or team 2. n is your design team's number. You will publish to it when your robot is ready to begin the competition. 
4. The robot must continuously publish it's pose in the form of a geometry_msgs/Pose at 2Hz to the topic '/team_x_pose' where x is team 1 or team 2.
5. The robot must subscribe to the topic '/CompetitionStart' and start the competition when it receives an empty message on this topic. This topic will continue to be published with empty messages at 2Hz.
**Changed:**
6. ROS_DOMAIN_ID must be set to your design team number. This will allow your topics and nodes to be seen by the competition nodes.
7. Your SystemD service must be set to run on your team's ROS_DOMAIN_ID. An example of syntax for this can be found in services/example.service.

## Description

This package contains the following code:

 - **pose_listener:** This node should subscribe to your team's robot pose as '/team_1_pose' or '/team_2_pose' and print to console whenever it receives a message.
 
 - **competition_publisher:** Once the competition is started this node will continuously publish empty messages to '/CompetitionStart'
 
 - **test_student:** This node demonstrates what your robot should be doing to interact with the competition nodes.
 - **test_student_with_switch:** This node demonstrates how to decided team 1 or 2 based on a switch input from an Arduino. This is a more advanced version of test_student that you can use to test your robot with a switch input. This can also be done in C++ but I find serial is easier in python.
 - **switch_sketch.ino:** This very simple Arduino sketch sends a 1 over serial if pin 12 is shorted to GND and a 2 if pin 12 is floating. This is used in test_student_with_switch.py to determine which team the robot is on.
 
## Installation

To use this package, clone the repository into your ROS2 workspace and colcon build the package. The package should be on the same directory level as your eced3901 package.

```bash
cd /ros2_ws/src
git clone https://github.com/JasperGrant/ECED3901-Competition-2024
cd ..
colcon build
```

## Systemd Service

Systemd allows you to run your launch file as a service. This is useful for running your robot code on boot. Example service files are included in the services directory.

Once your service file is created you must:

1. Copy the service file to /etc/systemd/system

```bash
sudo cp /path/to/your/service/file /etc/systemd/system/your_service_name.service
```

2. Reload the systemd daemon

```bash	
sudo systemctl daemon-reload
```

3. Enable the service to run on boot

```bash
sudo systemctl enable your_service_name.service
```

4. Start the service to test

```bash
sudo systemctl start your_service_name.service
```
service ROS2 nodes must run on ROS_DOMAIN_ID=design team number.

The next time you boot your robot it will run the enabled service.

To troublshoot run:

```bash
journalctl -f
```

This displays all output from services.

The following link is the source of my information on systemd services: https://wiki.arcoslab.org/tutorials/starting_ros2_nodes_with_systemd

## Pose Convention

To properly show the position of both robots on the UI a pose convention must be set. This convention is individual to each team so the opposing team will see the field with your (193,145) as their (0,0) and your 0 radians as their PI radians.

This is to say that each team will operate with their corner of origin as (0,0). We will perform the calculation to correct this in the UI, and you will only have to worry about the other team's orientation being flipped if you intend to use their pose data.

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

1. Start your robot node.

```bash
ros2 run eced3901_competition_2024 test_student 1
```

**OR** 

```bash
ros2 run eced3901_competition_2024 test_student 2
```

The robot node should publish it's team number to the topic '/team_1_ready' or '/team_2_ready'.

You should still not see any output from the pose_listener node.

1. Start the competition_publisher node by running the following command:

```bash
ros2 run eced3901_competition_2024 competition_publisher
```

You should see the pose of your robot being printed to the console by the pose_listener node.

You should now see the robot publishing it's pose to the topic '/team_1_pose' or '/team_2_pose' and it's team number to the topic '/team_1_ready' or '/team_2_ready'.
The competition_publisher node should be publishing empty messages to the topic '/CompetitionStart'.

An example below shows both test_student robots running at the same time:

![image](img/test_example.png)

## FAQ

**Q: I am seeing topics and nodes that are not mine?**
Ensure that your ROS_DOMAIN_ID is set to your team number. This will prevent you from seeing other team's topics and nodes.

**Q: I am changing my code but the changes are not showing up?**
A: Delete the build and install folders in /ros2_ws and rebuild the package. Either your executables will be built from the newest version or they will fail to build and you can see the error messages.

**Q: How am I going to interface with the Arduino?**
A: Example files are provided in the competition repo; basic arduino code can be found in the arduino subdirectory, and a python script that deals with the switch input coming from the arduino is detailed in test_student_with_switch.py from the eced3901_competition_2024 directory. 

**Q: How can I identify which COM Port the Arduino is connected to?**
A: Open a new terminal. Type "cd /dev", then "ls". A large list of files should appear. You should look for a file named ttyUSBX or ttyACMX, where X is a number associated with the port for serial connection that your Arduino is attached using. 
