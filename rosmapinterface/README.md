#rosmapinterface

The map interface as a ros node. The project is a very early prototype but allows for communication between the map and a ROS robot:
- The robot that sends RobotInformation.msg will be shown on the map. To show particular icon for the robot (instead of default one),
 deploy the icon to the "customicons" folder of the map installation (currently it's /share/maps/customicons). The icon name should be the same
 as the robot_name field in the message.
- From the map interace, you can choose the robot (clicking it on a list) and then create a waypoint and order it to go there. The fake robot will
 just print the message confirming that it received the message. 

To build the project, copy the entire directory to your ROS catkin workspace (i.e. ~/jade_ws/src/) and build as any other ros node (run catkin_make).
The binaries include the map interface node and a fake robot.
