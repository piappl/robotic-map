Robotic map interface, based on Marble. Author: Adam Dąbrowski, adabrowski@piap.pl. License: LGPLv3.

The map project creates the library. You can use it with your application through Qt slots/signals interface. Another option is to build and run rosmapinterface - then the map functions as a ros node. 

To install, run ./make-map.sh with a path to your chosen Qt version cmake.
Your customized content should be placed in /share/maps/ directory:
- /share/maps/localmap/ holds .cfg and .png file that is local map (i.e. scanned by robot before)
- /share/maps/customicons/ hold .png files that are robot icons to display on map. The name of the file should be the same as the robot name sent by the robot in RobotInformation ROS message.
(Directories are subject of change in future versions - moving content to more standard locations)

