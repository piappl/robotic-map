Robotic map interface, based on Marble. Author: Adam Dąbrowski, adabrowski@piap.pl. License: LGPLv3.

The map project creates the library. You can use it with your application through Qt slots/signals interface. Another option is to build and run rosmapinterface - then the map functions as a ros node. 

Prerequisites for Marble:
Qt5+
Modules:
qt5-default qtbase5-dev qt5-default qtdeclarative5-dev libqt5webkit5-dev qttools5-dev libqt5designercomponents5 libqt5designer5 qtdeclarative5-dev qtdeclarative5-dev-tools qtquick1-5-dev qtscript5-dev qttools5-dev-tools  qtbase5-dev-tools qtmultimedia5-dev libqt5svg5-dev
(Webkit will be removed, it's not really necessary)

Marble can install in a subdirectory of /usr/lib (i.e. /usr/lib/x86_64-linux-gnu).
Make sure paths to libraries of marble and mapwidget (libmarble-qt5, libmapwidget) are visible.

To install, run ./make-map.sh with a path to your chosen Qt version cmake.

Your customized content should be placed in /share/maps/ directory:
- /share/maps/localmap/ holds .cfg and .png file that is local map (i.e. scanned by robot before)
- /share/maps/customicons/ hold .png files that are robot icons to display on map. The name of the file should be the same as the robot name sent by the robot in RobotInformation ROS message.
(Directories are subject of change in future versions - moving content to more standard locations)

