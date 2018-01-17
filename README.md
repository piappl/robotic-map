# robotic-map
map - the robotic map interface library

<b>Introduction:</b>

Robotic map interface, based on Marble. Author: Adam Dąbrowski, adabrowski@piap.pl. License: LGPLv3

The map project creates the library. You can use it with your application through Qt slots/signals interface. It is supported by the robotic-map-ros-interface project, allowing the map functions as a ROS node.

<b> User Manual </b>

You can find User Manual in the docs directory.

<b>Functionalities</b>

- Layered virtual globe interface (thanks to Marble).
- Local maps, manual positioning of robots, merging global and local positioning.
- Displaying objects, robots.
- Connecting to a robot. Issuing navigation commands.
- Simple map editor (no persistence yet). Places, waypoints, areas.
- Simple QML interface.
- A framework for storing data, recording paths.
- Object follow mode.
- Choice of interface: QT slots & signals or (through robotic-map-ros-interface) ROS topics.

<b>Dependencies:</b>

Prerequisites for Marble: Qt5+ Modules: qt5-default qtbase5-dev qt5-default qtdeclarative5-dev libqt5webkit5-dev qttools5-dev libqt5designercomponents5 libqt5designer5 qtdeclarative5-dev qtdeclarative5-dev-tools qtquick1-5-dev qtscript5-dev qttools5-dev-tools qtbase5-dev-tools qtmultimedia5-dev libqt5svg5-dev (Webkit will be removed, it's not really necessary)

Marble can install in a subdirectory of /usr/lib (i.e. /usr/lib/x86_64-linux-gnu). Make sure paths to libraries of Marble and mapwidget (libmarble-qt5, libmapwidget) are visible.

<b>Installation:</b>

To install, run ./make-map.sh with a path to your chosen Qt version cmake.

<b>Customization:</b>

Your customized content should be placed in /share/maps/ directory:

/share/maps/localmap/ holds .cfg and .png file that is local map (i.e. scanned by robot before)
/share/maps/customicons/ hold .png files that are robot icons to display on map. The name of the file should be the same as the robot name sent by the robot in RobotInformation ROS message. (Directories are subject of change in future versions - moving content to more standard locations)

<b> Roadmap </b>

- Developer manual.
- Marble fork (& cooperation).
- Fix positioning approximation closer to poles.
- Data persistence.
- Dynamic local map updates.
- Rework interface to something more intuitive



