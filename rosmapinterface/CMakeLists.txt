cmake_minimum_required(VERSION 2.8.11)
project(rosmapinterface)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

set(CMAKE_PREFIX_PATH ~/Qt5.3.1/5.3/gcc_64/lib/cmake)
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)

message(WARNING, "1. |${catkin_INCLUDE_DIRS}|${CATKIN_DEVEL_PREFIX}|${CMAKE_INSTALL_PREFIX}|")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_message_files(FILES RobotInformation.msg WaypointInformation.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(Resources src/res.qrc)
add_executable(rosmapinterface
            src/main.cpp
            ${Resources})
target_link_libraries(rosmapinterface ${catkin_LIBRARIES} Qt5::Widgets Qt5::Quick Qt5::Core mapwidget)
add_dependencies(rosmapinterface ${catkin_EXPORTED_TARGETS})

#fakeRobot
add_executable(fakeRobot src/fakeRobot.cpp)
target_link_libraries(fakeRobot Qt5::Core ${catkin_LIBRARIES})

