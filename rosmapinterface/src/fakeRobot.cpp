#include <QtConcurrent/QtConcurrent>

#include <ros/ros.h>

#include "rosmapinterface/RobotInformation.h"
#include "rosmapinterface/WaypointInformation.h"

namespace
{
    const int robotID = 1337;
}

void spamBeacon(ros::NodeHandle &handle)
{
    const bool global = false; //change to false to have robot placed on local map
    const float globalFakeLat = 21; //degrees
    const float globalFakeLon = 52;
    const float localFakeX = 0; //meters
    const float localFakeY = 0;
    const float orientation = 0;

    ros::Publisher pub = handle.advertise<rosmapinterface::RobotInformation>("/robot_introduction", 5);

    qDebug("Sending beacons");
    while (ros::ok())
    {
        rosmapinterface::RobotInformation beacon;
        beacon.robot_id = robotID;
        beacon.robot_name = "fake";
        beacon.robot_type = "robot";
        beacon.description = "not a real robot";
        beacon.x = global ? globalFakeLat : localFakeX;
        beacon.y = global ? globalFakeLon : localFakeY;
        beacon.theta = orientation;
        beacon.localization_type = global ? (uint8_t)rosmapinterface::RobotInformation::TYPE_GLOBAL
                                          : (uint8_t)rosmapinterface::RobotInformation::TYPE_LOCAL_ABSOLUTE;

        pub.publish(beacon);
        sleep(1); //normal robot may want to spam beacon more often (like 2/sec)
    }
}

void waypointCallback(const rosmapinterface::WaypointInformation::ConstPtr &msg)
{
    qDebug("Waypoint received: %f %f", msg->x, msg->y);
}

void listenToWaypoints(ros::NodeHandle &handle)
{
    QString topic = "/robot/" + QString::number(robotID) + "/waypoint_information";
    qDebug("Listening to waypoints on topic %s", qPrintable(topic));
    ros::Subscriber sub = handle.subscribe<rosmapinterface::WaypointInformation>(
                topic.toStdString(), 100, waypointCallback);
    ros::spin();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rosmapinterface_fakerobot");
    while (!ros::master::check())
    {
        qDebug("Waiting for ros master");
        sleep(1);
    }

    ros::NodeHandle handle;
    QtConcurrent::run(spamBeacon, handle);
    listenToWaypoints(handle);
    return 0;
}
