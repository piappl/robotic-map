#include <QtConcurrent/QtConcurrent>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <people_msgs/People.h>

#include "rosmapinterface/RobotInformation.h"
#include "rosmapinterface/WaypointInformation.h"

namespace
{
    int robotID = 1337;
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
    qDebug("spamBeacon finishing");
}

void spamFakeLaser(ros::NodeHandle &handle)
{
    const float resolution = M_PI/200;
    const float angleMin = -M_PI/2;
    const float angleMax = M_PI/2;

    const int rangeMax = 80; //meters
    const int rangeMin = 1;

    QString topic = "/robot/" + QString::number(robotID) + "/laser_scan";
    ros::Publisher pub = handle.advertise<sensor_msgs::LaserScan>(topic.toStdString(), 5);

    qDebug("Sending laser scan points");
    while (ros::ok())
    {
        sensor_msgs::LaserScan scanspam;
        scanspam.angle_increment = resolution;
        scanspam.angle_min = angleMin;
        scanspam.angle_max = angleMax;
        scanspam.range_max = rangeMax;
        scanspam.range_min = rangeMin;
        int distance = rangeMax;
        for (float i = angleMin; i < angleMax; i = i + resolution)
        {
            //Initialize random if needed, but it's just some data
            bool change = (qrand() % 50) == 0;
            if (!change)
            {
                scanspam.ranges.push_back(distance);
            }
            else
            {
                distance = rangeMin + (qrand() % (rangeMax - rangeMin));
                scanspam.ranges.push_back(distance);
            }
        }

        pub.publish(scanspam);
        sleep(2);
    }
    qDebug("SpamFakeLaser finishing");
}

void spamPeople(ros::NodeHandle &handle)
{
    QString topic = "/robot/" + QString::number(robotID) + "/pedestrians";
    ros::Publisher pub = handle.advertise<people_msgs::People>(topic.toStdString(), 5);

    //qDebug("Sending people");

    const int peopleCount = 4;
    const int metersMax = 10;
    QVector<geometry_msgs::Point> positions;
    QVector<geometry_msgs::Point> velocities;
    for (int i = 0; i < peopleCount; ++i)
    {
        geometry_msgs::Point pos;
        pos.x = qrand() % metersMax;
        pos.y = qrand() % metersMax;
        positions.append(pos);

        geometry_msgs::Point vel;
        vel.x = (float)((qrand() % 100) - 50)/25;
        vel.y = (float)((qrand() % 100) - 50)/25;
        velocities.append(vel);
    }

    int velChange = 0;
    while (ros::ok())
    {
        //qDebug("People");
        if (velChange > 20)
        {
            for (int i = 0; i < peopleCount; ++i)
            {
                geometry_msgs::Point vel;
                vel.x = -velocities[i].x;
                vel.y = -velocities[i].y;
                velocities[i] = vel;
            }
            velChange = 0;
        }
        velChange++;

        people_msgs::People peoplespam;
        for (int i = 0; i < peopleCount; ++i)
        {
            people_msgs::Person person;
            QString name = "John Doe " + QString::number(i);
            person.name = name.toStdString();
            person.position = positions.at(i);
            person.reliability = 1;
            person.velocity = velocities.at(i);

            geometry_msgs::Point pos;
            pos.x = positions.at(i).x + velocities.at(i).x;
            pos.y = positions.at(i).y + velocities.at(i).y;
            positions[i] = pos;
            //qDebug("Person %f %f", person.position.x, person.position.y);

            peoplespam.people.push_back(person);
        }

        pub.publish(peoplespam);
        sleep(1);
    }
    qDebug("SpamPeople finishing");
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
    qDebug("Subscriber finishing");
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        qWarning("Usage: %s <int_robot_id>", argv[0]);
        return -1;
    }
    QString id(argv[1]);
    bool ok;
    robotID = id.toInt(&ok);
    if (!ok)
    {
        qWarning("Usage: %s <int_robot_id>. Robot ID must be a number", argv[0]);
        return -1;
    }

    ros::init(argc, argv, "rosmapinterface_fakerobot");
    while (!ros::master::check())
    {
        qDebug("Waiting for ros master");
        sleep(1);
    }

    ros::NodeHandle handle;
    QtConcurrent::run(spamBeacon, handle);
    QtConcurrent::run(spamFakeLaser, handle);
    QtConcurrent::run(spamPeople, handle);
    listenToWaypoints(handle);
    return 0;
}
