#include "lasersubscriber.h"
#include <tf/transform_listener.h>

using namespace MapAbstraction;

LaserSubscriber::LaserSubscriber(int robotID, QSharedPointer<ros::NodeHandle> handle)
    : mID(robotID)
{
    QString subscriberTopic = "/robot/" + QString::number(robotID) + "/laser_scan";
    mSub = handle->subscribe<sensor_msgs::LaserScan>(
                subscriberTopic.toStdString(), 100, &LaserSubscriber::laserScanCallback, this);
}

void LaserSubscriber::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    //TODO - add base transform (base_laser_link -> base_link) if needed to (x, y)

    tf::StampedTransform transform;
    bool doTransform = true;
    try
    {
        laserToBase.lookupTransform(msg->header.frame_id, "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        //No transform, can't produce meaningful data
        doTransform = false;
    }

    int pointCount = msg->ranges.size();
    float angle = msg->angle_min;
    LaserScanPoints points;
    for (int i = 0; i < pointCount; ++i)
    {
        float distance = msg->ranges.at(i);
        if (isnan(distance))
            continue; //Not a number

        if (msg->range_max < distance || msg->range_min > distance || 0 > distance)
            continue; //such values should be discarded according to msg spec

        if (angle > msg->angle_max)
        {
            ROS_WARN("Error in laser scan data conversion or data corrupted");
            break; //Shouldn't happen, but better to skip such doubtful data
        }

        float x = distance * cos(angle);
        float y = distance * sin(angle);

        if (doTransform)
        {
            geometry_msgs::PointStamped laser_point, base_point;
            laser_point.header.frame_id = msg->header.frame_id;
            laser_point.header.stamp = ros::Time();
            laser_point.point.x = x;
            laser_point.point.y = y;
            laser_point.point.z = 0;
            laserToBase.transformPoint("/base_link", laser_point, base_point);

            LaserScanPoint point(base_point.point.x, base_point.point.y);
            //qDebug("Point |%f|%f|", x, y);
            points.append(point);
        }
        else
        {
            LaserScanPoint point(x, y);
            points.append(point);
        }

        angle += msg->angle_increment;
    }

    if (points.empty())
        return;

    emit laserScanUpdate(mID, points);
}
