#ifndef LASERSUBSCRIBER_H
#define LASERSUBSCRIBER_H

#include <QSharedPointer>
#include <map/LaserScanPoint.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

class LaserSubscriber : public QObject
{
    Q_OBJECT
signals:
    void laserScanUpdate(int id, MapAbstraction::LaserScanPoints points);

public:
    LaserSubscriber(int id, QSharedPointer<ros::NodeHandle> handle);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

private:
    int mID;
    ros::Subscriber mSub;
    tf::TransformListener laserToBase;
};
typedef QSharedPointer<LaserSubscriber> LaserSubscriberPtr;

#endif //LASERSUBSCRIBER_H
