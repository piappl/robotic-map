#ifndef PEOPLESUBSCRIBER_H
#define PEOPLESUBSCRIBER_H

#include <QSharedPointer>
#include <map/DynamicObject.h>
#include <ros/ros.h>
#include <people_msgs/People.h>

class PeopleSubscriber : public QObject
{
    Q_OBJECT
signals:
    void peopleUpdate(int id, MapAbstraction::DynamicObjects people);

public:
    PeopleSubscriber(int id, QSharedPointer<ros::NodeHandle> handle);
    void peopleCallback(const people_msgs::People::ConstPtr &msg);

private:
    int mID;
    ros::Subscriber mSub;
};
typedef QSharedPointer<PeopleSubscriber> PeopleSubscriberPtr;

#endif //PEOPLESUBSCRIBER_H
