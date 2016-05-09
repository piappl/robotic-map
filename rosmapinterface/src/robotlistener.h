#ifndef ROBOTLISTENER_H
#define ROBOTLISTENER_H

#include <QObject>
#include <QMap>
#include <map/MapObjectsFwd.h>
#include <map/MapWrap.h>
#include "lasersubscriber.h"
#include "peoplesubscriber.h"
#include "rosmapinterface/RobotInformation.h"

//This interface is used to be noted of new robots in communication zone, which are broadcasting.
//Right now callback only used to advertise waypoint topic by the waypoint sender.
class RobotIDListener : public QObject
{
    Q_OBJECT
public:
    virtual ~RobotIDListener() {}
public slots:
    void robotDetected(int id) { onRobotDetected(id); }
private:
    virtual void onRobotDetected(int id) = 0;
};

class RobotListener : public QObject
{
    Q_OBJECT
signals:
    void robotUpdate(MapAbstraction::MapRobotObjectPtr newState);
    void receivingRobotID(int id);

public:
    RobotListener(MapAbstraction::MapWrapPtr mapWrap, QSharedPointer<ros::NodeHandle> handle);
    static void start(MapAbstraction::MapWrapPtr mapWrap,
                      QSharedPointer<ros::NodeHandle> handle,
                      RobotIDListener *l);

private:
    static void startNewThread(MapAbstraction::MapWrapPtr mapWrap,
                               QSharedPointer<ros::NodeHandle> handle,
                               RobotIDListener *l); //This is ran in new thread

    void beaconCallback(const rosmapinterface::RobotInformation::ConstPtr& msg);
    void runListener(RobotIDListener *l);
    void subscribeLaserData(int robotID);
    void subscribePeopleData(int robotID);

    QMap<int, LaserSubscriberPtr> mLaserSubscribers;
    QMap<int, PeopleSubscriberPtr> mPeopleSubscribers;
    MapAbstraction::MapWrapPtr mMapWrap;
    QSharedPointer<ros::NodeHandle> mHandle;
};

#endif //ROBOTLISTENER_H
