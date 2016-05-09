#include <QtConcurrent/QtConcurrent>
#include <string>
#include <map/RobotStates.h>
#include <map/MapRobotObject.h>
#include <map/MapReceiver.h>
#include "robotlistener.h"

using namespace MapAbstraction;

namespace
{
    MapAbstraction::MapObject::LocalizationType localization(int type)
    {
        switch (type)
        {
        case rosmapinterface::RobotInformation::TYPE_GLOBAL: return MapAbstraction::MapObject::Global;
        case rosmapinterface::RobotInformation::TYPE_LOCAL_ABSOLUTE: return MapAbstraction::MapObject::LocalAbsolute;
        case rosmapinterface::RobotInformation::TYPE_LOCAL_RELATIVE: return MapAbstraction::MapObject::LocalRelative;
        case rosmapinterface::RobotInformation::TYPE_NO_POSITION: return MapAbstraction::MapObject::None;
        default: return MapAbstraction::MapObject::None;
        }
    }

    MapAbstraction::MapObject::LocalizationMode mode(MapAbstraction::MapObject::LocalizationType type)
    {
        return type == MapAbstraction::MapObject::Global ? MapAbstraction::MapObject::Automatic
                                                         : MapAbstraction::MapObject::Assisted;
    }
}

RobotListener::RobotListener(MapWrapPtr mapWrap, QSharedPointer<ros::NodeHandle> handle)
    : mMapWrap(mapWrap), mHandle(handle)
{
}

void RobotListener::start(MapWrapPtr mapWrap, QSharedPointer<ros::NodeHandle> handle, RobotIDListener *l)
{
    QtConcurrent::run(&RobotListener::startNewThread, mapWrap, handle, l);
}

void RobotListener::startNewThread(MapWrapPtr mapWrap, QSharedPointer<ros::NodeHandle> handle, RobotIDListener *l)
{
    RobotListener listener(mapWrap, handle);
    listener.runListener(l);
}

void RobotListener::beaconCallback(const rosmapinterface::RobotInformation::ConstPtr& msg)
{
    bool hasOrientation = true;
    double orientation = msg->theta;
    QString name(msg->robot_type.data());
    QString type(msg->robot_name.data());
    QString description(msg->description.data());
    RobotState state = RobotStateNormal;
    MapAbstraction::MapObject::LocalizationType lt = localization(msg->localization_type);
    MapAbstraction::MapObject::LocalizationMode lm = mode(lt);
    MapAbstraction::GeoCoords coords(msg->x, msg->y);
    int robotID = msg->robot_id;
    int consoleID = 0;
    MapAbstraction::MapRobotObjectPtr obj(
                new MapAbstraction::MapRobotObject(
                    coords, orientation, state,
                    type, name, description,
                    robotID, consoleID,
                    lt, lm));
    obj->setOrientationAvailable(hasOrientation);
    emit robotUpdate(obj);

    //robot of robotID is present
    emit receivingRobotID(robotID);
    subscribeLaserData(robotID);
    subscribePeopleData(robotID);
}

void RobotListener::runListener(RobotIDListener *l)
{
    connect(this, SIGNAL(receivingRobotID(int)), l, SLOT(robotDetected(int)));
    qDebug("RobotInformation listener running");
    connect(this, SIGNAL(robotUpdate(MapAbstraction::MapRobotObjectPtr)),
            mMapWrap.data(), SLOT(updateRobot(MapAbstraction::MapRobotObjectPtr)), Qt::QueuedConnection);

    std::string subscriberTopic = "robot_introduction";
    ros::Subscriber subs = mHandle->subscribe<rosmapinterface::RobotInformation>(
                subscriberTopic, 100, &RobotListener::beaconCallback, this);


    ros::spin();
}

void RobotListener::subscribeLaserData(int robotID)
{
    if (mLaserSubscribers.contains(robotID))
        return;

    LaserSubscriberPtr laserSub(new LaserSubscriber(robotID, mHandle));
    connect(laserSub.data(), SIGNAL(laserScanUpdate(int, MapAbstraction::LaserScanPoints)),
            mMapWrap->mapReceiver().data(),
            SLOT(updateLaserPointsCloud(int, MapAbstraction::LaserScanPoints)), Qt::QueuedConnection);

    mLaserSubscribers.insert(robotID, laserSub);
}

void RobotListener::subscribePeopleData(int robotID)
{
    if (mPeopleSubscribers.contains(robotID))
        return;

    PeopleSubscriberPtr pplSub(new PeopleSubscriber(robotID, mHandle));
    connect(pplSub.data(), SIGNAL(peopleUpdate(int, MapAbstraction::DynamicObjects)),
            mMapWrap->mapReceiver().data(),
            SLOT(updateDynamicObjects(int, MapAbstraction::DynamicObjects)), Qt::QueuedConnection);

    mPeopleSubscribers.insert(robotID, pplSub);
}
