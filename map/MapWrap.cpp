#include <QQuickItem>
#include "MapWrap.h"
#include "MapFactory.h"
#include "MapRobotObject.h"
#include "MapPlaceObject.h"
#include "IGeoMap.h"
#include "RobotHeartbeat.h"
#include "RobotGeoReferences.h"

using namespace MapAbstraction;

MapWrap::MapWrap(QQuickItem *viewMap, int consoleID)
    : mConsoleID(consoleID),
      mGeoRefs(new RobotGeoReferences())
{
    MapFactory factory;
    mMap = factory.createMapItem(viewMap);
    mRobotHeartbeat.reset(new RobotHeartbeat(mMap, mGeoRefs, mConsoleID));
}

GeoMapSenderPtr MapWrap::sender() const
{
    return mMap->sender();
}

GeoLocalMapReceiverPtr MapWrap::localMapReceiver() const
{
    return mMap->localMapReceiver();
}

void MapWrap::robotConnected(int robotID, bool connected)
{
    mMap->connectedToRobot(robotID, connected);
}

void MapWrap::updateConsole(MapPlaceObjectPtr consoleInfo)
{
    bool hasIdentifier = false;
    int identifier = consoleInfo->name().toInt(&hasIdentifier);
    if (!hasIdentifier)
    {
        qWarning("Console has no valid identifier, cannot proceed");
        return;
    }

    GeoObjectID id = mGeoRefs->addOrGetGeoReference(identifier);
    MapPlaceObjectPtr localizedData = mGeoRefs->addLocalizedObject(id, consoleInfo).staticCast<MapPlaceObject>();
    mMap->updatePlacemark(id, localizedData);
}

void MapWrap::updateFileGeometry(const QString &sourceFile)
{
    mMap->updateFileGeometry(sourceFile);
}

void MapWrap::removeGeometry(const QString &key)
{
    mMap->removeGeometry(key);
}

void MapWrap::updateRobot(MapRobotObjectPtr robotInfo)
{
    GeoObjectID id = mGeoRefs->addOrGetGeoReference(robotInfo->robotID());
    MapRobotObjectPtr localizedData = mGeoRefs->addLocalizedObject(id, robotInfo).staticCast<MapRobotObject>();
    //qDebug("ConsoleID = %d, state = %d, mConsoleID = %d", localizedData->consoleID(), localizedData->state(), mConsoleID);
    if (localizedData->consoleID() == mConsoleID && localizedData->state() == RobotStateNormal)
        localizedData->setState(RobotStateConnected);
    mRobotHeartbeat->robotActive(id);
    mMap->updatePlacemark(id, localizedData);
}

void MapWrap::robotAvailabilityChanged(int robotId, bool available)
{
    //qWarning("robotAvailability changed %d, %s", robotId, available ? "available" : "not available");
    GeoObjectID id;
    if (available)
    {
        id = mGeoRefs->addOrGetGeoReference(robotId);
        mRobotHeartbeat->robotActive(id);
    }
    else
    {
        id = mGeoRefs->getGeoReference(robotId);
        if (!id.isNull())
        {
            mRobotHeartbeat->robotDisappeared(id);
        }
    }
}

