#ifndef MAPABSTRACTIONFWD_H
#define MAPABSTRACTIONFWD_H

#include <QSharedPointer>
#include "MapObjectsFwd.h"

namespace MapAbstraction
{
    class RobotHeartbeat;
    typedef QSharedPointer<RobotHeartbeat> RobotHeartbeatPtr;

    class RobotHeartState;
    typedef QSharedPointer<RobotHeartState> RobotHeartStatePtr;

    class RobotGeoReferences;
    typedef QSharedPointer<RobotGeoReferences> RobotGeoReferencesPtr;

    typedef QList<MapWaypointObjectPtr> MapPath;

    class GeoMapSender;
    typedef QSharedPointer<GeoMapSender> GeoMapSenderPtr;

    class GeoLocalMapReceiver;
    typedef QSharedPointer<GeoLocalMapReceiver> GeoLocalMapReceiverPtr;

    class RegionData;
    typedef QSharedPointer<RegionData> RegionDataPtr;
}

#endif // MAPABSTRACTIONFWD_H
