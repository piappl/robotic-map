#ifndef GEOMAPSENDER_H
#define GEOMAPSENDER_H

#include <QSharedPointer>
#include "MapAbstractionFwd.h"
#include "GeoCoords.h"
#include "Orientation.h"

namespace MapAbstraction
{
    class GeoMapSender : public QObject
    {
        Q_OBJECT
    signals:
        //ID is an external robotID (not map ID), path is a QList of WaypointObjectPtr
        void mapPathCreated(int targetRobotID, MapAbstraction::MapPath path);

        //Coords here are used to store local coordinates
        void robotPositioned(int targetRobotID, MapAbstraction::GeoCoords position, MapAbstraction::Orientation orientation);

        //Request connection with robot (for connection-oriented protocols,
        //for ROS robots, the handler should always respond with success (see IMapSignalReceiver)
        //connect = false is request for a disconnect.
        void requestConnect(int targetRobotID, bool connect);
    };
}

#endif // GEOMAPSENDER_H
