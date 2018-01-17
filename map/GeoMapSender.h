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

        //ID is an external robotID (not map ID)
        void skillTriggered(int targetRobotID, QString skill);

        //Coords here are used to store local coordinates
        void robotPositioned(int targetRobotID, MapAbstraction::GeoCoords position, MapAbstraction::Orientation orientation);

        //This signal is sent if operator position is available (i.e. it has been manually localized).
        //Robot position is in meters of distance in x axis and y axis (compass-enabled receivers use x=east and y=north).
        //Operator position is absolute (in radians)
        void robotPositionRelativeToOperator(int targetRobotID, MapAbstraction::GeoCoords position,
                                             MapAbstraction::GeoCoords operatorAbsolutePosition);

        //Request connection with robot (for connection-oriented protocols,
        //for ROS robots, the handler should always respond with success (see IMapSignalReceiver)
        //connect = false is request for a disconnect.
        void requestConnect(int targetRobotID, bool connect);
    };
}

#endif // GEOMAPSENDER_H
