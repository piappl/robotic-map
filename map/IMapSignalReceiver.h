#ifndef MAPCOMMUNICATIONMANAGER_H
#define MAPCOMMUNICATIONMANAGER_H

#include "MapObjectsFwd.h"
#include "GeoObjectID.h"

namespace MapAbstraction
{
    class IMapSignalReceiver
    {
    public slots:
        // Adds an object to the map. If it exists (by unique ID), modifies it accordingly
        // Id is provided by caller to manipulate the object in the future
        virtual void updatePlacemark(GeoObjectID id, MapObjectPtr newObject) = 0;

        // A response to requestConnect (see GeoMapSender)
        virtual void connectedToRobot(int robotID, bool connected) = 0;
     };
}

#endif // MAPCOMMUNICATIONMANAGER_H
