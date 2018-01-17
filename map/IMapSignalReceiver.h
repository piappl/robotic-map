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

        //Adds a file containing a Marble-accepted format of geometry. Adds or updates if present.
        virtual void updateFileGeometry(const QString &filename) = 0;

        //Removes data associated with the key (such as key = filename of updateFileGeometry)
        virtual void removeGeometry(const QString &key) = 0;
     };
}

#endif // MAPCOMMUNICATIONMANAGER_H
