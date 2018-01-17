#ifndef ROBOTGEOREFERENCES_H
#define ROBOTGEOREFERENCES_H

#include <QMap>
#include "MapAbstractionFwd.h"
#include "InternalTypesFwd.h"
#include "GeoObjectID.h"

namespace MapAbstraction
{   //This class owns all LocalizedObjectData
    class RobotGeoReferences
    {
    public:
        GeoObjectID addOrGetGeoReference(qint32 key);
        GeoObjectID getGeoReference(qint32 key) const;

        MapRobotObjectPtr getLocalizedRobot(const GeoObjectID &id) const;
        MapObjectPtr addLocalizedObject(const GeoObjectID &id, MapObjectPtr info);

    private:
        QMap<qint32, GeoObjectID> mMap;
        QMap<GeoObjectID, MapObjectPtr> mLocalizedMap;
    };
}

#endif // ROBOTGEOREFERENCES_H
