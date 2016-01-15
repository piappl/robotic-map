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

        MapRobotObjectPtr getLocalizedObject(const GeoObjectID &id) const;
        MapRobotObjectPtr addLocalizedObject(const GeoObjectID &id, MapRobotObjectPtr info);

    private:
        QMap<qint32, GeoObjectID> mMap;
        QMap<GeoObjectID, MapRobotObjectPtr> mLocalizedMap;
    };
}

#endif // ROBOTGEOREFERENCES_H
