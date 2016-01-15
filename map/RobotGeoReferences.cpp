#include "MapRobotObject.h"
#include "RobotGeoReferences.h"

namespace MapAbstraction
{
    GeoObjectID RobotGeoReferences::addOrGetGeoReference(qint32 key)
    {
        GeoObjectID id;
        if (!mMap.contains(key))
        {
            id = GeoReferenceFactory::createGeoObjectId();
            mMap.insert(key, id);
        }
        else
        {
            return getGeoReference(key);
        }
        return id;
    }

    GeoObjectID RobotGeoReferences::getGeoReference(qint32 key) const
    {
        return mMap.value(key);
    }

    MapRobotObjectPtr RobotGeoReferences::getLocalizedObject(const GeoObjectID &id) const
    {
        return mLocalizedMap.value(id);
    }

    MapRobotObjectPtr RobotGeoReferences::addLocalizedObject(const GeoObjectID &id,
                                                MapRobotObjectPtr info)
    {
        MapRobotObjectPtr robot(dynamic_cast<MapRobotObject*>(info->Clone()));
        if (!mLocalizedMap.contains(id))
        {
            mLocalizedMap[id] = robot;
        }
        else
        {
            mLocalizedMap.insert(id, robot);
        }
        return robot;
    }
}
