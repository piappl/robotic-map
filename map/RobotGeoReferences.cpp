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

    MapRobotObjectPtr RobotGeoReferences::getLocalizedRobot(const GeoObjectID &id) const
    {
        Q_ASSERT(mLocalizedMap.value(id) && PlacemarkRobot == mLocalizedMap.value(id)->category());
        return mLocalizedMap.value(id).staticCast<MapRobotObject>();
    }

    MapObjectPtr RobotGeoReferences::addLocalizedObject(const GeoObjectID &id, MapObjectPtr info)
    {
        MapObjectPtr object(info->Clone());
        if (!mLocalizedMap.contains(id))
        {
            mLocalizedMap[id] = object;
        }
        else
        {
            mLocalizedMap.insert(id, object);
        }
        return object;
    }
}
