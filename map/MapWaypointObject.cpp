#include "MapWaypointObject.h"

namespace MapAbstraction
{
    MapWaypointObject::MapWaypointObject(GeoCoords coords, short number)
        : MapOrderedObject(coords, number),
          mReached(false)
    {
    }

    MapObject *MapWaypointObject::Clone() const
    {
        return new MapWaypointObject(*this);
    }

    PlacemarkType MapWaypointObject::category() const
    {
        return PlacemarkWaypoint;
    }

    bool MapWaypointObject::compare(const MapObject &other) const
    {
        bool ret = false;
        const MapWaypointObject* otherWaypoint = dynamic_cast<const MapWaypointObject*>(&other);
        if (otherWaypoint)
        {
            ret = MapOrderedObject::compare(*otherWaypoint)
                    && reached() == otherWaypoint->reached();
        }
        return ret;
    }

    void MapWaypointObject::setReached(bool reached)
    {
        mReached = reached;
    }

    bool MapWaypointObject::reached() const
    {
        return mReached;
    }
}
