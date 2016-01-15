#ifndef MAPWAYPOINTOBJECT_H
#define MAPWAYPOINTOBJECT_H

#include "MapOrderedObject.h"

namespace MapAbstraction
{
    class MapWaypointObject : public MapOrderedObject
    { 
    public:
        MapWaypointObject(GeoCoords coords, short number);
        MapObject *Clone() const;
        PlacemarkType category() const;
        void setReached(bool reached);
        bool reached() const;

    private:
        bool compare(const MapObject &other) const;
        bool mReached;
    };
}

#endif // MAPWAYPOINTOBJECT_H
