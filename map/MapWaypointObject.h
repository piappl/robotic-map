#ifndef MAPWAYPOINTOBJECT_H
#define MAPWAYPOINTOBJECT_H

#include "MapOrderedObject.h"
#include "Orientation.h"

namespace MapAbstraction
{
    class MapWaypointObject : public MapOrderedObject
    { 
    public:
        MapWaypointObject(GeoCoords coords, short number, Orientation orientation = 0);
        MapObject *Clone() const;
        PlacemarkType category() const;
        void setReached(bool reached);
        void setOrientation(Orientation orientation);
        bool reached() const;
        Orientation orientation() const;

    private:
        bool compare(const MapObject &other) const;
        bool mReached;
        Orientation mOrientation;
    };
}

#endif // MAPWAYPOINTOBJECT_H
