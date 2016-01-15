#ifndef MAPPOLYGONNODEOBJECT_H
#define MAPPOLYGONNODEOBJECT_H

#include "MapOrderedObject.h"

namespace MapAbstraction
{   //Somewhat similar to a waypoint as for now
    class MapPolygonNodeObject : public MapOrderedObject
    {
    public:
        MapPolygonNodeObject(GeoCoords coords, short order);
        MapObject *Clone() const;
        PlacemarkType category() const;
        QString displayText() const;

    protected:
        bool compare(const MapObject &other) const;
    };
}

#endif // MAPPOLYGONNODEOBJECT_H
